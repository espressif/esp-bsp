/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "dma2d_utils.h"
#include "esp_cache.h"
#include "esp_timer.h"

static char *TAG = "convert";

extern int make_addr_and_size_align(uint8_t **addr, uint32_t *size);

static bool IRAM_ATTR dma2d_m2m_suc_eof_event_cb(void *user_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    SemaphoreHandle_t sem = (SemaphoreHandle_t)user_data;
    xSemaphoreGiveFromISR(sem, &xHigherPriorityTaskWoken);
    return (xHigherPriorityTaskWoken == pdTRUE);
}

static void dma2d_link_dscr_init(uint32_t *head, uint32_t *next, void *buf_ptr,
                                 uint32_t ha, uint32_t va, uint32_t hb, uint32_t vb,
                                 uint32_t eof, uint32_t en_2d, uint32_t pbyte, uint32_t mod,
                                 uint32_t bias_x, uint32_t bias_y)
{
    dma2d_descriptor_t *dma2d = (dma2d_descriptor_t *)head;
    memset(dma2d, 0, sizeof(dma2d_descriptor_t));
    dma2d->owner = DMA2D_DESCRIPTOR_BUFFER_OWNER_DMA;
    dma2d->suc_eof = eof;
    dma2d->dma2d_en = en_2d;
    dma2d->err_eof = 0;
    dma2d->hb_length = hb;
    dma2d->vb_size = vb;
    dma2d->pbyte = pbyte;
    dma2d->ha_length = ha;
    dma2d->va_size = va;
    dma2d->mode = mod;
    dma2d->y = bias_y;
    dma2d->x = bias_x;
    dma2d->buffer = buf_ptr;
    dma2d->next = (dma2d_descriptor_t *)next;
}

esp_err_t dma2d_color_convert(color_space_pixel_format_t src_format, color_space_pixel_format_t dest_format,
                        uint32_t block_w, uint32_t block_h,
                        const uint8_t *src_pic_buf, uint32_t src_offx, uint32_t src_offy, uint32_t src_pic_w, uint32_t src_pic_h,
                        const uint8_t *dest_pic_buf, uint32_t dest_offx, uint32_t dest_offy, uint32_t dest_pic_w, uint32_t dest_pic_h)
{
    assert(src_pic_buf);
    assert(dest_pic_buf);

    int64_t start = esp_timer_get_time();
    dma2d_m2m_trans_config_t m2m_trans_config = {0};

    if (dma2d_m2m_init() != ESP_OK) {
        return ESP_FAIL;
    }

    dma2d_descriptor_t *tx_dsc;
    dma2d_descriptor_t *rx_dsc;

    // Descriptor and buffer address and size should aligned to 64 bytes (the cacheline size alignment restriction) to be used by CPU
    dma2d_descriptor_t *tx_link_buffer = (dma2d_descriptor_t *)heap_caps_aligned_calloc(64, 1, 64, MALLOC_CAP_SPIRAM);
    dma2d_descriptor_t *rx_link_buffer = (dma2d_descriptor_t *)heap_caps_aligned_calloc(64, 1, 64, MALLOC_CAP_SPIRAM);
    assert(tx_link_buffer);
    assert(rx_link_buffer);
    tx_dsc = (dma2d_descriptor_t *)((uint32_t)tx_link_buffer);
    rx_dsc = (dma2d_descriptor_t *)((uint32_t)rx_link_buffer);

    dma2d_csc_config_t m2m_dma2d_tx_csc = {0};
    uint16_t src_pbyte = src_format.pixel_format == COLOR_PIXEL_RGB888 ? 3 : 2;
    uint16_t dest_pbyte = dest_format.pixel_format == COLOR_PIXEL_RGB888 ? 3 : 2;

    if (src_format.pixel_format == COLOR_PIXEL_RGB888 && dest_format.pixel_format == COLOR_PIXEL_RGB565) {
        m2m_dma2d_tx_csc.tx_csc_option = DMA2D_CSC_TX_RGB888_TO_RGB565;
    } else if (src_format.pixel_format == COLOR_PIXEL_RGB565 && dest_format.pixel_format == COLOR_PIXEL_RGB888) {
        m2m_dma2d_tx_csc.tx_csc_option = DMA2D_CSC_TX_RGB565_TO_RGB888;
    }

    const uint8_t *prtx = src_pic_buf;
    const uint8_t *prrx = dest_pic_buf;

    uint8_t *src_alignd = src_pic_buf + src_offy * src_pbyte * src_pic_w;
    uint32_t src_size_alignd = src_pic_w * block_h * src_pbyte;
    make_addr_and_size_align(&src_alignd, &src_size_alignd);

    uint8_t *dest_alignd = dest_pic_buf + dest_pic_w * dest_offy * dest_pbyte;
    uint32_t dest_size_alignd = dest_pic_w * block_h * dest_pbyte;
    make_addr_and_size_align(&dest_alignd, &dest_size_alignd);

    // Writeback TX buffers and Invalidate RX buffers
    esp_cache_msync((void *)src_alignd, src_size_alignd, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    esp_cache_msync((void *)dest_alignd, dest_size_alignd, ESP_CACHE_MSYNC_FLAG_INVALIDATE);

    SemaphoreHandle_t counting_sem = xSemaphoreCreateCounting(1, 0);

    dma2d_transfer_ability_t transfer_ability_config = {
        .data_burst_length = DMA2D_DATA_BURST_LENGTH_128,
        .desc_burst_en = true,
        .mb_size = DMA2D_MACRO_BLOCK_SIZE_NONE,
    };

    // DMA description preparation
    dma2d_link_dscr_init((uint32_t *)tx_dsc, NULL, (void *)prtx,
                            src_pic_w, src_pic_h,
                            block_w, block_h,
                            1, 1, dma2d_desc_pixel_format_to_pbyte_value(src_format),
                            DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE, src_offx, src_offy);
    dma2d_link_dscr_init((uint32_t *)rx_dsc, NULL, (void *)prrx,
                            dest_pic_w, dest_pic_h,
                            block_w, block_h,
                            0, 1, dma2d_desc_pixel_format_to_pbyte_value(dest_format),
                            DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE, dest_offx, dest_offy);
    // Writeback the DMA descriptors
    esp_cache_msync((void *)tx_dsc, 64, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    esp_cache_msync((void *)rx_dsc, 64, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);

    // Construct dma2d_m2m_trans_config_t structure
    m2m_trans_config.tx_desc_base_addr = (intptr_t)tx_dsc;
    m2m_trans_config.rx_desc_base_addr = (intptr_t)rx_dsc;
    m2m_trans_config.trans_eof_cb = dma2d_m2m_suc_eof_event_cb;
    m2m_trans_config.user_data = (void *)counting_sem;
    m2m_trans_config.transfer_ability_config = &transfer_ability_config;
    m2m_trans_config.tx_csc_config = &m2m_dma2d_tx_csc;

    dma2d_m2m(&m2m_trans_config);
    xSemaphoreTake(counting_sem, portMAX_DELAY);

    free(tx_link_buffer);
    free(rx_link_buffer);
    vSemaphoreDelete(counting_sem);
    dma2d_m2m_deinit();

    int duration = esp_timer_get_time() - start;
    uint32_t data_size = block_w * block_h * (src_pbyte + dest_pbyte);
    float speed = data_size * 0.95367 / duration;
    static uint32_t count = 0;
    if (count++ % 21 == 0) {
        ESP_LOGI(TAG, "dma2d %s: %d ms, (x:%ld, y:%ld)[w:%ld, h:%ld][spx:%d, dpx:%d], %.1f MB/s", src_pbyte == dest_pbyte ? "memcpy" : "convert",
            duration / 1000, dest_offx, dest_offy, block_w, block_h, src_pbyte, dest_pbyte, speed);
    }
    return ESP_OK;
}
