/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_dma2d_utils.h"
#include "esp_cache.h"
#include "esp_timer.h"
#include "esp_lvgl_port_dma2d.h"

static char *TAG = "convert";

#define ALIGN_UP(num, align)      (((num) + ((align) - 1)) & ~((align) - 1))
#define ALIGN_DOWN(num, align)    ((num) & (~((align) - 1)))

#define DMA2D_LOG_CYCLE           (21)

static size_t get_cache_line_size(const void *addr)
{
    esp_err_t ret = ESP_FAIL;
    size_t cache_line_size = 0;
    uint32_t heap_caps = esp_ptr_external_ram(addr) ? MALLOC_CAP_SPIRAM : MALLOC_CAP_INTERNAL;
    ret = esp_cache_get_alignment(heap_caps, &cache_line_size);
    return ret == ESP_OK ? cache_line_size : 0;
}

static int make_addr_and_size_aligned(uint8_t **addr, uint32_t *size)
{
    uint8_t *buf = *addr;
    uint16_t cache_line = get_cache_line_size(buf);
    uint8_t *buf_alignd = (uint8_t *) ALIGN_DOWN((uint32_t)buf, cache_line);
    uint32_t len = *size;
    len += (buf - buf_alignd);
    uint32_t size_align = ALIGN_UP(len, cache_line);

    *addr = buf_alignd;
    *size = size_align;
    return ESP_OK;
}

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

esp_err_t dma2d_color_convert(dma2d_pic_config_t *src, dma2d_pic_config_t *dest)
{
    if (src->pic_buf == NULL || dest->pic_buf == NULL) {
        return ESP_FAIL;
    }

    if (src->format.pixel_format == COLOR_PIXEL_RGB666 || dest->format.pixel_format == COLOR_PIXEL_RGB666) {
        return ESP_FAIL;
    }

    int64_t start = esp_timer_get_time();
    dma2d_m2m_trans_config_t m2m_trans_config = {0};

    if (dma2d_m2m_init() != ESP_OK) {
        return ESP_FAIL;
    }

    int dsc_size = 64;
    size_t cache_line_size = 0;
    esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &cache_line_size);
    dma2d_descriptor_t *tx_dsc = (dma2d_descriptor_t *)heap_caps_aligned_calloc(cache_line_size, 1, dsc_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
    dma2d_descriptor_t *rx_dsc = (dma2d_descriptor_t *)heap_caps_aligned_calloc(cache_line_size, 1, dsc_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
    if (tx_dsc == NULL || rx_dsc == NULL) {
        return ESP_FAIL;
    }

    dma2d_csc_config_t dma2d_tx_csc = {0};
    uint16_t src_pbyte = src->format.pixel_format == COLOR_PIXEL_RGB888 ? 3 : 2;
    uint16_t dest_pbyte = dest->format.pixel_format == COLOR_PIXEL_RGB888 ? 3 : 2;

    if (src->format.pixel_format == COLOR_PIXEL_RGB888 && dest->format.pixel_format == COLOR_PIXEL_RGB565) {
        dma2d_tx_csc.tx_csc_option = DMA2D_CSC_TX_RGB888_TO_RGB565;
    } else if (src->format.pixel_format == COLOR_PIXEL_RGB565 && dest->format.pixel_format == COLOR_PIXEL_RGB888) {
        dma2d_tx_csc.tx_csc_option = DMA2D_CSC_TX_RGB565_TO_RGB888;
    }

    uint8_t *src_alignd = src->pic_buf + src->offset_y * src_pbyte * src->pic_w;
    uint32_t src_size_alignd = src->pic_w * src->block_h * src_pbyte;
    make_addr_and_size_aligned(&src_alignd, &src_size_alignd);

    uint8_t *dest_alignd = dest->pic_buf + dest->offset_y * dest_pbyte * dest->pic_w;
    uint32_t dest_size_alignd = dest->pic_w * dest->block_h * dest_pbyte;
    make_addr_and_size_aligned(&dest_alignd, &dest_size_alignd);

    // Writeback TX buffers and Invalidate RX buffers
    esp_cache_msync((void *)src_alignd, src_size_alignd, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
    esp_cache_msync((void *)dest_alignd, dest_size_alignd, ESP_CACHE_MSYNC_FLAG_INVALIDATE);

    SemaphoreHandle_t sem = xSemaphoreCreateCounting(1, 0);

    dma2d_transfer_ability_t transfer_ability_config = {
        .data_burst_length = DMA2D_DATA_BURST_LENGTH_128,
        .desc_burst_en = true,
        .mb_size = DMA2D_MACRO_BLOCK_SIZE_NONE,
    };

    // DMA description preparation
    dma2d_link_dscr_init((uint32_t *)tx_dsc, NULL, (void *)src->pic_buf,
                         src->pic_w, src->pic_h,
                         src->block_w, src->block_h,
                         1, 1, dma2d_desc_pixel_format_to_pbyte_value(src->format),
                         DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE, src->offset_x, src->offset_y);
    dma2d_link_dscr_init((uint32_t *)rx_dsc, NULL, (void *)dest->pic_buf,
                         dest->pic_w, dest->pic_h,
                         dest->block_w, dest->block_h,
                         0, 1, dma2d_desc_pixel_format_to_pbyte_value(dest->format),
                         DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE, dest->offset_x, dest->offset_y);
    // Writeback the DMA descriptors
    esp_cache_msync((void *)tx_dsc, dsc_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    esp_cache_msync((void *)rx_dsc, dsc_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);

    // Construct dma2d_m2m_trans_config_t structure
    m2m_trans_config.tx_desc_base_addr = (intptr_t)tx_dsc;
    m2m_trans_config.rx_desc_base_addr = (intptr_t)rx_dsc;
    m2m_trans_config.trans_eof_cb = dma2d_m2m_suc_eof_event_cb;
    m2m_trans_config.user_data = (void *)sem;
    m2m_trans_config.transfer_ability_config = &transfer_ability_config;
    m2m_trans_config.tx_csc_config = &dma2d_tx_csc;

    dma2d_m2m(&m2m_trans_config);
    xSemaphoreTake(sem, portMAX_DELAY);

    free(tx_dsc);
    free(rx_dsc);
    vSemaphoreDelete(sem);
    dma2d_m2m_deinit();

    int duration = esp_timer_get_time() - start;
    uint32_t data_size = dest->block_w * dest->block_h * (src_pbyte + dest_pbyte);
    float speed = data_size * 0.95367 / duration;
    static uint32_t count = 0;
    if (count++ % DMA2D_LOG_CYCLE == 0) {
        ESP_LOGD(TAG, "dma2d %s: %d ms, (x:%ld, y:%ld)[w:%ld, h:%ld][spx:%d, dpx:%d], %.1f MB/s", src_pbyte == dest_pbyte ? "memcpy" : "convert",
                 duration / 1000, dest->offset_x, dest->offset_y, dest->block_w, dest->block_h, src_pbyte, dest_pbyte, speed);
    }
    return ESP_OK;
}
