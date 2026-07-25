/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Shared draw-buffer cache handlers for ESP PPA/DMA2D units
 */

#include "lv_draw_esp_buf.h"

#if LV_USE_PPA || LV_USE_ESP_DMA2D

#include "draw/lv_draw_buf_private.h"
#include "esp_cache.h"

static void invalidate_cache(const lv_draw_buf_t *draw_buf, const lv_area_t *area);

void LV_ATTRIBUTE_FAST_MEM lv_draw_esp_buf_init_handlers(void)
{
    lv_draw_buf_handlers_t *handlers = lv_draw_buf_get_handlers();
    handlers->invalidate_cache_cb = invalidate_cache;
}

static void LV_ATTRIBUTE_FAST_MEM invalidate_cache(const lv_draw_buf_t *draw_buf, const lv_area_t *area)
{
    LV_UNUSED(area);
    esp_cache_msync(draw_buf->data, draw_buf->data_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
}

#endif /* LV_USE_PPA || LV_USE_ESP_DMA2D */
