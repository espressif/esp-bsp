/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PPA draw buffer handlers
 */

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_PPA
#include LV_STDINT_INCLUDE
#include "draw/lv_draw_buf_private.h"

/*******************************************************************************
* Function definitions
*******************************************************************************/
static void invalidate_cache(const lv_draw_buf_t *draw_buf, const lv_area_t *area);

/*******************************************************************************
* Public API functions
*******************************************************************************/
void LV_ATTRIBUTE_FAST_MEM lv_draw_buf_ppa_init_handlers(void)
{
    lv_draw_buf_handlers_t *handlers = lv_draw_buf_get_handlers();
    handlers->invalidate_cache_cb = invalidate_cache;
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static void LV_ATTRIBUTE_FAST_MEM invalidate_cache(const lv_draw_buf_t *draw_buf, const lv_area_t *area)
{
    esp_cache_msync(draw_buf->data, draw_buf->data_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_TYPE_DATA);
}
#endif /* LV_USE_PPA */
