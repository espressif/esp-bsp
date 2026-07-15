/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PPA LVGL draw unit
 */

#pragma once

#include "lvgl_public.h"

#if LV_USE_PPA

#include "draw/lv_draw_private.h"
#include "display/lv_display_private.h"
#include "misc/lv_area_private.h"

#if LV_USE_PPA_RUNTIME_TUNING
#include "esp_err.h"
#endif

#endif /* LV_USE_PPA */

#ifdef __cplusplus
extern "C" {
#endif

#if LV_USE_PPA

/*******************************************************************************
* Public API
*******************************************************************************/

void lv_draw_ppa_init(void);
void lv_draw_ppa_deinit(void);

void lv_draw_ppa_fill(lv_draw_task_t *t, const lv_draw_fill_dsc_t *dsc,
                      const lv_area_t *coords);

void lv_draw_ppa_img(lv_draw_task_t *t, const lv_draw_image_dsc_t *dsc,
                     const lv_area_t *coords);

void lv_draw_ppa_layer(lv_draw_task_t *t, const lv_draw_image_dsc_t *dsc,
                       const lv_area_t *coords);

#if LV_USE_PPA_TILE_COMPOSER
bool lv_draw_ppa_layer_recolor_opa_supported(const lv_draw_image_dsc_t *dsc);
void lv_draw_ppa_layer_composite(lv_draw_task_t *t, const lv_draw_image_dsc_t *dsc,
                                 const lv_area_t *coords);
#endif

void lv_draw_ppa_border(lv_draw_task_t *t, const lv_draw_border_dsc_t *dsc,
                        const lv_area_t *coords);

void lv_draw_ppa_mask_rect(lv_draw_task_t *t, const lv_draw_mask_rect_dsc_t *dsc);

#if LV_USE_PPA_GRADIENT
void lv_draw_ppa_gradient(lv_draw_task_t *t, const lv_draw_fill_dsc_t *dsc, const lv_area_t *coords);
#endif

#if LV_USE_PPA_ROUND_FILL
void lv_draw_ppa_round_fill(lv_draw_task_t *t, const lv_draw_fill_dsc_t *dsc, const lv_area_t *coords);
#endif

#if LV_USE_PPA_LINE
void lv_draw_ppa_line(lv_draw_task_t *t, const lv_draw_line_dsc_t *dsc);
#endif

#if LV_USE_PPA_TRIANGLE
void lv_draw_ppa_triangle(lv_draw_task_t *t, const lv_draw_triangle_dsc_t *dsc);
#endif

#if LV_USE_PPA_ARC
void lv_draw_ppa_arc(lv_draw_task_t *t, const lv_draw_arc_dsc_t *dsc, const lv_area_t *coords);
#endif

#if 0
void lv_draw_ppa_label(lv_draw_task_t *t, const lv_draw_label_dsc_t *dsc, const lv_area_t *coords);
void lv_draw_ppa_letter(lv_draw_task_t *t, const lv_draw_letter_dsc_t *dsc, const lv_area_t *coords);
#endif

#if LV_USE_PPA_RUNTIME_TUNING || LV_USE_PPA_STATS
typedef enum {
    LV_DRAW_PPA_CLIENT_FILL = 0,
    LV_DRAW_PPA_CLIENT_BLEND,
    LV_DRAW_PPA_CLIENT_SRM,
    LV_DRAW_PPA_CLIENT_COUNT,
} lv_draw_ppa_client_kind_t;
#endif

#if LV_USE_PPA_RUNTIME_TUNING
typedef enum {
    LV_DRAW_PPA_BURST_8 = 0,
    LV_DRAW_PPA_BURST_16,
    LV_DRAW_PPA_BURST_32,
    LV_DRAW_PPA_BURST_64,
    LV_DRAW_PPA_BURST_128,
} lv_draw_ppa_burst_kind_t;

/**
 * Update the data burst length of one PPA client at runtime. Must be called
 * from the LVGL dispatch context (or with the LVGL lock held) because the
 * underlying client is unregistered and re-registered with the new
 * configuration. Pending sub-operations on the affected client are drained
 * before the change takes effect.
 *
 * @return ESP_OK on success.
 */
esp_err_t lv_draw_ppa_set_burst_length(lv_draw_ppa_client_kind_t client,
                                       lv_draw_ppa_burst_kind_t burst);

/**
 * Update the maximum number of pending transactions for one PPA client.
 * @param pending  1..512
 * @return ESP_OK on success.
 */
esp_err_t lv_draw_ppa_set_pending_trans(lv_draw_ppa_client_kind_t client,
                                        uint16_t pending);
#endif

#if LV_USE_PPA_STATS
typedef struct {
    uint32_t total_tasks;       /**< LVGL tasks completed by the PPA draw unit */
    uint32_t total_ops;         /**< PPA sub-operations submitted */
    uint32_t failed_ops;        /**< Sub-operations whose enqueue returned an error */
    uint32_t max_pending_seen;  /**< Peak in-flight sub-op count for a single task */
    uint64_t total_wait_us;     /**< Cumulative time spent in wait_for_finish_cb */
} lv_draw_ppa_stats_t;

void lv_draw_ppa_get_stats(lv_draw_ppa_stats_t *out);
void lv_draw_ppa_reset_stats(void);
#endif

#endif /* LV_USE_PPA */

#ifdef __cplusplus
}
#endif
