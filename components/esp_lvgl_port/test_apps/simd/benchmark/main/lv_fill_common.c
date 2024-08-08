/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "lv_fill_common.h"
#include "lvgl.h"
#include "esp_lvgl_port_lv_blend.h"

#define LV_FILL_CHECK(cond, ret_val) ({                                     \
            if (!(cond)) {                                                  \
                return (ret_val);                                           \
            }                                                               \
})

static blend_params_t *s_blend_params = NULL;
static test_area_t *s_area = NULL;

static lv_color_t test_color = {
    .blue = 0x56,
    .green = 0x34,
    .red = 0x12,
};

esp_err_t get_blend_params(blend_params_t **blend_params_ret, test_area_t **area_ret)
{
    LV_FILL_CHECK((s_area != NULL || s_blend_params != NULL), ESP_ERR_INVALID_STATE);

    *blend_params_ret = s_blend_params;
    *area_ret = s_area;

    return ESP_OK;
}

esp_err_t set_color_format(blend_params_t *blend_params, lv_color_format_t color_format)
{
    LV_FILL_CHECK(blend_params != NULL, ESP_ERR_INVALID_STATE);

    blend_params->draw_unit.target_layer->color_format = color_format;

    return ESP_OK;
}

esp_err_t init_blend_params(void)
{
    LV_FILL_CHECK((s_area == NULL || s_blend_params == NULL), ESP_ERR_INVALID_STATE);

    // Allocate space for test_area_t
    test_area_t *p_test_area = (test_area_t *)malloc(sizeof(test_area_t));

    // Allocate space for lv_draw_buf_t
    lv_draw_buf_t *p_draw_buf = (lv_draw_buf_t *)malloc(sizeof(lv_draw_buf_t));

    // Allocate space for lv_layer_t
    lv_layer_t *p_target_layer = (lv_layer_t *)malloc(sizeof(lv_layer_t));

    // Allocate space for blend_params_t
    blend_params_t *p_blend_params = (blend_params_t *)malloc(sizeof(blend_params_t));

    if (p_test_area == NULL ||
            p_draw_buf == NULL ||
            p_target_layer == NULL ||
            p_blend_params == NULL) {
        return ESP_ERR_NO_MEM;
    }

    // Initialize draw buf data with NULL
    lv_draw_buf_t draw_buf = {
        .data = NULL,
    };

    // Fill allocated space with initialized struct
    *p_draw_buf = draw_buf;

    // Initialize targe layer
    lv_layer_t target_layer = {
        //.color_format     will be set with setter function
        //.buf_area         not being pointer, can't be updated now and will have to be updated for each test case
        .draw_buf = p_draw_buf,
    };

    // Fill allocated space with initialized struct
    *p_target_layer = target_layer;

    // Initialize blend params struct
    blend_params_t blend_params = {
        .blend_dsc = {
            .blend_area = &p_test_area->blend,
            .src_buf = NULL,
            .opa = LV_OPA_MAX,
            .color = test_color,
            .mask_buf = NULL,
            .mask_res = LV_DRAW_SW_MASK_RES_FULL_COVER,
            .mask_area = NULL,
        },
        .draw_unit = {
            .target_layer = p_target_layer,
            .clip_area = &p_test_area->clip,
        },
    };

    *p_blend_params = blend_params;
    s_blend_params = p_blend_params;
    s_area = p_test_area;

    return ESP_OK;
}

esp_err_t free_blend_params(void)
{
    // Free area
    free(s_area);

    // Free blend params
    if (s_blend_params != NULL) {
        // Check target layer
        if (s_blend_params->draw_unit.target_layer != NULL) {
            free(s_blend_params->draw_unit.target_layer->draw_buf);
            free(s_blend_params->draw_unit.target_layer);
        }
        free(s_blend_params);
    }

    s_area = NULL;
    s_blend_params = NULL;

    return ESP_OK;
}
