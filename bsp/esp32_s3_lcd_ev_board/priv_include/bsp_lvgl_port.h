/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "esp_lcd_types.h"
#include "esp_lcd_touch.h"
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize LCD panel and LVGL portation
 *
 * @note This function initialize RGB LCD panel and add add disp to LVGL.
 *
 * @return Pointer to LVGL display or NULL when error occured
 */
lv_disp_t *bsp_display_lcd_init();

#ifdef __cplusplus
}
#endif
