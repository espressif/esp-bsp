/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP LVGL port
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create a manual refresh task for the RGB interface
 *
 * @return
 *      - ESP_OK                    on success
 */
esp_err_t lvgl_rgb_create_manual_task(const lvgl_port_display_cfg_t *disp_cfg);

/**
 * @brief Registers the send completion event for the RGB interface
 *
 * @return
 *      - ESP_OK                    on success
 */
esp_err_t lvgl_rgb_register_event_callbacks(lvgl_port_display_ctx_t *disp_ctx, const lvgl_port_display_cfg_t *disp_cfg);

#ifdef __cplusplus
}
#endif
