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

esp_err_t lvgl_rgb_create_manual_task(const lvgl_port_display_cfg_t *disp_cfg);

esp_err_t lvgl_rgb_register_event_callbacks(lvgl_port_display_ctx_t *disp_ctx, const lvgl_port_display_cfg_t *disp_cfg);

#ifdef __cplusplus
}
#endif
