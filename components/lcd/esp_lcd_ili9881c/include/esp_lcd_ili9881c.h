/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file
 * @brief ESP LCD: ILI9881C
 */

#pragma once

#include "esp_lcd_panel_vendor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create LCD control panel for ILI9881C
 *
 * @param[in] io LCD panel IO handle
 * @param[in] panel_dev_config general panel device configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *      - ESP_OK: Create LCD panel successfully
 *      - ESP_ERR_INVALID_ARG: Create LCD panel failed because of invalid arguments
 *      - ESP_ERR_NO_MEM: Create LCD panel failed because of memory allocation failure
 *      - ESP_FAIL: Create LCD panel failed because of other errors
 */
esp_err_t esp_lcd_new_panel_ili9881c(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

#ifdef __cplusplus
}
#endif
