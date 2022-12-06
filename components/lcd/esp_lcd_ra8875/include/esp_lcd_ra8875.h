/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file
 * @brief ESP LCD: RA8875
 */

#pragma once

#include "esp_lcd_panel_vendor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Vendor specific configuration structure for panel device
 */
typedef struct {
    int wait_gpio_num;      /*!< GPIO used to indicate busy state of the LCD panel, set to -1 if it's not used */
    uint16_t lcd_width;     /*!< Width size of the LCD panel in pixels */
    uint16_t lcd_height;    /*!< Height size of the LCD panel in pixels */
    int mcu_bit_interface;  /*!< Selection between 8-bit and 16-bit MCU interface */
} esp_lcd_panel_ra8875_config_t;

/**
 * @brief Create LCD panel for model RA8875
 *
 * @param[in] io LCD panel IO handle
 * @param[in] panel_dev_config general panel device configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *          - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_ERR_NO_MEM        if out of memory
 *          - ESP_OK                on success
 */
esp_err_t esp_lcd_new_panel_ra8875(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

#ifdef __cplusplus
}
#endif
