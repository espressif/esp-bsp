/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_lcd_panel_vendor.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Vendor specific configuration structure for panel device
 *
 * @note Put this structure into esp_lcd_panel_dev_config_t like this `.vendor_config = (void*)&vendor_config`
 *
 */
typedef struct {
    uint16_t lcd_width;     /*!< Width size of the LCD panel in pixels */
    uint16_t lcd_height;    /*!< Height size of the LCD panel in pixels */
} esp_lcd_panel_sh1107_config_t;

/**
 * @brief Create LCD panel for model SH1107
 *
 * @param[in] io LCD panel IO handle
 * @param[in] panel_dev_config general panel device configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *          - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_ERR_NO_MEM        if out of memory
 *          - ESP_OK                on success
 */
esp_err_t esp_lcd_new_panel_sh1107(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief I2C address of the SH1107 controller
 *
 */
#define ESP_LCD_IO_I2C_SH1107_ADDRESS  (0x3C)
#define ESP_LCD_IO_I2C_SH1107_ADDRESS1 (0x3D)

/**
 * @brief Touch IO configuration structure
 *
 */
#define ESP_LCD_IO_I2C_SH1107_CONFIG()             \
    {                                              \
        .dev_addr = ESP_LCD_IO_I2C_SH1107_ADDRESS, \
        .control_phase_bytes = 1,           \
        .dc_bit_offset = 0,                 \
        .lcd_cmd_bits = 8,                  \
        .lcd_param_bits = 8,                \
        .flags =                            \
        {                                   \
            .disable_control_phase = 1,     \
        }                                   \
    }

#ifdef __cplusplus
}
#endif
