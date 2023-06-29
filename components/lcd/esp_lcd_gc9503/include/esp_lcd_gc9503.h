/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file
 * @brief ESP LCD: GC9503
 */

#pragma once

#include "esp_lcd_types.h"
#include "esp_lcd_panel_rgb.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create LCD panel for GC9503.
 *
 * @note  This function first initialize GC9503 with vendor specific initialization, then calls `esp_lcd_new_rgb_panel()` to create a RGB LCD panel.
 * @note  Vendor specific initialization can be different between manufacturers, should consult the LCD supplier for initialization sequence code.
 *
 * @param[in]  io_handle  LCD panel IO handle
 * @param[in]  rgb_config Pointer to RGB panel timing configuration structure
 * @param[out] ret_panel  Returned LCD panel handle
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_lcd_new_panel_gc9503(esp_lcd_panel_io_handle_t io_handle, const esp_lcd_rgb_panel_config_t *rgb_config, esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief 3-wire SPI panel IO configuration structure
 *
 */
#define GC9503_PANEL_IO_3WIRE_SPI_CONFIG(line_cfg)          \
    {                                                       \
        .line_config = line_cfg,                            \
        .expect_clk_speed = PANEL_IO_3WIRE_SPI_CLK_MAX,     \
        .spi_mode = 0,                                      \
        .lcd_cmd_bytes = 1,                                 \
        .lcd_param_bytes = 1,                               \
        .flags = {                                          \
            .use_dc_bit = true,                             \
            .dc_zero_on_data = false,                       \
            .lsb_first = false,                             \
            .cs_high_active = false,                        \
            .del_keep_cs_inactive = true,                   \
        },                                                  \
    }

/**
 * @brief RGB timing structure
 *
 * @note  frame_rate = pclk_hz / (h_res + hsync_pulse_width + hsync_back_porch + hsync_front_porch)
 *                             / (v_res + vsync_pulse_width + vsync_back_porch + vsync_front_porch)
 *
 */
#define GC9503_480_480_PANEL_60HZ_RGB_TIMING()      \
    {                                               \
        .pclk_hz = 16 * 1000 * 1000,                \
        .h_res = 480,                               \
        .v_res = 480,                               \
        .hsync_pulse_width = 10,                    \
        .hsync_back_porch = 10,                     \
        .hsync_front_porch = 20,                    \
        .vsync_pulse_width = 10,                    \
        .vsync_back_porch = 10,                     \
        .vsync_front_porch = 10,                    \
        .flags.pclk_active_neg = false,             \
    }

#ifdef __cplusplus
}
#endif
