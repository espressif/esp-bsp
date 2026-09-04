/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file
 * @brief ESP LCD: LT9611
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_vendor.h"
#include "soc/soc_caps.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LCD panel vendor configuration
 *
 * @note Pass this structure through `esp_lcd_panel_dev_config_t::vendor_config`.
 */
typedef struct {
    struct {
        esp_lcd_dsi_bus_handle_t dsi_bus;             /*!< MIPI-DSI bus handle */
        const esp_lcd_dpi_panel_config_t *dpi_config; /*!< MIPI-DPI panel configuration */
        uint8_t lane_num;                             /*!< Number of MIPI-DSI data lanes. 0 falls back to 2. */
    } mipi_config;
    struct {
        uint32_t hsync_active_low: 1; /*!< 1: HSYNC active low, 0: active high (default) */
        uint32_t vsync_active_low: 1; /*!< 1: VSYNC active low, 0: active high (default) */
    } flags;
} lt9611_vendor_config_t;

/**
 * @brief Create an LCD control panel for LT9611
 *
 * The driver adds an I2C device on `i2c_bus` internally. The I2C address is fixed.
 *
 * @param[in] i2c_bus I2C master bus handle
 * @param[in] panel_dev_config General panel device configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *      - ESP_OK: success
 *      - ESP_ERR_INVALID_ARG: invalid arguments
 *      - ESP_ERR_NO_MEM: out of memory
 *      - Otherwise: failure
 */
esp_err_t esp_lcd_new_panel_lt9611(i2c_master_bus_handle_t i2c_bus,
                                   const esp_lcd_panel_dev_config_t *panel_dev_config,
                                   esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief Return whether an HDMI sink is connected
 *
 * @param[in] panel LCD panel handle returned from `esp_lcd_new_panel_lt9611`
 * @return true when HDMI hot-plug detect is asserted
 */
bool esp_lcd_panel_lt9611_is_connected(esp_lcd_panel_t *panel);

/**
 * @brief MIPI-DSI lane bit rate in Mbps
 *
 * @param dpi_freq DPI source clock in MHz
 * @param bpp      Bits per pixel
 * @param lane_num Number of DSI data lanes
 */
#define LT9611_LANE_BITRATE_MBPS(dpi_freq, bpp, lane_num) ((dpi_freq) * (bpp) / (lane_num))

#if SOC_IS(ESP32P4)
/**
 * @brief MIPI-DSI lane bit rate for 1920x1080@60 RGB565 on ESP32-P4
 *
 * 160 MHz * 16 bpp / 2 lanes = 1280 Mbps/lane.
 */
#define LT9611_1920x1080_60HZ_RGB16_LANE_BITRATE_MBPS LT9611_LANE_BITRATE_MBPS(160, 16, 2)

/**
 * @brief MIPI-DSI lane bit rate for 1280x720@60 RGB888 on ESP32-P4
 *
 * 80 MHz * 24 bpp / 2 lanes = 960 Mbps/lane.
 */
#define LT9611_1280x720_60HZ_BGR24_LANE_BITRATE_MBPS LT9611_LANE_BITRATE_MBPS(80, 24, 2)
#endif

/**
 * @brief MIPI DPI configuration for 1920x1080@60 Hz in RGB565
 *
 * @note ESP32-P4 two-lane DSI cannot sustain 1080p60 RGB888.
 * @note CEA VIC 16 timing: refresh = 148500000 / 2200 / 1125 = 60 Hz.
 */
#define LT9611_1920x1080_60HZ_RGB16_DPI_PANEL_CONFIG_WITH_FBS(NUM_FBS) \
    {                                                                  \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_PLL_F160M,                 \
        .dpi_clock_freq_mhz = 148.5f,                                  \
        .virtual_channel = 0,                                          \
        .in_color_format = LCD_COLOR_FMT_RGB565,                       \
        .num_fbs = (NUM_FBS),                                          \
        .video_timing = {                                              \
            .h_size = 1920,                                            \
            .v_size = 1080,                                            \
            .hsync_back_porch = 148,                                   \
            .hsync_pulse_width = 44,                                   \
            .hsync_front_porch = 88,                                   \
            .vsync_back_porch = 36,                                    \
            .vsync_pulse_width = 5,                                    \
            .vsync_front_porch = 4,                                    \
        },                                                             \
        .flags = {                                                     \
            .disable_lp = true,                                        \
        },                                                             \
    }

/**
 * @brief MIPI DPI configuration for 1280x720@60 Hz in RGB888
 *
 * @note CEA VIC 4 timing: refresh = 74250000 / 1650 / 750 = 60 Hz.
 */
#define LT9611_1280x720_60HZ_BGR24_DPI_PANEL_CONFIG_WITH_FBS(NUM_FBS) \
    {                                                                 \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_PLL_F160M,                \
        .dpi_clock_freq_mhz = 74.25f,                                 \
        .virtual_channel = 0,                                         \
        .in_color_format = LCD_COLOR_FMT_RGB888,                      \
        .num_fbs = (NUM_FBS),                                         \
        .video_timing = {                                             \
            .h_size = 1280,                                           \
            .v_size = 720,                                            \
            .hsync_back_porch = 220,                                  \
            .hsync_pulse_width = 40,                                  \
            .hsync_front_porch = 110,                                 \
            .vsync_back_porch = 20,                                   \
            .vsync_pulse_width = 5,                                   \
            .vsync_front_porch = 5,                                   \
        },                                                            \
        .flags = {                                                    \
            .disable_lp = true,                                       \
        },                                                            \
    }

#ifdef __cplusplus
}
#endif
