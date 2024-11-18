/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file
 * @brief ESP LCD: LT8912B
 */

#pragma once

#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LCD IO handles.
 */
typedef struct {
    esp_lcd_panel_io_handle_t main;
    esp_lcd_panel_io_handle_t cec_dsi;
    esp_lcd_panel_io_handle_t avi;
} esp_lcd_panel_lt8912b_io_t;

/**
 * @brief LCD panel video timing configuration.
 */
typedef struct {
    uint16_t hfp;
    uint16_t hs;
    uint16_t hbp;
    uint16_t hact;
    uint16_t htotal;
    uint16_t vfp;
    uint16_t vs;
    uint16_t vbp;
    uint16_t vact;
    uint16_t vtotal;
    bool     h_polarity;
    bool     v_polarity;
    uint16_t vic;
    uint8_t  aspect_ratio;  // 0=no data, 1=4:3, 2=16:9, 3=no data.
    uint32_t pclk_mhz;
} esp_lcd_panel_lt8912b_video_timing_t;

/**
 * @brief LCD panel vendor configuration.
 *
 * @note  This structure needs to be passed to the `esp_lcd_panel_dev_config_t::vendor_config`.
 *
 */
typedef struct {
    esp_lcd_panel_lt8912b_video_timing_t video_timing;       /*!< Video timing settings for HDMI */
    struct {
        esp_lcd_dsi_bus_handle_t dsi_bus;               /*!< MIPI-DSI bus configuration */
        const esp_lcd_dpi_panel_config_t *dpi_config;   /*!< MIPI-DPI panel configuration */
        uint8_t  lane_num;                              /*!< Number of MIPI-DSI lanes */
    } mipi_config;
} lt8912b_vendor_config_t;

/**
 * @brief Create LCD control panel for LT8912B
 *
 * @param[in] io LCD panel IO handles
 * @param[in] panel_dev_config general panel device configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *      - ESP_OK: Create LCD panel successfully
 *      - ESP_ERR_INVALID_ARG: Create LCD panel failed because of invalid arguments
 *      - ESP_ERR_NO_MEM: Create LCD panel failed because of memory allocation failure
 *      - ESP_FAIL: Create LCD panel failed because of other errors
 */
esp_err_t esp_lcd_new_panel_lt8912b(const esp_lcd_panel_lt8912b_io_t *io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief Return true if HDMI is ready
 *
 * @param[in] panel LCD panel handle
 * @return
 *      - true when ready
 */
bool esp_lcd_panel_lt8912b_is_ready(esp_lcd_panel_t *panel);

/**
 * @brief I2C address of the LT8912B controller
 *
 */
#define LT8912B_IO_I2C_MAIN_ADDRESS (0x48)
#define LT8912B_IO_I2C_CEC_ADDRESS  (0x49)
#define LT8912B_IO_I2C_AVI_ADDRESS  (0x4A)


#define LT8912B_ASPECT_RATION_NO 0x00
#define LT8912B_ASPECT_RATION_4_3 0x01
#define LT8912B_ASPECT_RATION_16_9 0x02

/**
 * @brief MIPI-DSI bus configuration structure
 *
 */
#define LT8912B_PANEL_BUS_DSI_2CH_CONFIG()               \
    {                                                    \
        .bus_id = 0,                                     \
        .num_data_lanes = 2,                             \
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,     \
        .lane_bit_rate_mbps = 1000,                      \
    }

/**
 * @brief MIPI-I2C panel IO configuration structure
 *
 */
#define LT8912B_IO_CFG(clk_speed_hz, address)            \
    {                                                    \
        .scl_speed_hz = clk_speed_hz,                    \
        .dev_addr = address,                             \
        .control_phase_bytes = 1,                        \
        .lcd_cmd_bits = 8,                               \
        .lcd_param_bits = 8,                             \
        .dc_bit_offset = 0,                              \
        .flags =                                         \
        {                                                \
            .disable_control_phase = 1,                  \
        }                                                \
    }

/**
 * @brief Video timing configuration structure (800x600 60Hz)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_800x600_60Hz() \
    {                               \
        .hfp            = 48,       \
        .hs             = 128,      \
        .hbp            = 88,       \
        .hact           = 800,      \
        .htotal         = 1056,     \
        .vfp            = 1,        \
        .vs             = 4,        \
        .vbp            = 23,       \
        .vact           = 600,      \
        .vtotal         = 628,      \
        .h_polarity     = 1,        \
        .v_polarity     = 1,        \
        .vic            = 0,        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_mhz       = 40,    \
    }

#define LT8912B_800x600_PANEL_60HZ_DPI_CONFIG() \
    {                           \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 40,                          \
        .virtual_channel = 0,                              \
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB888,     \
        .num_fbs = 1,                                      \
        .video_timing = {                                  \
            .h_size = BSP_LCD_H_RES,                       \
            .v_size = BSP_LCD_V_RES,                       \
            .hsync_back_porch = 88,                        \
            .hsync_pulse_width = 128,                      \
            .hsync_front_porch = 48,                       \
            .vsync_back_porch = 23,                        \
            .vsync_pulse_width = 4,                        \
            .vsync_front_porch = 1,                        \
        },                                                 \
        .flags.use_dma2d = true,                           \
        .flags.disable_lp = true,                          \
    }

/**
 * @brief Video timing configuration structure (1024x768 60Hz)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1024x768_60Hz() \
    {                               \
        .hfp            = 48,       \
        .hs             = 32,      \
        .hbp            = 80,      \
        .hact           = 1024,     \
        .htotal         = 1184,     \
        .vfp            = 3,        \
        .vs             = 4,        \
        .vbp            = 15,       \
        .vact           = 768,      \
        .vtotal         = 790,      \
        .h_polarity     = 1,        \
        .v_polarity     = 0,        \
        .vic            = 0,        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_mhz       = 56,    \
    }

#define LT8912B_1024x768_PANEL_60HZ_DPI_CONFIG() \
    {                           \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 56,                          \
        .virtual_channel = 0,                              \
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB888,     \
        .num_fbs = 1,                                      \
        .video_timing = {                                  \
            .h_size = BSP_LCD_H_RES,                       \
            .v_size = BSP_LCD_V_RES,                       \
            .hsync_back_porch = 80,                        \
            .hsync_pulse_width = 32,                      \
            .hsync_front_porch = 48,                       \
            .vsync_back_porch = 15,                        \
            .vsync_pulse_width = 4,                        \
            .vsync_front_porch = 3,                        \
        },                                                 \
        .flags.use_dma2d = true,                           \
        .flags.disable_lp = true,                          \
    }

/**
 * @brief Video timing configuration structure (1280x720 60Hz)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1280x720_60Hz() \
    {                               \
        .hfp            = 48,      \
        .hs             = 32,       \
        .hbp            = 80,      \
        .hact           = 1280,     \
        .htotal         = 1440,     \
        .vfp            = 3,        \
        .vs             = 5,        \
        .vbp            = 13,       \
        .vact           = 720,      \
        .vtotal         = 741,      \
        .h_polarity     = 1,        \
        .v_polarity     = 0,        \
        .vic            = 0,        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_mhz       = 64,    \
    }

#define LT8912B_1280x720_PANEL_60HZ_DPI_CONFIG() \
    {                           \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 64,                          \
        .virtual_channel = 0,                              \
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB888,     \
        .num_fbs = 1,                                      \
        .video_timing = {                                  \
            .h_size = BSP_LCD_H_RES,                       \
            .v_size = BSP_LCD_V_RES,                       \
            .hsync_back_porch = 80,                        \
            .hsync_pulse_width = 32,                      \
            .hsync_front_porch = 48,                       \
            .vsync_back_porch = 13,                        \
            .vsync_pulse_width = 5,                        \
            .vsync_front_porch = 3,                        \
        },                                                 \
        .flags.use_dma2d = true,                           \
        .flags.disable_lp = true,                          \
    }

/**
 * @brief Video timing configuration structure (1280x800 60Hz)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1280x800_60Hz() \
    {                               \
        .hfp            = 48,       \
        .hs             = 32,       \
        .hbp            = 80,      \
        .hact           = 1280,     \
        .htotal         = 1440,     \
        .vfp            = 3,        \
        .vs             = 6,        \
        .vbp            = 14,       \
        .vact           = 800,      \
        .vtotal         = 823,      \
        .h_polarity     = 1,        \
        .v_polarity     = 0,        \
        .vic            = 0,        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_mhz       = 70,    \
    }

#define LT8912B_1280x800_PANEL_60HZ_DPI_CONFIG() \
    {                           \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 70,                          \
        .virtual_channel = 0,                              \
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB888,     \
        .num_fbs = 1,                                      \
        .video_timing = {                                  \
            .h_size = BSP_LCD_H_RES,                       \
            .v_size = BSP_LCD_V_RES,                       \
            .hsync_back_porch = 80,                        \
            .hsync_pulse_width = 32,                      \
            .hsync_front_porch = 48,                       \
            .vsync_back_porch = 14,                        \
            .vsync_pulse_width = 6,                        \
            .vsync_front_porch = 3,                        \
        },                                                 \
        .flags.use_dma2d = true,                           \
        .flags.disable_lp = true,                          \
    }

/**
 * @brief Video timing configuration structure (1920x1080 30Hz)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1920x1080_30Hz() \
    {                               \
        .hfp            = 48,       \
        .hs             = 32,       \
        .hbp            = 80,      \
        .hact           = 1920,     \
        .htotal         = 2080,     \
        .vfp            = 3,        \
        .vs             = 5,        \
        .vbp            = 8,       \
        .vact           = 1080,     \
        .vtotal         = 1096,     \
        .h_polarity     = 1,        \
        .v_polarity     = 0,        \
        .vic            = 0,       \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_mhz       = 70,   \
    }

#define LT8912B_1920x1080_PANEL_30HZ_DPI_CONFIG() \
    {                           \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 70,                          \
        .virtual_channel = 0,                              \
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB888,     \
        .num_fbs = 1,                                      \
        .video_timing = {                                  \
            .h_size = BSP_LCD_H_RES,                       \
            .v_size = BSP_LCD_V_RES,                       \
            .hsync_back_porch = 80,                        \
            .hsync_pulse_width = 32,                      \
            .hsync_front_porch = 48,                       \
            .vsync_back_porch = 8,                        \
            .vsync_pulse_width = 5,                        \
            .vsync_front_porch = 3,                        \
        },                                                 \
        .flags.use_dma2d = true,                           \
        .flags.disable_lp = true,                          \
    }

/**
 * @brief Video timing configuration structure (1920x1080 60Hz)
 *
 */

/* This setting is not working yet, it is only for developing and testing */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1920x1080_60Hz() \
    {                               \
        .hfp            = 48,       \
        .hs             = 32,       \
        .hbp            = 80,      \
        .hact           = 1920,     \
        .htotal         = 2080,     \
        .vfp            = 3,        \
        .vs             = 5,        \
        .vbp            = 19,       \
        .vact           = 1080,     \
        .vtotal         = 1107,     \
        .h_polarity     = 1,        \
        .v_polarity     = 0,        \
        .vic            = 0,       \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_mhz       = 120,   \
    }

/* This setting is not working yet, it is only for developing and testing */
#define LT8912B_1920x1080_PANEL_60HZ_DPI_CONFIG() \
    {                           \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 120,                          \
        .virtual_channel = 0,                              \
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB888,     \
        .num_fbs = 1,                                      \
        .video_timing = {                                  \
            .h_size = BSP_LCD_H_RES,                       \
            .v_size = BSP_LCD_V_RES,                       \
            .hsync_back_porch = 80,                        \
            .hsync_pulse_width = 32,                      \
            .hsync_front_porch = 48,                       \
            .vsync_back_porch = 19,                        \
            .vsync_pulse_width = 5,                        \
            .vsync_front_porch = 3,                        \
        },                                                 \
        .flags.use_dma2d = true,                           \
        .flags.disable_lp = true,                          \
    }


#ifdef __cplusplus
}
#endif
