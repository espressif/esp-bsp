/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
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
    float pclk_mhz;
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
esp_err_t esp_lcd_new_panel_lt8912b(const esp_lcd_panel_lt8912b_io_t *io,
                                    const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

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
        .phy_clk_src = 0,                                \
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
        .hfp            = 40,       \
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
        .vic            = 0, /* Custom configuration */        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_mhz       = 40.0,    \
    }

#define LT8912B_800x600_PANEL_60HZ_DPI_CONFIG() LT8912B_800x600_PANEL_60HZ_DPI_CONFIG_WITH_FBS(1)
#define LT8912B_800x600_PANEL_60HZ_DPI_CONFIG_WITH_FBS(NUM_FBS) \
    {                           \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 40.0,                          \
        .virtual_channel = 0,                              \
        .in_color_format = LCD_COLOR_FMT_RGB888,     \
        .num_fbs = NUM_FBS,                                      \
        .video_timing = {                                  \
            .h_size = 800,                       \
            .v_size = 600,                       \
            .hsync_back_porch = 88,                        \
            .hsync_pulse_width = 128,                      \
            .hsync_front_porch = 40,                       \
            .vsync_back_porch = 23,                        \
            .vsync_pulse_width = 4,                        \
            .vsync_front_porch = 1,                        \
        },                                                 \
        .flags.disable_lp = true,                          \
    }

/**
 * @brief Video timing configuration structure (1024x768 60Hz)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1024x768_60Hz() \
    {                               \
        .hfp            = 24,       \
        .hs             = 136,      \
        .hbp            = 160,      \
        .hact           = 1024,     \
        .htotal         = 1344,     \
        .vfp            = 3,        \
        .vs             = 6,        \
        .vbp            = 29,       \
        .vact           = 768,      \
        .vtotal         = 806,      \
        .h_polarity     = 0,        \
        .v_polarity     = 0,        \
        .vic            = 0, /* Custom configuration */        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_4_3, \
        .pclk_mhz       = 65,    \
    }

#define LT8912B_1024x768_PANEL_60HZ_DPI_CONFIG() LT8912B_1024x768_PANEL_60HZ_DPI_CONFIG_WITH_FBS(1)
#define LT8912B_1024x768_PANEL_60HZ_DPI_CONFIG_WITH_FBS(NUM_FBS) \
    {                           \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 65,                          \
        .virtual_channel = 0,                              \
        .in_color_format = LCD_COLOR_FMT_RGB888,     \
        .num_fbs = NUM_FBS,                                      \
        .video_timing = {                                  \
            .h_size = 1024,                       \
            .v_size = 768,                       \
            .hsync_back_porch = 160,                        \
            .hsync_pulse_width = 136,                      \
            .hsync_front_porch = 24,                       \
            .vsync_back_porch = 29,                        \
            .vsync_pulse_width = 6,                        \
            .vsync_front_porch = 3,                        \
        },                                                 \
        .flags.disable_lp = true,                          \
    }

/**
 * @brief Video timing configuration structure (1280x720 50Hz CEA standard)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1280x720_50Hz() \
    {                               \
        .hfp            = 110,      \
        .hs             = 40,       \
        .hbp            = 220,      \
        .hact           = 1280,     \
        .htotal         = 1650,     \
        .vfp            = 5,        \
        .vs             = 5,        \
        .vbp            = 20,       \
        .vact           = 720,      \
        .vtotal         = 750,      \
        .h_polarity     = 1,        \
        .v_polarity     = 1,        \
        .vic            = 19, /* CEA 720p50 */        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_mhz       = 74.25,    \
    }

#define LT8912B_1280x720_PANEL_50HZ_DPI_CONFIG() LT8912B_1280x720_PANEL_50HZ_DPI_CONFIG_WITH_FBS(1)
#define LT8912B_1280x720_PANEL_50HZ_DPI_CONFIG_WITH_FBS(NUM_FBS) \
    {                           \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 74.25,                       \
        .virtual_channel = 0,                              \
        .in_color_format = LCD_COLOR_FMT_RGB888,           \
        .num_fbs = NUM_FBS,                                \
        .video_timing = {                                  \
            .h_size = 1280,                                \
            .v_size = 720,                                 \
            .hsync_back_porch = 220,                       \
            .hsync_pulse_width = 40,                       \
            .hsync_front_porch = 110,                      \
            .vsync_back_porch = 20,                        \
            .vsync_pulse_width = 5,                        \
            .vsync_front_porch = 5,                        \
        },                                                 \
        .flags.disable_lp = true,                          \
    }

/**
 * @brief Video timing configuration structure (1280x720 60Hz CEA standard)
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1280x720_60Hz() \
    {                               \
        .hfp            = 110,      /* front porch */ \
        .hs             = 40,       /* sync pulse */ \
        .hbp            = 220,      /* back porch */ \
        .hact           = 1280,     /* active pixels */ \
        .htotal         = 1650,     /* total pixels */ \
        .vfp            = 5,        /* vertical front porch */ \
        .vs             = 5,        /* vertical sync pulse */ \
        .vbp            = 20,       /* vertical back porch */ \
        .vact           = 720,      /* vertical active lines */ \
        .vtotal         = 750,      /* total lines */ \
        .h_polarity     = 1,        /* HS polarity + */ \
        .v_polarity     = 1,        /* VS polarity + */ \
        .vic            = 4,        /* CEA 720p60 */ \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_mhz       = 74.25f,   /* exact CEA pixel clock */ \
    }

#define LT8912B_1280x720_PANEL_60HZ_DPI_CONFIG() LT8912B_1280x720_PANEL_60HZ_DPI_CONFIG_WITH_FBS(1)
#define LT8912B_1280x720_PANEL_60HZ_DPI_CONFIG_WITH_FBS(NUM_FBS) \
    {                           \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT, \
        .dpi_clock_freq_mhz = 74.25f,\
        .virtual_channel = 0, \
        .in_color_format = LCD_COLOR_FMT_RGB888, \
        .num_fbs = NUM_FBS, \
        .video_timing = { \
            .h_size = 1280, \
            .v_size = 720, \
            .hsync_back_porch = 220, \
            .hsync_pulse_width = 40, \
            .hsync_front_porch = 110, \
            .vsync_back_porch = 20, \
            .vsync_pulse_width = 5, \
            .vsync_front_porch = 5, \
        }, \
        .flags.disable_lp = true, \
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
        .vic            = 0, /* Custom configuration */    \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_mhz       = 71.11,    \
    }

#define LT8912B_1280x800_PANEL_60HZ_DPI_CONFIG() LT8912B_1280x800_PANEL_60HZ_DPI_CONFIG_WITH_FBS(1)
#define LT8912B_1280x800_PANEL_60HZ_DPI_CONFIG_WITH_FBS(NUM_FBS) \
    {                           \
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,       \
        .dpi_clock_freq_mhz = 71.11,                          \
        .virtual_channel = 0,                              \
        .in_color_format = LCD_COLOR_FMT_RGB888,     \
        .num_fbs = NUM_FBS,                                      \
        .video_timing = {                                  \
            .h_size = 1280,                       \
            .v_size = 800,                       \
            .hsync_back_porch = 80,                        \
            .hsync_pulse_width = 32,                      \
            .hsync_front_porch = 48,                       \
            .vsync_back_porch = 14,                        \
            .vsync_pulse_width = 6,                        \
            .vsync_front_porch = 3,                        \
        },                                                 \
        .flags.disable_lp = true,                          \
    }

/**
 * @brief Video timing configuration structure (1600x900 60Hz CVT-RB)
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1600x900_60Hz() \
{ \
    .hfp            = 48, \
    .hs             = 32, \
    .hbp            = 80, \
    .hact           = 1600, \
    .htotal         = 1760, \
    .vfp            = 3, \
    .vs             = 5, \
    .vbp            = 23, \
    .vact           = 900, \
    .vtotal         = 931, \
    .h_polarity     = 1, \
    .v_polarity     = 0, \
    .vic            = 0, \
    .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
    .pclk_mhz       = 97.75f, \
}

#define LT8912B_1600x900_PANEL_60HZ_DPI_CONFIG() \
    LT8912B_1600x900_PANEL_60HZ_DPI_CONFIG_WITH_FBS(1)

#define LT8912B_1600x900_PANEL_60HZ_DPI_CONFIG_WITH_FBS(NUM_FBS) \
{ \
    .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT, \
    .dpi_clock_freq_mhz = 97.75f, \
    .virtual_channel = 0, \
    .in_color_format = LCD_COLOR_FMT_RGB888, \
    .num_fbs = NUM_FBS, \
    .video_timing = { \
        .h_size = 1600, \
        .v_size = 900, \
        .hsync_back_porch = 80, \
        .hsync_pulse_width = 32, \
        .hsync_front_porch = 48, \
        .vsync_back_porch = 23, \
        .vsync_pulse_width = 5, \
        .vsync_front_porch = 3, \
    }, \
    .flags.disable_lp = true, \
}

/**
 * @brief Video timing configuration structure (1920x1080 30Hz CEA-861 VIC 34)
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1920x1080_30Hz() \
{ \
    .hfp            = 88, \
    .hs             = 44, \
    .hbp            = 148, \
    .hact           = 1920, \
    .htotal         = 2200, \
    .vfp            = 4, \
    .vs             = 5, \
    .vbp            = 36, \
    .vact           = 1080, \
    .vtotal         = 1125, \
    .h_polarity     = 1, \
    .v_polarity     = 1, \
    .vic            = 0, /* Custom configuration */ \
    .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
    .pclk_mhz       = 74.25f, \
}

#define LT8912B_1920x1080_PANEL_30HZ_DPI_CONFIG() \
    LT8912B_1920x1080_PANEL_30HZ_DPI_CONFIG_WITH_FBS(1)

#define LT8912B_1920x1080_PANEL_30HZ_DPI_CONFIG_WITH_FBS(NUM_FBS) \
{ \
    .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT, \
    .dpi_clock_freq_mhz = 74.25f, \
    .virtual_channel = 0, \
    .in_color_format = LCD_COLOR_FMT_RGB888, \
    .num_fbs = NUM_FBS, \
    .video_timing = { \
        .h_size = 1920, \
        .v_size = 1080, \
        .hsync_back_porch = 148, \
        .hsync_pulse_width = 44, \
        .hsync_front_porch = 88, \
        .vsync_back_porch = 36, \
        .vsync_pulse_width = 5, \
        .vsync_front_porch = 4, \
    }, \
    .flags.disable_lp = true, \
}


#ifdef __cplusplus
}
#endif
