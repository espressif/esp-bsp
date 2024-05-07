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
#include "esp_lcd_panel_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    esp_lcd_panel_io_handle_t main;
    esp_lcd_panel_io_handle_t cec_dsi;
    esp_lcd_panel_io_handle_t avi;
} esp_lcd_panel_lt8912b_io_t;

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
    uint32_t pclk_khz;
} esp_lcd_panel_lt8912b_video_timing_t;

typedef struct {
    esp_lcd_panel_lt8912b_video_timing_t video_timing;
} esp_lcd_panel_lt8912b_cfg_t;

/**
 * @brief Create LCD control panel for LT8912B
 *
 * @param[in] io LCD panel IO handles
 * @param[in] cfg specific configuration of panel
 * @param[in] panel_dev_config general panel device configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *      - ESP_OK: Create LCD panel successfully
 *      - ESP_ERR_INVALID_ARG: Create LCD panel failed because of invalid arguments
 *      - ESP_ERR_NO_MEM: Create LCD panel failed because of memory allocation failure
 *      - ESP_FAIL: Create LCD panel failed because of other errors
 */
esp_err_t esp_lcd_new_panel_lt8912b(const esp_lcd_panel_lt8912b_io_t *io, const esp_lcd_panel_lt8912b_cfg_t *cfg, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel);

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
#define ESP_LCD_IO_I2C_LT8912B_MAIN_ADDRESS (0x48)
#define ESP_LCD_IO_I2C_LT8912B_CEC_ADDRESS  (0x49)
#define ESP_LCD_IO_I2C_LT8912B_AVI_ADDRESS  (0x4A)


#define LT8912B_ASPECT_RATION_NO 0x00
#define LT8912B_ASPECT_RATION_4_3 0x01
#define LT8912B_ASPECT_RATION_16_9 0x02

/**
 * @brief Video timing configuration structure (640x480 60Hz)
 *
 * @note refresh_rate = (pclk_khz * 1000) / (hact + hs + hbp + hfp) / (vact + vs + vbp + vfp)
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_640x480_60HZ() \
    {                               \
        .hfp            = 16,       \
        .hs             = 96,       \
        .hbp            = 48,       \
        .hact           = 640,      \
        .htotal         = 800,      \
        .vfp            = 10,       \
        .vs             = 2,        \
        .vbp            = 33,       \
        .vact           = 480,      \
        .vtotal         = 525,      \
        .h_polarity     = 0,        \
        .v_polarity     = 0,        \
        .vic            = 1,        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_4_3, \
        .pclk_khz       = 25000,    \
    }

/**
 * @brief Video timing configuration structure (720x480 60Hz)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_720x480_60Hz() \
    {                               \
        .hfp            = 16,       \
        .hs             = 62,       \
        .hbp            = 60,       \
        .hact           = 720,      \
        .htotal         = 858,      \
        .vfp            = 9,        \
        .vs             = 6,        \
        .vbp            = 30,       \
        .vact           = 480,      \
        .vtotal         = 525,      \
        .h_polarity     = 0,        \
        .v_polarity     = 0,        \
        .vic            = 2,        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_4_3, \
        .pclk_khz       = 27000,    \
    }

/**
 * @brief Video timing configuration structure (720x576 50Hz)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_720x576_50Hz() \
    {                               \
        .hfp            = 12,       \
        .hs             = 64,       \
        .hbp            = 68,       \
        .hact           = 720,      \
        .htotal         = 864,      \
        .vfp            = 5,        \
        .vs             = 5,        \
        .vbp            = 39,       \
        .vact           = 576,      \
        .vtotal         = 625,      \
        .h_polarity     = 0,        \
        .v_polarity     = 0,        \
        .vic            = 17,       \
        .aspect_ratio   = LT8912B_ASPECT_RATION_4_3, \
        .pclk_khz       = 27000,    \
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
        .pclk_khz       = 40000,    \
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
        .vic            = 0,        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_khz       = 40000,    \
    }

/**
 * @brief Video timing configuration structure (1280x720 60Hz)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1280x720_60Hz() \
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
        .vic            = 4,        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_khz       = 74250,    \
    }

/**
 * @brief Video timing configuration structure (1280x720 30Hz)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1280x720_30Hz() \
    {                               \
        .hfp            = 48,       \
        .hs             = 128,      \
        .hbp            = 88,      \
        .hact           = 1280,     \
        .htotal         = 1650,     \
        .vfp            = 1,        \
        .vs             = 4,        \
        .vbp            = 23,       \
        .vact           = 720,      \
        .vtotal         = 750,      \
        .h_polarity     = 1,        \
        .v_polarity     = 1,        \
        .vic            = 0,        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_khz       = 40000,    \
    }

/**
 * @brief Video timing configuration structure (1280x800 60Hz)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1280x800_60Hz() \
    {                               \
        .hfp            = 28,       \
        .hs             = 32,       \
        .hbp            = 100,      \
        .hact           = 1280,     \
        .htotal         = 1440,     \
        .vfp            = 2,        \
        .vs             = 6,        \
        .vbp            = 15,       \
        .vact           = 800,      \
        .vtotal         = 823,      \
        .h_polarity     = 0,        \
        .v_polarity     = 0,        \
        .vic            = 0,        \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_khz       = 71000,    \
    }

/**
 * @brief Video timing configuration structure (1920x1080 60Hz)
 *
 */
#define ESP_LCD_LT8912B_VIDEO_TIMING_1920x1080_60Hz() \
    {                               \
        .hfp            = 88,       \
        .hs             = 44,       \
        .hbp            = 148,      \
        .hact           = 1920,     \
        .htotal         = 2200,     \
        .vfp            = 4,        \
        .vs             = 5,        \
        .vbp            = 36,       \
        .vact           = 1080,     \
        .vtotal         = 1125,     \
        .h_polarity     = 1,        \
        .v_polarity     = 1,        \
        .vic            = 16,       \
        .aspect_ratio   = LT8912B_ASPECT_RATION_16_9, \
        .pclk_khz       = 148500,   \
    }


#ifdef __cplusplus
}
#endif
