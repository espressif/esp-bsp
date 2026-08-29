/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_err_check.h"
#include "esp_video_device.h"
#include "esp_video_init.h"

#include "bsp/m5stack_core_s3.h"

#define CAMERA_LEDC_CH   CONFIG_BSP_CAMERA_XCLK_LEDC_CH

esp_err_t bsp_camera_start(const bsp_camera_cfg_t *cfg)
{

    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    /* Enable Feature */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_CAMERA, true));
    vTaskDelay(pdMS_TO_TICKS(100));

    const esp_video_init_dvp_config_t base_dvp_config = {
        .sccb_config = {
            .init_sccb = false,
            .i2c_handle = bsp_i2c_get_handle(),
            .freq = 100000,
        },
        .reset_pin = BSP_CAMERA_RST,
        .pwdn_pin  = -1,
        .dvp_pin = {
            .data_width = 8,
            .data_io = {
                BSP_CAMERA_D0,
                BSP_CAMERA_D1,
                BSP_CAMERA_D2,
                BSP_CAMERA_D3,
                BSP_CAMERA_D4,
                BSP_CAMERA_D5,
                BSP_CAMERA_D6,
                BSP_CAMERA_D7,
            },
            .vsync_io = BSP_CAMERA_VSYNC,
            .de_io = BSP_CAMERA_HSYNC,
            .pclk_io = BSP_CAMERA_PCLK,
            .xclk_io = BSP_CAMERA_GPIO_XCLK, // Set XCLK pin to generate XCLK signal
        },
        .xclk_freq = BSP_CAMERA_XCLK_CLOCK_MHZ * 1000000,
    };

    esp_video_init_config_t cam_config = {
        .dvp      = &base_dvp_config,
    };

    return esp_video_init(&cam_config);
}
