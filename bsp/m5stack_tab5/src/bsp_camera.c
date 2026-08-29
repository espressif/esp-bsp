/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_err_check.h"
#include "esp_video_device.h"
#include "esp_video_init.h"

#include "bsp/m5stack_tab5.h"

#define CAMERA_LEDC_CH   CONFIG_BSP_CAMERA_XCLK_LEDC_CH

esp_err_t bsp_camera_start(const bsp_camera_cfg_t *cfg)
{

    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    /* Enable Feature */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_CAMERA, true));
    vTaskDelay(pdMS_TO_TICKS(100));

    const esp_video_init_csi_config_t base_csi_config = {
        .sccb_config = {
            .init_sccb = false,
            .i2c_handle = bsp_i2c_get_handle(),
            .freq = 400000,
        },
        .reset_pin = BSP_CAMERA_RST,
        .pwdn_pin  = -1,
    };

    esp_video_init_config_t cam_config = {
        .csi      = &base_csi_config,
    };

    return esp_video_init(&cam_config);
}
