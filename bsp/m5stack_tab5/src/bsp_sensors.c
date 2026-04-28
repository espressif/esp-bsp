/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_err_check.h"
#include "bsp/m5stack_tab5.h"
#include "bmi270.h"

static const char *TAG = "M5Stack Tab5";

esp_err_t bsp_sensor_init(const bsp_sensor_config_t *cfg, sensor_handle_t *sensor_handle)
{
    ESP_RETURN_ON_FALSE(cfg != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the sensor config can't be NULL");
    ESP_RETURN_ON_FALSE(sensor_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the sensor handle can't be NULL");

    esp_err_t err = ESP_OK;

    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    sensor_config_t config = {
        .type = cfg->type,
        .mode = cfg->mode,
        .min_delay = cfg->period
    };

    switch (cfg->type) {
    case IMU_ID:
        config.bus = bsp_i2c_get_handle();
        config.addr = BMI270_I2C_ADDRESS_L;
        err = iot_sensor_create("sensor_hub_bmi270", &config, sensor_handle);
        break;
    default:
        return ESP_FAIL;
    }

    return err;
}
