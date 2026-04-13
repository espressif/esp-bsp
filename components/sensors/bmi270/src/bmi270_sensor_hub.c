/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_check.h"
#include "bmi270.h"
#include "iot_sensor_hub.h"

/*******************************************************************************
* Function definitions
*******************************************************************************/

esp_err_t bmi270_impl_init(bus_handle_t bus_handle, uint8_t addr);
esp_err_t bmi270_impl_deinit(void);
esp_err_t bmi270_impl_test (void);
esp_err_t bmi270_impl_acquire_acce (float *acce_x, float *acce_y, float *acce_z);
esp_err_t bmi270_impl_acquire_gyro(float *gyro_x, float *gyro_y, float *gyro_z);

/*******************************************************************************
* Local variables
*******************************************************************************/

static const char *TAG = "BMI270 sensor hub";
static bmi270_handle_t *sensor_hub_bmi270_handle;


/*******************************************************************************
* Public functions
*******************************************************************************/

esp_err_t bmi270_impl_init(bus_handle_t bus_handle, uint8_t addr)
{
    esp_err_t ret = ESP_OK;
    const bmi270_driver_config_t driver_config = {
        .addr = addr,
        .interface = BMI270_USE_I2C,
        .i2c_bus = bus_handle
    };
    ESP_RETURN_ON_ERROR(bmi270_create(&driver_config, &sensor_hub_bmi270_handle), TAG,
                        "Failed to initialize");

    // Set a reasonable default configuration
    const bmi270_config_t config = {
        .acce_odr = BMI270_ACC_ODR_100_HZ,
        .acce_range = BMI270_ACC_RANGE_4_G,
        .gyro_odr = BMI270_GYR_ODR_100_HZ,
        .gyro_range = BMI270_GYR_RANGE_1000_DPS
    };
    ret = bmi270_start(sensor_hub_bmi270_handle, &config);
    if (ret != ESP_OK) {
        ESP_RETURN_ON_ERROR(bmi270_delete(sensor_hub_bmi270_handle), TAG, "Failed to clean up a handle");
        return ret;
    }
    return ESP_OK;
}

esp_err_t bmi270_impl_deinit(void)
{
    bmi270_stop(sensor_hub_bmi270_handle);
    bmi270_delete(sensor_hub_bmi270_handle);
    return ESP_OK;
}

esp_err_t bmi270_impl_test (void)
{
    uint8_t chip_id;

    ESP_RETURN_ON_ERROR(bmi270_get_chip_id(sensor_hub_bmi270_handle, &chip_id), TAG,
                        "Failed to read chip id");
    if (chip_id == BMI270_CHIP_ID) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

esp_err_t bmi270_impl_acquire_acce (float *acce_x, float *acce_y, float *acce_z)
{
    ESP_RETURN_ON_ERROR(bmi270_get_acce_data(sensor_hub_bmi270_handle, acce_x, acce_y, acce_z), TAG,
                        "Failed to read accelerometer data");
    return ESP_OK;
}

esp_err_t bmi270_impl_acquire_gyro(float *gyro_x, float *gyro_y, float *gyro_z)
{
    ESP_RETURN_ON_ERROR(bmi270_get_gyro_data(sensor_hub_bmi270_handle, gyro_x, gyro_y, gyro_z), TAG,
                        "Failed to read gyroscope data");
    return ESP_OK;
}


static imu_impl_t bmi270_impl = {
    .init = bmi270_impl_init,
    .deinit = bmi270_impl_deinit,
    .test = bmi270_impl_test,
    .acquire_acce = bmi270_impl_acquire_acce,
    .acquire_gyro = bmi270_impl_acquire_gyro,
};

SENSOR_HUB_DETECT_FN(IMU_ID, sensor_hub_bmi270, &bmi270_impl);
