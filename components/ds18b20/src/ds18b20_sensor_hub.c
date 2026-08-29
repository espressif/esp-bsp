/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "ds18b20.h"
#include "iot_sensor_hub.h"

static const char *TAG = "ds18b20_sensor_hub";
static ds18b20_device_handle_t sensor_hub_ds18b20_handle;

/*******************************************************************************
* Private functions for sensor hub implementation
*******************************************************************************/

esp_err_t ds18b20_impl_init(bus_handle_t bus_handle, uint8_t addr)
{
    ESP_RETURN_ON_FALSE(bus_handle != NULL, ESP_ERR_INVALID_ARG, TAG, "Bus handle can't be NULL");
    ESP_UNUSED(addr);

    ds18b20_config_t config;

    return ds18b20_new_device_from_bus(bus_handle, &config, &sensor_hub_ds18b20_handle);
}

esp_err_t ds18b20_impl_deinit(void)
{
    esp_err_t ret = ESP_OK;

    ret = ds18b20_del_device(sensor_hub_ds18b20_handle);
    sensor_hub_ds18b20_handle = NULL;
    return ret;
}

esp_err_t ds18b20_impl_test(void)
{
    return ds18b20_trigger_temperature_conversion(sensor_hub_ds18b20_handle);
}

esp_err_t ds18b20_impl_acquire_temperature(float *temperature)
{
    ESP_RETURN_ON_FALSE(temperature != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the temperature value can't be NULL");
    ESP_RETURN_ON_ERROR(ds18b20_trigger_temperature_conversion(sensor_hub_ds18b20_handle), TAG,
                        "Failed to start temperature measurement.");
    return ds18b20_get_temperature(sensor_hub_ds18b20_handle, temperature);
}

esp_err_t ds18b20_impl_acquire_humidity(float *humidity)
{
    ESP_UNUSED(humidity);
    return ESP_ERR_NOT_SUPPORTED;
}

static humiture_impl_t ds18b20_impl = {
    .init = ds18b20_impl_init,
    .deinit = ds18b20_impl_deinit,
    .test = ds18b20_impl_test,
    .acquire_temperature = ds18b20_impl_acquire_temperature,
    .acquire_humidity = ds18b20_impl_acquire_humidity,
};

SENSOR_HUB_DETECT_FN(HUMITURE_ID, sensor_hub_ds18b20, &ds18b20_impl);
