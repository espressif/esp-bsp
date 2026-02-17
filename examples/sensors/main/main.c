/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief BSP Sensors Example
 * @details Acquire sensor data using the sensor hub component
 * @example https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=sensors
 */

#include <stdio.h>
#include "bsp/esp-bsp.h"
#include "esp_log.h"

#define IMU_SAMPLING_PERIOD             300
#define HUMITURE_SAMPLING_PERIOD        500

static const char *TAG = "example";

static sensor_handle_t imu_sensor_handle = NULL;
static sensor_handle_t humiture_sensor_handle = NULL;

void sensor_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    sensor_data_t *sensor_data = (sensor_data_t *)event_data;

    switch (id) {
    case SENSOR_STARTED:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x STARTED",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr);
        break;
    case SENSOR_STOPED:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x STOPPED",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr);
        break;
    case SENSOR_HUMI_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x HUMI_DATA_READY - "
                 "humidity=%.2f",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->humidity);
        break;
    case SENSOR_TEMP_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x TEMP_DATA_READY - "
                 "temperature=%.2f",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->temperature);
        break;
    case SENSOR_ACCE_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x ACCE_DATA_READY - "
                 "acce_x=%.2f, acce_y=%.2f, acce_z=%.2f",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->acce.x, sensor_data->acce.y, sensor_data->acce.z);
        break;
    case SENSOR_GYRO_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x GYRO_DATA_READY - "
                 "gyro_x=%.2f, gyro_y=%.2f, gyro_z=%.2f",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->gyro.x, sensor_data->gyro.y, sensor_data->gyro.z);
        break;
    default:
        ESP_LOGI(TAG, "Timestamp = %" PRIi64 " - event id = %" PRIi32, sensor_data->timestamp, id);
        break;
    }
}

void app_main(void)
{
#if BSP_CAPS_IMU
    bsp_sensor_config_t imu_config = {
        .type = IMU_ID,
        .mode = MODE_POLLING,
        .period = IMU_SAMPLING_PERIOD
    };
    ESP_ERROR_CHECK(bsp_sensor_init(&imu_config, &imu_sensor_handle));
    iot_sensor_handler_register(imu_sensor_handle, sensor_event_handler, NULL);
    iot_sensor_start(imu_sensor_handle);
#endif

#if BSP_CAPS_HUMITURE
    bsp_sensor_config_t humiture_config = {
        .type = HUMITURE_ID,
        .mode = MODE_POLLING,
        .period = HUMITURE_SAMPLING_PERIOD
    };

    ESP_ERROR_CHECK(bsp_sensor_init(&humiture_config, &humiture_sensor_handle));
    iot_sensor_handler_register(humiture_sensor_handle, sensor_event_handler, NULL);
    iot_sensor_start(humiture_sensor_handle);
#endif
}
