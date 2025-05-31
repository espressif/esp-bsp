/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "bh1750.h"

#define BH_1750_MEASUREMENT_ACCURACY    1.2    /*!< the typical measurement accuracy of  BH1750 sensor */

#define BH1750_POWER_DOWN        0x00    /*!< Command to set Power Down*/
#define BH1750_POWER_ON          0x01    /*!< Command to set Power On*/

const char *s_TAG = "BH1750";


bh1750_handle_t bh1750_create(i2c_master_bus_handle_t bus_handle, const uint8_t bh1750_addr)
{
    bh1750_handle_t bh1750_handle =  malloc(sizeof(bh1750_dev_t));
    if (bh1750_handle == NULL) {
        ESP_LOGE(s_TAG, "unable to allocate memory to initialize bh1750 handle");
        return bh1750_handle;
    }
    i2c_device_config_t bh1750_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = bh1750_addr,
        .scl_speed_hz = CONFIG_BH1750_I2C_CLK_SPEED,
    };

    ESP_LOGI(s_TAG, "adding BH1750 as device to bus\n");
    i2c_master_bus_add_device(bus_handle, &bh1750_config, &(bh1750_handle->bh1750_i2c_handle) );
    ESP_LOGI(s_TAG, "device added to bus\n");

    return bh1750_handle;
}

esp_err_t bh1750_delete(bh1750_handle_t *sensor)
{

    i2c_master_bus_rm_device( (*sensor)->bh1750_i2c_handle);

    free(*sensor);
    *sensor = NULL; // Not a dangling pointer anymore

    return ESP_OK;
}

esp_err_t bh1750_power_down(bh1750_handle_t sensor)
{
    uint8_t cmd = BH1750_POWER_DOWN;

    ESP_RETURN_ON_ERROR (i2c_master_transmit(sensor->bh1750_i2c_handle, &cmd, sizeof(cmd), 50), s_TAG, "unable to power down\n");
    return ESP_OK;
}

esp_err_t bh1750_power_on(bh1750_handle_t sensor)
{
    uint8_t cmd = BH1750_POWER_ON;
    ESP_RETURN_ON_ERROR (i2c_master_transmit(sensor->bh1750_i2c_handle, &cmd, sizeof(cmd), 50), s_TAG, "unable to power up\n");
    return ESP_OK;
}

esp_err_t bh1750_set_measure_time(bh1750_handle_t sensor, const uint8_t measure_time)
{
    uint8_t buf[2] = {0x40, 0x60}; // constant part of the the MTreg
    buf[0] |= measure_time >> 5;
    buf[1] |= measure_time & 0x1F;

    ESP_RETURN_ON_ERROR (i2c_master_transmit(sensor->bh1750_i2c_handle, buf, sizeof(buf), 50), s_TAG, "unable to set measurement time\n");
    return ESP_OK;
}


esp_err_t bh1750_set_measure_mode(bh1750_handle_t sensor, const bh1750_measure_mode_t cmd_measure)
{
    ESP_RETURN_ON_ERROR (i2c_master_transmit(sensor->bh1750_i2c_handle, (uint8_t *)cmd_measure, sizeof(cmd_measure), 50), s_TAG, "unable to set measurement mode\n");
    return ESP_OK;
}

esp_err_t bh1750_get_data(bh1750_handle_t sensor, float *const data)
{
    uint8_t bh1750_data[2];

    ESP_RETURN_ON_ERROR (i2c_master_receive(sensor->bh1750_i2c_handle, bh1750_data, 2, 1000 / portTICK_PERIOD_MS), s_TAG, "unable to read\n");

    *data = (( bh1750_data[0] << 8 | bh1750_data[1] ) / BH_1750_MEASUREMENT_ACCURACY);
    return ESP_OK;
}
