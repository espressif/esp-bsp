/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bh1750.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h" // for pdMS_TO_TICKS

#define BH_1750_MEASUREMENT_ACCURACY    1.2    /*!< the typical measurement accuracy of  BH1750 sensor */

#define BH1750_POWER_DOWN        0x00    /*!< Command to set Power Down*/
#define BH1750_POWER_ON          0x01    /*!< Command to set Power On*/
#define I2C_CLK_SPEED            400000

typedef struct {
    i2c_master_dev_handle_t i2c_handle;
} bh1750_dev_t;

static esp_err_t bh1750_write_byte(const bh1750_dev_t *const sens, const uint8_t byte)
{
    return i2c_master_transmit(sens->i2c_handle, &byte, 1, pdMS_TO_TICKS(1000));
}

esp_err_t bh1750_create(i2c_master_bus_handle_t i2c_bus, const uint8_t dev_addr, bh1750_handle_t *handle_ret)
{
    esp_err_t ret = ESP_OK;
    bh1750_dev_t *sensor = (bh1750_dev_t *) calloc(1, sizeof(bh1750_dev_t));
    if (!sensor) {
        return ESP_ERR_NO_MEM;
    }

    // Add new I2C device
    const i2c_device_config_t i2c_dev_cfg = {
        .device_address = dev_addr,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    ret = i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &sensor->i2c_handle);
    if (ret != ESP_OK) {
        free(sensor);
        return ret;
    }

    assert(sensor->i2c_handle);
    *handle_ret = sensor;
    return ret;
}

esp_err_t bh1750_delete(bh1750_handle_t sensor)
{
    bh1750_dev_t *sens = (bh1750_dev_t *) sensor;
    if (sens->i2c_handle) {
        i2c_master_bus_rm_device(sens->i2c_handle);
    }
    free(sens);
    return ESP_OK;
}

esp_err_t bh1750_power_down(bh1750_handle_t sensor)
{
    bh1750_dev_t *sens = (bh1750_dev_t *) sensor;
    return bh1750_write_byte(sens, BH1750_POWER_DOWN);
}

esp_err_t bh1750_power_on(bh1750_handle_t sensor)
{
    bh1750_dev_t *sens = (bh1750_dev_t *) sensor;
    return bh1750_write_byte(sens, BH1750_POWER_ON);
}

esp_err_t bh1750_set_measure_time(bh1750_handle_t sensor, const uint8_t measure_time)
{
    bh1750_dev_t *sens = (bh1750_dev_t *) sensor;
    uint32_t i = 0;
    uint8_t buf[2] = {0x40, 0x60}; // constant part of the the MTreg
    buf[0] |= measure_time >> 5;
    buf[1] |= measure_time & 0x1F;
    for (i = 0; i < 2; i++) {
        esp_err_t ret = bh1750_write_byte(sens, buf[i]);
        if (ESP_OK != ret) {
            return ret;
        }
    }
    return ESP_OK;
}

esp_err_t bh1750_set_measure_mode(bh1750_handle_t sensor, const bh1750_measure_mode_t cmd_measure)
{
    bh1750_dev_t *sens = (bh1750_dev_t *) sensor;
    return bh1750_write_byte(sens, (uint8_t)cmd_measure);
}

esp_err_t bh1750_get_data(bh1750_handle_t sensor, float *const data)
{
    bh1750_dev_t *sens = (bh1750_dev_t *) sensor;
    uint8_t read_buffer[2];
    esp_err_t ret = i2c_master_receive(sens->i2c_handle, read_buffer, sizeof(read_buffer), pdMS_TO_TICKS(1000));
    if (ESP_OK != ret) {
        return ret;
    }
    *data = (( read_buffer[0] << 8 | read_buffer[1] ) / BH_1750_MEASUREMENT_ACCURACY);
    return ESP_OK;
}
