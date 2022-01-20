/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "driver/i2c.h"
#include "bh1750.h"

#define BH_1750_MEASUREMENT_ACCURACY    1.2    /*!< the typical measurement accuracy of  BH1750 sensor */

#define BH1750_POWER_DOWN        0x00    /*!< Command to set Power Down*/
#define BH1750_POWER_ON          0x01    /*!< Command to set Power On*/

typedef struct {
    i2c_port_t bus;
    uint16_t dev_addr;
} bh1750_dev_t;

static esp_err_t bh1750_write_byte(const bh1750_dev_t *const sens, const uint8_t byte)
{
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, byte, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

bh1750_handle_t bh1750_create(i2c_port_t port, const uint16_t dev_addr)
{
    bh1750_dev_t *sensor = (bh1750_dev_t *) calloc(1, sizeof(bh1750_dev_t));
    sensor->bus = port;
    sensor->dev_addr = dev_addr << 1;
    return (bh1750_handle_t) sensor;
}

esp_err_t bh1750_delete(bh1750_handle_t sensor)
{
    bh1750_dev_t *sens = (bh1750_dev_t *) sensor;
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
    esp_err_t ret;
    uint8_t bh1750_data_h, bh1750_data_l;
    bh1750_dev_t *sens = (bh1750_dev_t *) sensor;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_READ, true);
    assert(ESP_OK == ret);
    ret = i2c_master_read_byte(cmd, &bh1750_data_h, I2C_MASTER_ACK);
    assert(ESP_OK == ret);
    ret = i2c_master_read_byte(cmd, &bh1750_data_l, I2C_MASTER_NACK);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ESP_OK != ret) {
        return ret;
    }
    *data = (( bh1750_data_h << 8 | bh1750_data_l ) / BH_1750_MEASUREMENT_ACCURACY);
    return ESP_OK;
}
