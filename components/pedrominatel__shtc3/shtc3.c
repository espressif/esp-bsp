/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "include/shtc3.h"
#include "shtc3.h"

static const char *TAG = "SHTC3";

i2c_master_dev_handle_t shtc3_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = dev_speed,
    };

    i2c_master_dev_handle_t dev_handle;

    // Add device to the I2C bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    return dev_handle;
}

esp_err_t shtc3_device_delete(i2c_master_dev_handle_t dev_handle)
{
    return i2c_master_bus_rm_device(dev_handle);
}

static esp_err_t shtc3_wake(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    shtc3_register_w_t reg_addr = SHTC3_REG_WAKE;
    uint8_t read_reg[2] = { reg_addr >> 8, reg_addr & 0xff };
    
    ret = i2c_master_transmit(dev_handle, read_reg, 2, -1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to wake up SHTC3 sensor");
    esp_rom_delay_us(SHTC3_WARMUP_US); //We need some time for sensor warm-up after wake
    
    return ret;
}

static esp_err_t shtc3_sleep(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret;
    shtc3_register_w_t reg_addr = SHTC3_REG_SLEEP;
    uint8_t read_reg[2] = { reg_addr >> 8, reg_addr & 0xff };

    ret = i2c_master_transmit(dev_handle, read_reg, 2, -1);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to put SHTC3 sensor to sleep");
    
    return ret;
}

esp_err_t shtc3_get_id(i2c_master_dev_handle_t dev_handle, uint8_t *id)
{
    esp_err_t ret;
    shtc3_register_rw_t reg_addr = SHTC3_REG_READ_ID;
    uint8_t read_reg[2] = { reg_addr >> 8, reg_addr & 0xff };
    uint8_t b_read[2] = {0};

    shtc3_wake(dev_handle);
    ret = i2c_master_transmit_receive(dev_handle, read_reg, 2, b_read, 2, 200);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read SHTC3 ID");

    // Copy the read ID to the provided id pointer
    id[0] = b_read[0];
    id[1] = b_read[1];

    return ret;
}

 esp_err_t shtc3_get_th(i2c_master_dev_handle_t dev_handle, shtc3_register_rw_t reg, float *data1, float *data2)
{
    esp_err_t ret;
    uint8_t b_read[6] = {0};
    uint8_t read_reg[2] = { reg >> 8, reg & 0xff };

    shtc3_wake(dev_handle);
    // Read 4 bytes of data from the sensor
    ret = i2c_master_transmit_receive(dev_handle, read_reg, 2, b_read, 6, 200);
    ESP_RETURN_ON_ERROR(ret, TAG, "Failed to read data from SHTC3 sensor");
    shtc3_sleep(dev_handle);

    // Convert the data
    *data1 = ((((b_read[0] * 256.0) + b_read[1]) * 175) / 65535.0) - 45;
    *data2 = ((((b_read[3] * 256.0) + b_read[4]) * 100) / 65535.0);

    return ret;
}
