/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
*
* SPDX-License-Identifier: Apache-2.0
*/
/*
 * mpu60xx.c
 *
 *  Created on: 15-Oct-2024
 *      Author: Rohan Jeet <jeetrohan92@gmail.com>
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#include "freertos/FreeRTOS.h"

#include "mpu60xx.h"
#include "mpu60xx_pvt.h"


static const float_t s_g_value = 9.80665; // standard acceleration of gravity, multiplier to convert accelerometer reading to m/s


esp_err_t mpu60xx_read_sensor_raw (mpu60xx_handle_t mpu60xx, mpu60xx_reading_raw_t *sensor_raw_read)
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    uint8_t data[14];

    ESP_RETURN_ON_ERROR (mpu60xx_read_register(mpu_handle, MPU60xx_ACCEL_XOUTH_REG, data, 14), s_TAG, "Failed to read raw values");

    sensor_raw_read->accel.x_raw = ((uint16_t)data[0] << 8) | data[1];
    sensor_raw_read->accel.y_raw = ((uint16_t)data[2] << 8) | data[3];
    sensor_raw_read->accel.z_raw = ((uint16_t)data[4] << 8) | data[5];

    sensor_raw_read->temp_raw = ((uint16_t)data[6] << 8) | data[7];

    sensor_raw_read->gyro.x_raw  = ((uint16_t)data[8] << 8) | data[9];
    sensor_raw_read->gyro.y_raw  = ((uint16_t)data[10] << 8) | data[11];
    sensor_raw_read->gyro.z_raw  = ((uint16_t)data[12] << 8) | data[13];

    return ESP_OK;
}


esp_err_t mpu60xx_read_sensor(mpu60xx_handle_t mpu60xx, mpu60xx_reading_t *sensor_read)
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    mpu60xx_reading_raw_t  sensor_raw_read;
    ESP_RETURN_ON_ERROR (mpu60xx_read_sensor_raw(mpu_handle, &sensor_raw_read), "", "");

    sensor_read->temperature = 36.53 + ( (int16_t)sensor_raw_read.temp_raw ) / 340.0;

    mpu60xx_accel_range_t accel_range;
    ESP_RETURN_ON_ERROR ( getAccelerometerRange(mpu_handle, &accel_range), s_TAG, "Failed to read Accelerometer Range" );

    float accel_scale = 1;
    // select appropriate scale based on range
    switch (accel_range) {
    case MPU60XX_RANGE_2_G: accel_scale = 16384;
        break;

    case MPU60XX_RANGE_4_G: accel_scale = 8192;
        break;

    case MPU60XX_RANGE_8_G: accel_scale = 4096;
        break;

    case MPU60XX_RANGE_16_G: accel_scale = 2048;
        break;
    }

    // raw values provide results in terms of 'g'(standard acceleration of gravity),
    // multiply with 's_g_value' to convert into 'm/s^2'
    sensor_read->accel.x = s_g_value * ((float_t)  sensor_raw_read.accel.x_raw ) / accel_scale;
    sensor_read->accel.y  = s_g_value * ((float_t) sensor_raw_read.accel.y_raw ) / accel_scale;
    sensor_read->accel.z  = s_g_value * ((float_t) sensor_raw_read.accel.z_raw ) / accel_scale;


    mpu60xx_gyro_range_t gyro_range;
    ESP_RETURN_ON_ERROR ( getGyrometerRange(mpu_handle, &gyro_range), s_TAG, "Failed to get Gyrometer Range" );

    float gyro_scale = 1;

    // select appropriate scale based on range
    switch (gyro_range) {
    case MPU60XX_RANGE_250_DEG: gyro_scale = 131;
        break;

    case MPU60XX_RANGE_500_DEG: gyro_scale = 65.5;
        break;

    case MPU60XX_RANGE_1000_DEG: gyro_scale = 32.8;
        break;

    case MPU60XX_RANGE_2000_DEG: gyro_scale = 16.4;
        break;
    }

    // setup range dependant scaling
    sensor_read->gyro.x = ((float_t) sensor_raw_read.gyro.x_raw) / gyro_scale;
    sensor_read->gyro.y = ((float_t) sensor_raw_read.gyro.y_raw) / gyro_scale;
    sensor_read->gyro.z = ((float_t) sensor_raw_read.gyro.z_raw) / gyro_scale;


    return ESP_OK;
}


esp_err_t mpu60xx_init(mpu60xx_init_config_t config_handle, mpu60xx_handle_t mpu60xx)
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    ESP_LOGI(s_TAG, "mpu60xx is available\n");


    uint8_t cmd[2];

    //reset MPU60xx
    cmd[0] = MPU60xx_PWR_MGMT1_REG;
    cmd[1] = 0x80;
    ESP_RETURN_ON_ERROR (mpu60xx_write_register(mpu_handle, MPU60xx_PWR_MGMT1_REG, cmd, 2), "", "" );
    vTaskDelay(100 / portTICK_PERIOD_MS);

    if (config_handle.temp_sensor == true) { //enable temperature sensor, default is disabled
        cmd[0] = MPU60xx_PWR_MGMT1_REG;
        cmd[1] = 0x00;
        ESP_RETURN_ON_ERROR ( mpu60xx_write_register(mpu_handle, MPU60xx_PWR_MGMT1_REG, cmd, 2), "", "");
    }


    mpu60xx_setGyrometerRange(mpu_handle, config_handle.gyro_res);

    mpu60xx_setAccelerometerRange(mpu_handle, config_handle.accel_res);

    mpu60xx_setSampleRate(mpu_handle, config_handle.sample_rate, config_handle.dlpf_bw);

    cmd[0] = MPU60xx_INT_EN_REG;
    cmd[1] = 0x00;
    mpu60xx_write_register(mpu_handle, MPU60xx_INT_EN_REG, cmd, 2);

    cmd[0] = MPU60xx_USER_CTRL_REG;
    cmd[1] = 0x00;
    mpu60xx_write_register(mpu_handle, MPU60xx_USER_CTRL_REG, cmd, 2);

    ESP_LOGI(s_TAG, "mpu60xx initialization complete\n");

    return ESP_OK;
}

mpu60xx_handle_t mpu60xx_create( i2c_master_bus_handle_t bus_handle, uint8_t mpu60xx_address )
{
    mpu60xx_handle_t mpu60xx_handle = malloc(sizeof(mpu60xx_dev_t));

    if (mpu60xx_handle == NULL) {
        ESP_LOGE(s_TAG, "unable to allocate memory to initialize mpu60xx handle");
        return mpu60xx_handle;
    }

    i2c_device_config_t mpu60xxconfig = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = mpu60xx_address,
        .scl_speed_hz = CONFIG_MPU60xx_I2C_CLK_SPEED,
    };

    mpu60xx_dev_t *my_mpu60xx_handle = (mpu60xx_dev_t *)mpu60xx_handle;

    ESP_LOGI(s_TAG, "adding mpu60xx as device to bus\n");
    i2c_master_bus_add_device(bus_handle, &mpu60xxconfig, &(my_mpu60xx_handle->i2c_dev_handle) );
    ESP_LOGI(s_TAG, "device added to bus\n");

    my_mpu60xx_handle->bus_handle = bus_handle;
    my_mpu60xx_handle->i2c_address = mpu60xx_address;
    return mpu60xx_handle;
}


esp_err_t mpu60xx_remove( mpu60xx_handle_t *mpu60xx)
{
    ESP_RETURN_ON_FALSE( (*mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t **mpu60xx_ptr = (mpu60xx_dev_t **) mpu60xx;

    i2c_master_bus_rm_device( (*mpu60xx_ptr)->i2c_dev_handle);
    free(*mpu60xx_ptr);
    *mpu60xx_ptr = NULL; // now MPU60xx handle is not a dangling pointer
    return ESP_OK;
}
