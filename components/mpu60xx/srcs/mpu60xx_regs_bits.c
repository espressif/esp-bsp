/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * mpu60xx.c
 *
 *  Created on: 4-Jun-2025
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

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "mpu60xx.h"
#include "mpu60xx_regs_bits.h"
#include "mpu60xx_pvt.h"

esp_err_t mpu60xx_setAccelerometerRange(mpu60xx_handle_t mpu60xx, mpu60xx_accel_range_t range)
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    uint8_t cmd[2];
    cmd[0] = MPU60xx_ACCEL_CFG_REG;
    cmd[1] = range;
    esp_err_t  ret = mpu60xx_write_register(mpu_handle, MPU60xx_ACCEL_CFG_REG, cmd, 2);
    return ret;
}

esp_err_t getAccelerometerRange(mpu60xx_handle_t mpu60xx, mpu60xx_accel_range_t *range )
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    uint8_t reg_value;

    esp_err_t  ret = mpu60xx_read_register(mpu_handle, MPU60xx_ACCEL_CFG_REG, &reg_value, 1);

    if (ret == ESP_OK) {
        *range = reg_value & 0x18;
    }
    return ret;
}

esp_err_t mpu60xx_setGyrometerRange(mpu60xx_handle_t mpu60xx, mpu60xx_gyro_range_t range)
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    uint8_t cmd[2];
    cmd[0] = MPU60xx_GYRO_CFG_REG;
    cmd[1] = range;
    esp_err_t  ret = mpu60xx_write_register(mpu_handle, MPU60xx_GYRO_CFG_REG, cmd, 2);
    return ret;
}

esp_err_t getGyrometerRange(mpu60xx_handle_t mpu60xx, mpu60xx_gyro_range_t *range )
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    uint8_t reg_value;

    esp_err_t  ret = mpu60xx_read_register(mpu_handle, MPU60xx_GYRO_CFG_REG, &reg_value, 1);

    if (ret == ESP_OK) {
        *range = reg_value & 0x18;
    }
    return ret;
}

esp_err_t mpu60xx_setSampleRate ( mpu60xx_handle_t mpu60xx, uint32_t sample_rate, mpu60xx_lowpass_t dlpf_bw)
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    uint8_t cmd[2], smpl_rt_div = 0;
    uint32_t gy_out_rate = 8000;
    esp_err_t ret;

    if (sample_rate < 5) {
        sample_rate = 5;    //minimum valid value =4.94
    }

    if (dlpf_bw != MPU60XX_BAND_260_HZ) {
        if (sample_rate > 1000) {
            sample_rate = 1000;    // max when DLPF used
        }
        gy_out_rate = 1000;
    } else if (sample_rate > 8000) {
        sample_rate = 8000;    // max when DLPF not used
    }


    ret = en_DLPF(mpu_handle, sample_rate);
    if (ret != ESP_OK) {
        return ret;
    }

    smpl_rt_div = (uint8_t) ( gy_out_rate / sample_rate) - 1;
    cmd[0] = MPU60xx_SMPRT_DIV;
    cmd[1] = smpl_rt_div;
    ret = mpu60xx_write_register(mpu_handle, MPU60xx_SMPRT_DIV, cmd, 2);
    return ret;
}

esp_err_t en_DLPF( mpu60xx_handle_t mpu60xx, mpu60xx_lowpass_t b_width)
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    uint8_t reg_value, cmd[2];
    esp_err_t ret;

    ret = mpu60xx_read_register(mpu_handle, MPU60xx_CFG_REG, &reg_value, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(s_TAG, "unable to read MPU60xx_CFG_REG  register");
        return ret;
    }

    reg_value = reg_value &  ((uint8_t)(~7));
    reg_value |= b_width;

    cmd[0] = MPU60xx_CFG_REG;
    cmd[1] = reg_value;
    ret = mpu60xx_write_register(mpu_handle, MPU60xx_CFG_REG, cmd, 2);

    if (ret != ESP_OK) {
        ESP_LOGE(s_TAG, "unable to write to MPU60xx_CFG_REG  register");
    }
    return ret;

}

esp_err_t en_DHPF( mpu60xx_handle_t mpu60xx, mpu60xx_highpass_t bandwidth)
{
    ESP_RETURN_ON_FALSE((mpu60xx != NULL), ESP_ERR_INVALID_ARG, s_TAG, "mpu60xx handle is not initiated");
    mpu60xx_dev_t *mpu_handle = (mpu60xx_dev_t *) mpu60xx;
    ESP_RETURN_ON_ERROR (i2c_master_probe(mpu_handle->bus_handle, mpu_handle->i2c_address, 50), s_TAG, "mpu60xx not detected on i2c");

    esp_err_t ret;
    uint8_t reg_value, cmd[2];
    ret = mpu60xx_read_register(mpu_handle, MPU60xx_ACCEL_CFG_REG, &reg_value, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(s_TAG, "unable to read MPU60xx_CFG_REG  register");
        return ret;
    }

    reg_value = reg_value &  ((uint8_t)(~7));
    reg_value |= bandwidth;

    cmd[0] = MPU60xx_ACCEL_CFG_REG;
    cmd[1] = reg_value;
    ret = mpu60xx_write_register(mpu_handle, MPU60xx_ACCEL_CFG_REG, cmd, 2);

    if (ret != ESP_OK) {
        ESP_LOGE(s_TAG, "unable to write to MPU60xx_CFG_REG  register");
    }

    return ret;
}
