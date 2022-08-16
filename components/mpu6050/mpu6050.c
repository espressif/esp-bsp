/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "driver/i2c.h"
#include "mpu6050.h"

#define ALPHA                       0.99f        /*!< Weight of gyroscope */
#define RAD_TO_DEG                  57.27272727f /*!< Radians to degrees */

/* MPU6050 register */
#define MPU6050_GYRO_CONFIG         0x1Bu
#define MPU6050_ACCEL_CONFIG        0x1Cu
#define MPU6050_ACCEL_XOUT_H        0x3Bu
#define MPU6050_GYRO_XOUT_H         0x43u
#define MPU6050_TEMP_XOUT_H         0x41u
#define MPU6050_PWR_MGMT_1          0x6Bu
#define MPU6050_WHO_AM_I            0x75u

typedef struct {
    i2c_port_t bus;
    uint16_t dev_addr;
    uint32_t counter;
    float dt;  /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval *timer;
} mpu6050_dev_t;

static esp_err_t mpu6050_write(mpu6050_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;
    esp_err_t  ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t mpu6050_read(mpu6050_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;
    esp_err_t  ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_READ, true);
    assert(ESP_OK == ret);
    ret = i2c_master_read(cmd, data_buf, data_len, I2C_MASTER_LAST_NACK);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

mpu6050_handle_t mpu6050_create(i2c_port_t port, const uint16_t dev_addr)
{
    mpu6050_dev_t *sensor = (mpu6050_dev_t *) calloc(1, sizeof(mpu6050_dev_t));
    sensor->bus = port;
    sensor->dev_addr = dev_addr << 1;
    sensor->counter = 0;
    sensor->dt = 0;
    sensor->timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    return (mpu6050_handle_t) sensor;
}

void mpu6050_delete(mpu6050_handle_t sensor)
{
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;
    free(sens);
}

esp_err_t mpu6050_get_deviceid(mpu6050_handle_t sensor, uint8_t *const deviceid)
{
    return mpu6050_read(sensor, MPU6050_WHO_AM_I, deviceid, 1);
}

esp_err_t mpu6050_wake_up(mpu6050_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6050_read(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp &= (~BIT6);
    ret = mpu6050_write(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_sleep(mpu6050_handle_t sensor)
{
    esp_err_t ret;
    uint8_t tmp;
    ret = mpu6050_read(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp |= BIT6;
    ret = mpu6050_write(sensor, MPU6050_PWR_MGMT_1, &tmp, 1);
    return ret;
}

esp_err_t mpu6050_config(mpu6050_handle_t sensor, const mpu6050_acce_fs_t acce_fs, const mpu6050_gyro_fs_t gyro_fs)
{
    uint8_t config_regs[2] = {gyro_fs << 3,  acce_fs << 3};
    return mpu6050_write(sensor, MPU6050_GYRO_CONFIG, config_regs, sizeof(config_regs));
}

esp_err_t mpu6050_get_acce_sensitivity(mpu6050_handle_t sensor, float *const acce_sensitivity)
{
    esp_err_t ret;
    uint8_t acce_fs;
    ret = mpu6050_read(sensor, MPU6050_ACCEL_CONFIG, &acce_fs, 1);
    acce_fs = (acce_fs >> 3) & 0x03;
    switch (acce_fs) {
    case ACCE_FS_2G:
        *acce_sensitivity = 16384;
        break;

    case ACCE_FS_4G:
        *acce_sensitivity = 8192;
        break;

    case ACCE_FS_8G:
        *acce_sensitivity = 4096;
        break;

    case ACCE_FS_16G:
        *acce_sensitivity = 2048;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_get_gyro_sensitivity(mpu6050_handle_t sensor, float *const gyro_sensitivity)
{
    esp_err_t ret;
    uint8_t gyro_fs;
    ret = mpu6050_read(sensor, MPU6050_GYRO_CONFIG, &gyro_fs, 1);
    gyro_fs = (gyro_fs >> 3) & 0x03;
    switch (gyro_fs) {
    case GYRO_FS_250DPS:
        *gyro_sensitivity = 131;
        break;

    case GYRO_FS_500DPS:
        *gyro_sensitivity = 65.5;
        break;

    case GYRO_FS_1000DPS:
        *gyro_sensitivity = 32.8;
        break;

    case GYRO_FS_2000DPS:
        *gyro_sensitivity = 16.4;
        break;

    default:
        break;
    }
    return ret;
}

esp_err_t mpu6050_get_raw_acce(mpu6050_handle_t sensor, mpu6050_raw_acce_value_t *const raw_acce_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_ACCEL_XOUT_H, data_rd, sizeof(data_rd));

    raw_acce_value->raw_acce_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_acce_value->raw_acce_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_acce_value->raw_acce_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));
    return ret;
}

esp_err_t mpu6050_get_raw_gyro(mpu6050_handle_t sensor, mpu6050_raw_gyro_value_t *const raw_gyro_value)
{
    uint8_t data_rd[6];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_GYRO_XOUT_H, data_rd, sizeof(data_rd));

    raw_gyro_value->raw_gyro_x = (int16_t)((data_rd[0] << 8) + (data_rd[1]));
    raw_gyro_value->raw_gyro_y = (int16_t)((data_rd[2] << 8) + (data_rd[3]));
    raw_gyro_value->raw_gyro_z = (int16_t)((data_rd[4] << 8) + (data_rd[5]));

    return ret;
}

esp_err_t mpu6050_get_acce(mpu6050_handle_t sensor, mpu6050_acce_value_t *const acce_value)
{
    esp_err_t ret;
    float acce_sensitivity;
    mpu6050_raw_acce_value_t raw_acce;

    ret = mpu6050_get_acce_sensitivity(sensor, &acce_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = mpu6050_get_raw_acce(sensor, &raw_acce);
    if (ret != ESP_OK) {
        return ret;
    }

    acce_value->acce_x = raw_acce.raw_acce_x / acce_sensitivity;
    acce_value->acce_y = raw_acce.raw_acce_y / acce_sensitivity;
    acce_value->acce_z = raw_acce.raw_acce_z / acce_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6050_get_gyro(mpu6050_handle_t sensor, mpu6050_gyro_value_t *const gyro_value)
{
    esp_err_t ret;
    float gyro_sensitivity;
    mpu6050_raw_gyro_value_t raw_gyro;

    ret = mpu6050_get_gyro_sensitivity(sensor, &gyro_sensitivity);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = mpu6050_get_raw_gyro(sensor, &raw_gyro);
    if (ret != ESP_OK) {
        return ret;
    }

    gyro_value->gyro_x = raw_gyro.raw_gyro_x / gyro_sensitivity;
    gyro_value->gyro_y = raw_gyro.raw_gyro_y / gyro_sensitivity;
    gyro_value->gyro_z = raw_gyro.raw_gyro_z / gyro_sensitivity;
    return ESP_OK;
}

esp_err_t mpu6050_get_temp(mpu6050_handle_t sensor, mpu6050_temp_value_t *const temp_value)
{
    uint8_t data_rd[2];
    esp_err_t ret = mpu6050_read(sensor, MPU6050_TEMP_XOUT_H, data_rd, sizeof(data_rd));
    temp_value->temp = (int16_t)((data_rd[0] << 8) | (data_rd[1])) / 340.00 + 36.53;
    return ret;
}

esp_err_t mpu6050_complimentory_filter(mpu6050_handle_t sensor, const mpu6050_acce_value_t *const acce_value,
                                       const mpu6050_gyro_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle)
{
    float acce_angle[2];
    float gyro_angle[2];
    float gyro_rate[2];
    mpu6050_dev_t *sens = (mpu6050_dev_t *) sensor;

    sens->counter++;
    if (sens->counter == 1) {
        acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
        acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);
        complimentary_angle->roll = acce_angle[0];
        complimentary_angle->pitch = acce_angle[1];
        gettimeofday(sens->timer, NULL);
        return ESP_OK;
    }

    struct timeval now, dt_t;
    gettimeofday(&now, NULL);
    timersub(&now, sens->timer, &dt_t);
    sens->dt = (float) (dt_t.tv_sec) + (float)dt_t.tv_usec / 1000000;
    gettimeofday(sens->timer, NULL);

    acce_angle[0] = (atan2(acce_value->acce_y, acce_value->acce_z) * RAD_TO_DEG);
    acce_angle[1] = (atan2(acce_value->acce_x, acce_value->acce_z) * RAD_TO_DEG);

    gyro_rate[0] = gyro_value->gyro_x;
    gyro_rate[1] = gyro_value->gyro_y;
    gyro_angle[0] = gyro_rate[0] * sens->dt;
    gyro_angle[1] = gyro_rate[1] * sens->dt;

    complimentary_angle->roll = (ALPHA * (complimentary_angle->roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
    complimentary_angle->pitch = (ALPHA * (complimentary_angle->pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);

    return ESP_OK;
}
