/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_check.h"
#include "icm42670.h"

#define I2C_CLK_SPEED 400000

#define ALPHA                       0.99f        /*!< Weight of gyroscope */
#define RAD_TO_DEG                  57.27272727f /*!< Radians to degrees */

#define ICM42607_ID 0x60
#define ICM42670_ID 0x67

/* ICM42670 register */
#define ICM42670_WHOAMI         0x75
#define ICM42670_GYRO_CONFIG0   0x20
#define ICM42670_ACCEL_CONFIG0  0x21
#define ICM42670_TEMP_CONFIG    0x22
#define ICM42670_PWR_MGMT0      0x1F
#define ICM42670_TEMP_DATA      0x09
#define ICM42670_ACCEL_DATA     0x0B
#define ICM42670_GYRO_DATA      0x11

/* Sensitivity of the gyroscope */
#define GYRO_FS_2000_SENSITIVITY (16.4)
#define GYRO_FS_1000_SENSITIVITY (32.8)
#define GYRO_FS_500_SENSITIVITY  (65.5)
#define GYRO_FS_250_SENSITIVITY  (131.0)

/* Sensitivity of the accelerometer */
#define ACCE_FS_16G_SENSITIVITY (2048)
#define ACCE_FS_8G_SENSITIVITY  (4096)
#define ACCE_FS_4G_SENSITIVITY  (8192)
#define ACCE_FS_2G_SENSITIVITY  (16384)

/*******************************************************************************
* Types definitions
*******************************************************************************/

typedef struct {
    i2c_master_dev_handle_t i2c_handle;
    uint32_t counter;
    float dt;  /*!< delay time between two measurements, dt should be small (ms level) */
    struct timeval *timer;
} icm42670_dev_t;

/*******************************************************************************
* Function definitions
*******************************************************************************/
static esp_err_t icm42670_write(icm42670_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *data_buf, const uint8_t data_len);
static esp_err_t icm42670_read(icm42670_handle_t sensor, const uint8_t reg_start_addr, uint8_t *data_buf, const uint8_t data_len);

static esp_err_t icm42670_get_raw_value(icm42670_handle_t sensor, uint8_t reg, icm42670_raw_value_t *value);

/*******************************************************************************
* Local variables
*******************************************************************************/
static const char *TAG = "ICM42670";

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t icm42670_create(i2c_master_bus_handle_t i2c_bus, const uint8_t dev_addr, icm42670_handle_t *handle_ret)
{
    esp_err_t ret = ESP_OK;

    // Allocate memory and init the driver object
    icm42670_dev_t *sensor = (icm42670_dev_t *) calloc(1, sizeof(icm42670_dev_t));
    struct timeval *timer = (struct timeval *) calloc(1, sizeof(struct timeval));
    ESP_RETURN_ON_FALSE(sensor != NULL && timer != NULL, ESP_ERR_NO_MEM, TAG, "Not enough memory");
    sensor->timer = timer;

    // Add new I2C device
    const i2c_device_config_t i2c_dev_cfg = {
        .device_address = dev_addr,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &sensor->i2c_handle), err, TAG, "Failed to add new I2C device");
    assert(sensor->i2c_handle);

    // Check device presence
    uint8_t dev_id = 0;
    icm42670_get_deviceid(sensor, &dev_id);
    ESP_GOTO_ON_FALSE(dev_id == ICM42607_ID || dev_id == ICM42670_ID, ESP_ERR_NOT_FOUND, err, TAG, "Incorrect Device ID (0x%02x).", dev_id);

    ESP_LOGD(TAG, "Found device %s, ID: 0x%02x", (dev_id == ICM42607_ID ? "ICM42607" : "ICM42670"), dev_id);
    *handle_ret = sensor;
    return ret;

err:
    icm42670_delete(sensor);
    return ret;
}

void icm42670_delete(icm42670_handle_t sensor)
{
    icm42670_dev_t *sens = (icm42670_dev_t *) sensor;

    if (sens->i2c_handle) {
        i2c_master_bus_rm_device(sens->i2c_handle);
    }

    if (sens->timer) {
        free(sens->timer);
    }

    free(sens);
}

esp_err_t icm42670_get_deviceid(icm42670_handle_t sensor, uint8_t *deviceid)
{
    esp_err_t ret = ESP_FAIL;

    assert(deviceid != NULL);

    for (int i = 0; (i < 5 && ret != ESP_OK); i++) {
        ret = icm42670_read(sensor, ICM42670_WHOAMI, deviceid, 1);
    }

    return ret;
}

esp_err_t icm42670_config(icm42670_handle_t sensor, const icm42670_cfg_t *config)
{
    uint8_t data[2];

    assert(config != NULL);

    /* Gyroscope */
    data[0] = ((config->gyro_fs & 0x03) << 5) | (config->gyro_odr & 0x0F);
    /* Accelerometer */
    data[1] = ((config->acce_fs & 0x03) << 5) | (config->acce_odr & 0x0F);

    return icm42670_write(sensor, ICM42670_GYRO_CONFIG0, data, sizeof(data));
}

esp_err_t icm42670_acce_set_pwr(icm42670_handle_t sensor, icm42670_acce_pwr_t state)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t data;

    ret = icm42670_read(sensor, ICM42670_PWR_MGMT0, &data, 1);
    if (ret == ESP_OK) {
        data |= (state & 0x03);

        ret = icm42670_write(sensor, ICM42670_PWR_MGMT0, &data, sizeof(data));
    }

    return ret;
}

esp_err_t icm42670_gyro_set_pwr(icm42670_handle_t sensor, icm42670_gyro_pwr_t state)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t data;

    ret = icm42670_read(sensor, ICM42670_PWR_MGMT0, &data, 1);
    if (ret == ESP_OK) {
        data |= ((state & 0x03) << 2);

        ret = icm42670_write(sensor, ICM42670_PWR_MGMT0, &data, sizeof(data));
    }

    return ret;
}

esp_err_t icm42670_get_acce_sensitivity(icm42670_handle_t sensor, float *sensitivity)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t acce_fs;

    assert(sensitivity != NULL);

    *sensitivity = 0;

    ret = icm42670_read(sensor, ICM42670_ACCEL_CONFIG0, &acce_fs, 1);
    if (ret == ESP_OK) {
        acce_fs = (acce_fs >> 5) & 0x03;
        switch (acce_fs) {
        case ACCE_FS_16G:
            *sensitivity = ACCE_FS_16G_SENSITIVITY;
            break;
        case ACCE_FS_8G:
            *sensitivity = ACCE_FS_8G_SENSITIVITY;
            break;
        case ACCE_FS_4G:
            *sensitivity = ACCE_FS_4G_SENSITIVITY;
            break;
        case ACCE_FS_2G:
            *sensitivity = ACCE_FS_2G_SENSITIVITY;
            break;
        }
    }

    return ret;
}

esp_err_t icm42670_get_gyro_sensitivity(icm42670_handle_t sensor, float *sensitivity)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t gyro_fs;

    assert(sensitivity != NULL);

    *sensitivity = 0;

    ret = icm42670_read(sensor, ICM42670_GYRO_CONFIG0, &gyro_fs, 1);
    if (ret == ESP_OK) {
        gyro_fs = (gyro_fs >> 5) & 0x03;
        switch (gyro_fs) {
        case GYRO_FS_2000DPS:
            *sensitivity = GYRO_FS_2000_SENSITIVITY;
            break;
        case GYRO_FS_1000DPS:
            *sensitivity = GYRO_FS_1000_SENSITIVITY;
            break;
        case GYRO_FS_500DPS:
            *sensitivity = GYRO_FS_500_SENSITIVITY;
            break;
        case GYRO_FS_250DPS:
            *sensitivity = GYRO_FS_250_SENSITIVITY;
            break;
        }
    }

    return ret;
}

esp_err_t icm42670_get_temp_raw_value(icm42670_handle_t sensor, uint16_t *value)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t data[2];

    assert(value != NULL);

    *value = 0;

    ret = icm42670_read(sensor, ICM42670_TEMP_DATA, data, sizeof(data));
    if (ret == ESP_OK) {
        *value = (uint16_t)((data[0] << 8) + data[1]);
    }

    return ret;
}

esp_err_t icm42670_get_acce_raw_value(icm42670_handle_t sensor, icm42670_raw_value_t *value)
{
    return icm42670_get_raw_value(sensor, ICM42670_ACCEL_DATA, value);
}

esp_err_t icm42670_get_gyro_raw_value(icm42670_handle_t sensor, icm42670_raw_value_t *value)
{
    return icm42670_get_raw_value(sensor, ICM42670_GYRO_DATA, value);
}

esp_err_t icm42670_get_acce_value(icm42670_handle_t sensor, icm42670_value_t *value)
{
    esp_err_t ret;
    float sensitivity;
    icm42670_raw_value_t raw_value;

    assert(value != NULL);

    value->x = 0;
    value->y = 0;
    value->z = 0;

    ret = icm42670_get_acce_sensitivity(sensor, &sensitivity);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get sensitivity error!");

    ret = icm42670_get_acce_raw_value(sensor, &raw_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get raw value error!");

    value->x = raw_value.x / sensitivity;
    value->y = raw_value.y / sensitivity;
    value->z = raw_value.z / sensitivity;

    return ESP_OK;
}

esp_err_t icm42670_get_gyro_value(icm42670_handle_t sensor, icm42670_value_t *value)
{
    esp_err_t ret;
    float sensitivity;
    icm42670_raw_value_t raw_value;

    assert(value != NULL);

    value->x = 0;
    value->y = 0;
    value->z = 0;

    ret = icm42670_get_gyro_sensitivity(sensor, &sensitivity);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get sensitivity error!");

    ret = icm42670_get_gyro_raw_value(sensor, &raw_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get raw value error!");

    value->x = raw_value.x / sensitivity;
    value->y = raw_value.y / sensitivity;
    value->z = raw_value.z / sensitivity;

    return ESP_OK;
}

esp_err_t icm42670_get_temp_value(icm42670_handle_t sensor, float *value)
{
    esp_err_t ret;
    uint16_t raw_value;

    assert(value != NULL);

    *value = 0;

    ret = icm42670_get_temp_raw_value(sensor, &raw_value);
    ESP_RETURN_ON_ERROR(ret, TAG, "Get raw value error!");

    *value = ((float)raw_value / 128.0) + 25.0;

    return ESP_OK;
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static esp_err_t icm42670_get_raw_value(icm42670_handle_t sensor, uint8_t reg, icm42670_raw_value_t *value)
{
    esp_err_t ret = ESP_FAIL;
    uint8_t data[6];

    assert(value != NULL);

    value->x = 0;
    value->y = 0;
    value->z = 0;

    ret = icm42670_read(sensor, reg, data, sizeof(data));
    if (ret == ESP_OK) {
        value->x = (int16_t)((data[0] << 8) + data[1]);
        value->y = (int16_t)((data[2] << 8) + data[3]);
        value->z = (int16_t)((data[4] << 8) + data[5]);
    }

    return ret;
}

static esp_err_t icm42670_write(icm42670_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *data_buf, const uint8_t data_len)
{
    icm42670_dev_t *sens = (icm42670_dev_t *) sensor;
    assert(sens);

    assert(data_len < 5);
    uint8_t write_buff[5] = {reg_start_addr};
    memcpy(&write_buff[1], data_buf, data_len);
    return i2c_master_transmit(sens->i2c_handle, write_buff, data_len + 1, -1);
}

static esp_err_t icm42670_read(icm42670_handle_t sensor, const uint8_t reg_start_addr, uint8_t *data_buf, const uint8_t data_len)
{
    uint8_t reg_buff[] = {reg_start_addr};
    icm42670_dev_t *sens = (icm42670_dev_t *) sensor;
    assert(sens);

    /* Write register number and read data */
    return i2c_master_transmit_receive(sens->i2c_handle, reg_buff, sizeof(reg_buff), data_buf, data_len, -1);
}

esp_err_t icm42670_complimentory_filter(icm42670_handle_t sensor, const icm42670_value_t *const acce_value,
                                        const icm42670_value_t *const gyro_value, complimentary_angle_t *const complimentary_angle)
{
    float acce_angle[2];
    float gyro_angle[2];
    float gyro_rate[2];
    icm42670_dev_t *sens = (icm42670_dev_t *) sensor;

    sens->counter++;
    if (sens->counter == 1) {
        acce_angle[0] = (atan2(acce_value->y, acce_value->z) * RAD_TO_DEG);
        acce_angle[1] = (atan2(acce_value->x, acce_value->z) * RAD_TO_DEG);
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

    acce_angle[0] = (atan2(acce_value->y, acce_value->z) * RAD_TO_DEG);
    acce_angle[1] = (atan2(acce_value->x, acce_value->z) * RAD_TO_DEG);

    gyro_rate[0] = gyro_value->x;
    gyro_rate[1] = gyro_value->y;
    gyro_angle[0] = gyro_rate[0] * sens->dt;
    gyro_angle[1] = gyro_rate[1] * sens->dt;

    complimentary_angle->roll = (ALPHA * (complimentary_angle->roll + gyro_angle[0])) + ((1 - ALPHA) * acce_angle[0]);
    complimentary_angle->pitch = (ALPHA * (complimentary_angle->pitch + gyro_angle[1])) + ((1 - ALPHA) * acce_angle[1]);

    return ESP_OK;
}
