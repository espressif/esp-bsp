/*
 * SPDX-FileCopyrightText: 2023-2026 Espressif Systems (Shanghai) CO LTD
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
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "icm42670.h"

#define I2C_CLK_SPEED 400000

#define ALPHA                       0.97f        /*!< Weight of gyroscope */
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
    bool initialized_filter;
    uint64_t previous_measurement_us;
    complimentary_angle_t previous_measurement;
} icm42670_dev_t;

/*******************************************************************************
* Function definitions
*******************************************************************************/
static esp_err_t icm42670_write(icm42670_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *data_buf,
                                const uint8_t data_len);
static esp_err_t icm42670_read(icm42670_handle_t sensor, const uint8_t reg_start_addr, uint8_t *data_buf,
                               const uint8_t data_len);

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
    ESP_RETURN_ON_FALSE(sensor != NULL, ESP_ERR_NO_MEM, TAG, "Not enough memory");

    // Add new I2C device
    const i2c_device_config_t i2c_dev_cfg = {
        .device_address = dev_addr,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &sensor->i2c_handle), err, TAG,
                      "Failed to add new I2C device");
    assert(sensor->i2c_handle);

    // Check device presence
    uint8_t dev_id = 0;
    icm42670_get_deviceid(sensor, &dev_id);
    ESP_GOTO_ON_FALSE(dev_id == ICM42607_ID
                      || dev_id == ICM42670_ID, ESP_ERR_NOT_FOUND, err, TAG, "Incorrect Device ID (0x%02x).", dev_id);

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

    *value = ((int16_t)raw_value / 128.0f) + 25.0f;

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

static esp_err_t icm42670_write(icm42670_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *data_buf,
                                const uint8_t data_len)
{
    icm42670_dev_t *sens = (icm42670_dev_t *) sensor;
    assert(sens);

    assert(data_len < 5);
    uint8_t write_buff[5] = {reg_start_addr};
    memcpy(&write_buff[1], data_buf, data_len);
    return i2c_master_transmit(sens->i2c_handle, write_buff, data_len + 1, -1);
}

static esp_err_t icm42670_read(icm42670_handle_t sensor, const uint8_t reg_start_addr, uint8_t *data_buf,
                               const uint8_t data_len)
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
    icm42670_dev_t *sens = (icm42670_dev_t *) sensor;
    float measurement_delta;
    uint64_t current_time_us;
    float acc_roll_angle;
    float acc_pitch_angle;
    float gyro_roll_angle;
    float gyro_pitch_angle;

    acc_roll_angle = (atan2(acce_value->y,
                            sqrt(acce_value->x * acce_value->x + acce_value->z * acce_value->z)) * RAD_TO_DEG);
    acc_pitch_angle = (atan2(-acce_value->x,
                             sqrt(acce_value->y * acce_value->y + acce_value->z * acce_value->z)) * RAD_TO_DEG);

    if (!sens->initialized_filter) {
        sens->initialized_filter = true;
        sens->previous_measurement_us = esp_timer_get_time();
        sens->previous_measurement.roll = acc_roll_angle;
        sens->previous_measurement.pitch = acc_pitch_angle;
    }

    current_time_us = esp_timer_get_time();
    measurement_delta = (current_time_us - sens->previous_measurement_us) / 1000000.0f;
    sens->previous_measurement_us = current_time_us;

    gyro_roll_angle = gyro_value->x * measurement_delta;
    gyro_pitch_angle = gyro_value->y * measurement_delta;

    complimentary_angle->roll = (ALPHA * (sens->previous_measurement.roll + gyro_roll_angle)) + ((
                                    1 - ALPHA) * acc_roll_angle);
    complimentary_angle->pitch = (ALPHA * (sens->previous_measurement.pitch + gyro_pitch_angle)) + ((
                                     1 - ALPHA) * acc_pitch_angle);

    sens->previous_measurement.roll = complimentary_angle->roll;
    sens->previous_measurement.pitch = complimentary_angle->pitch;

    return ESP_OK;
}

esp_err_t icm42670_read_register(icm42670_handle_t sensor, uint8_t reg, uint8_t *val)
{
    return icm42670_read(sensor, reg, val, 1);
}

esp_err_t icm42670_write_register(icm42670_handle_t sensor, uint8_t reg, uint8_t val)
{
    return icm42670_write(sensor, reg, &val, 1);
}

esp_err_t icm42670_read_mreg_register(icm42670_handle_t sensor, uint8_t mreg, uint8_t reg,
                                      uint8_t *val)
{
    uint8_t blk_sel_r = 0;
    if (mreg == 1) {
        blk_sel_r = 0;
    } else if (mreg == 2) {
        blk_sel_r = 0x28;
    } else if (mreg == 3) {
        blk_sel_r = 0x50;
    } else {
        ESP_LOGE(TAG, "Invalid MREG value %d", mreg);
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(icm42670_write(sensor, ICM42670_BLK_SEL_R, &blk_sel_r, 1), TAG,
                        "Failed to set BLK_SEL_R");
    ESP_RETURN_ON_ERROR(icm42670_write(sensor, ICM42670_MADDR_R, &reg, 1), TAG,
                        "Failed to set MADDR_R");

    esp_rom_delay_us(10);

    ESP_RETURN_ON_ERROR(icm42670_read(sensor, ICM42670_M_R, val, 1), TAG, "Failed to read M_R");

    esp_rom_delay_us(10);

    return ESP_OK;
}

esp_err_t icm42670_write_mreg_register(icm42670_handle_t sensor, uint8_t mreg, uint8_t reg,
                                       uint8_t val)
{
    uint8_t blk_sel_w = 0;
    if (mreg == 1) {
        blk_sel_w = 0;
    } else if (mreg == 2) {
        blk_sel_w = 0x28;
    } else if (mreg == 3) {
        blk_sel_w = 0x50;
    } else {
        ESP_LOGE(TAG, "Invalid MREG value %d", mreg);
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(icm42670_write(sensor, ICM42670_BLK_SEL_W, &blk_sel_w, 1), TAG,
                        "Failed to set BLK_SEL_W");
    ESP_RETURN_ON_ERROR(icm42670_write(sensor, ICM42670_MADDR_W, &reg, 1), TAG,
                        "Failed to set MADDR_W");
    ESP_RETURN_ON_ERROR(icm42670_write(sensor, ICM42670_M_W, &val, 1), TAG, "Failed to set M_W");

    esp_rom_delay_us(10);

    return ESP_OK;
}
