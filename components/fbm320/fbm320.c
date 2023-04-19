/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "driver/i2c.h"
#include "fbm320.h"

// FBM320 registers
#define FBM320_WHO_AM_I                0x6Bu
#define FBM320_CONFIG_REG              0xF4u
#define FBM320_RESULT_START            0xF6u
#define FBM320_CALIBRATION_DATA_START0 0xAAu // 18 bytes
#define FBM320_CALIBRATION_DATA_1      0xD0u // 1 byte
#define FBM320_CALIBRATION_DATA_2      0xF1u // 1 byte
#define FBM320_CALIBRATION_DATA_LEN    20u   // 20 bytes together

typedef struct {
    int32_t C0, C1, C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, C12, C13;
} fbm320_calibration_data_t;

typedef struct {
    i2c_port_t bus;
    uint16_t dev_addr;
    bool initialized;
    fbm320_calibration_data_t calibration_data;
} fbm320_dev_t;

static esp_err_t fbm320_write(fbm320_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    fbm320_dev_t *sens = (fbm320_dev_t *) sensor;
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

static esp_err_t fbm320_read(fbm320_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    fbm320_dev_t *sens = (fbm320_dev_t *) sensor;
    esp_err_t ret;
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

fbm320_handle_t fbm320_create(i2c_port_t port, const uint16_t dev_addr)
{
    fbm320_dev_t *sensor = (fbm320_dev_t *) calloc(1, sizeof(fbm320_dev_t));
    sensor->bus = port;
    sensor->dev_addr = dev_addr << 1;
    sensor->initialized = false;
    return (fbm320_handle_t) sensor;
}

void fbm320_delete(fbm320_handle_t sensor)
{
    fbm320_dev_t *sens = (fbm320_dev_t *) sensor;
    free(sens);
}

esp_err_t fbm320_get_deviceid(fbm320_handle_t sensor, uint8_t *const deviceid)
{
    return fbm320_read(sensor, FBM320_WHO_AM_I, deviceid, 1);
}

esp_err_t fbm320_init(fbm320_handle_t sensor)
{
    esp_err_t ret;
    uint16_t R[10];
    uint8_t rx_buffer[FBM320_CALIBRATION_DATA_LEN];
    fbm320_dev_t *sens = (fbm320_dev_t *) sensor;

    ret = fbm320_read(sensor, FBM320_CALIBRATION_DATA_START0, rx_buffer, 18);
    ret |= fbm320_read(sensor, FBM320_CALIBRATION_DATA_1, &rx_buffer[18], 1);
    ret |= fbm320_read(sensor, FBM320_CALIBRATION_DATA_2, &rx_buffer[19], 1);
    if (ESP_OK != ret) {
        return ret;
    }

    for (int i = 0; i < 10; i++) {
        R[i] = (rx_buffer[2 * i] << 8) | rx_buffer[2 * i + 1];
    }

    /* Coefficient reconstruction */
    sens->calibration_data.C0 = R[0] >> 4;
    sens->calibration_data.C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
    sens->calibration_data.C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
    sens->calibration_data.C3 = R[2] >> 3;
    sens->calibration_data.C5 = R[4] >> 1;
    sens->calibration_data.C6 = R[5] >> 3;
    sens->calibration_data.C8 = R[7] >> 3;
    sens->calibration_data.C9 = R[8] >> 2;
    sens->calibration_data.C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
    sens->calibration_data.C11 = R[9] & 0xFF;
    sens->calibration_data.C4 = ((uint32_t)R[3] << 2) | (R[0] & 3);
    sens->calibration_data.C7 = ((uint32_t)R[6] << 3) | (R[5] & 7);
    sens->calibration_data.C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7);

    // first dummy read
    uint8_t cmd = 0x2e;
    ret = fbm320_write(sensor, FBM320_CONFIG_REG, &cmd, 1);
    if (ESP_OK != ret) {
        return ret;
    }

    sens->initialized = true;

    return ret;
}

static esp_err_t fbm320_read_result(fbm320_handle_t sensor, int32_t *const result)
{
    uint8_t rx_buffer[3];
    esp_err_t ret = fbm320_read(sensor, FBM320_RESULT_START, rx_buffer, 3);
    *result = (rx_buffer[0] << 16) | (rx_buffer[1] << 8) | rx_buffer[2];
    return ret;
}

esp_err_t fbm320_get_data(fbm320_handle_t sensor, const fbm320_measure_mode_t meas_mode, int32_t *const temperature, int32_t *const pressure)
{
    esp_err_t ret;
    fbm320_dev_t *sens = (fbm320_dev_t *) sensor;
    if (!sens->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t temperature_raw, pressure_raw;

    // trigger TEMPERATURE measurement and wait for result
    uint8_t cmd = 0x2e;
    ret = fbm320_write(sensor, FBM320_CONFIG_REG, &cmd, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ret = fbm320_read_result(sensor, &temperature_raw);
    if (ESP_OK != ret) {
        return ret;
    }

    // trigger PRESSURE  measurement and wait for result
    ret = fbm320_write(sensor, FBM320_CONFIG_REG, (uint8_t *)&meas_mode, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    if (meas_mode == FBM320_MEAS_PRESS_OSR_8192) {
        vTaskDelay(20 / portTICK_PERIOD_MS);
    } else {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    ret = fbm320_read_result(sensor, &pressure_raw);
    if (ESP_OK != ret) {
        return ret;
    }

    int32_t X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32;
    int32_t PP1, PP2, PP3, PP4, CF, DT, DT2; // helper variables
    fbm320_calibration_data_t *cal_data = &sens->calibration_data; // helper pointer

    DT = ((temperature_raw - 8388608) >> 4) + (cal_data->C0 << 4);
    X01 = (cal_data->C1 + 4459) * DT >> 1;
    X02 = ((((cal_data->C2 - 256) * DT) >> 14) * DT) >> 4;
    X03 = (((((cal_data->C3 * DT) >> 18) * DT) >> 18) * DT);
    *temperature = ((2500 << 15) - X01 - X02 - X03) >> 15;

    DT2 = (X01 + X02 + X03) >> 12;
    X11 = ((cal_data->C5 - 4443) * DT2);
    X12 = (((cal_data->C6 * DT2) >> 16) * DT2) >> 2;
    X13 = ((X11 + X12) >> 10) + ((cal_data->C4 + 120586) << 4);

    X21 = ((cal_data->C8 + 7180) * DT2) >> 10;
    X22 = (((cal_data->C9 * DT2) >> 17) * DT2) >> 12;
    X23 = abs(X22 - X21);
    X24 = (X23 >> 11) * (cal_data->C7 + 166426);
    X25 = ((X23 & 0x7FF) * (cal_data->C7 + 166426)) >> 11;
    X26 = (X21 >= X22) ? (((0 - X24 - X25) >> 11) + cal_data->C7 + 166426) : (((X24 + X25) >> 11) + cal_data->C7 + 166426);

    PP1 = ((pressure_raw - 8388608) - X13) >> 3;
    PP2 = (X26 >> 11) * PP1;
    PP3 = ((X26 & 0x7FF) * PP1) >> 11;
    PP4 = (PP2 + PP3) >> 10;

    CF = (2097152 + cal_data->C12 * DT2) >> 3;
    X31 = (((CF * cal_data->C10) >> 17) * PP4) >> 2;
    X32 = (((((CF * cal_data->C11) >> 15) * PP4) >> 18) * PP4);
    *pressure = ((X31 + X32) >> 15) + PP4 + 99880;

    return ret;
}
