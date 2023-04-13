/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mag3110.h"
#include "esp_log.h"
#include "esp_timer.h" // for calibration function
#include "esp_idf_version.h" // for backward compatibility of esp-timer

#define MAG3110_I2C_ADDRESS 0x0Eu // MAG3110 constant address
#define MAG3110_OUT_X_MSB   0x01u
#define MAG3110_WHO_AM_I    0x07u
#define MAG3110_OFF_X_MSB   0x09u
#define MAG3110_CTRL_REG1   0x10u

// ctrl reg1
#define MAG3110_ACTIVE_MODE  0x01u
#define MAG3110_STANDBY_MODE 0x00u

// ctrl reg2
#define MAG3110_AUTO_MRST_EN 0x80u
#define MAG3110_RAW_DATA     0x20u

typedef struct {
    i2c_port_t bus;
    uint16_t dev_addr;

    // calibration data
    int16_t max[3];
    int16_t min[3];
} mag3110_dev_t;

static esp_err_t mag3110_write(mag3110_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    mag3110_dev_t *sens = (mag3110_dev_t *) sensor;
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

static esp_err_t mag3110_read(mag3110_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    mag3110_dev_t *sens = (mag3110_dev_t *) sensor;
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

mag3110_handle_t mag3110_create(const i2c_port_t port)
{
    mag3110_dev_t *sensor = (mag3110_dev_t *) calloc(1, sizeof(mag3110_dev_t));
    sensor->bus = port;
    sensor->dev_addr = MAG3110_I2C_ADDRESS << 1;
    return (mag3110_handle_t) sensor;
}

void mag3110_delete(mag3110_handle_t sensor)
{
    mag3110_dev_t *sens = (mag3110_dev_t *) sensor;
    free(sens);
}

esp_err_t mag3110_start(mag3110_handle_t sensor, const mag3110_data_rate_t data_rate)
{
    uint8_t ctrl_reg[2];

    ctrl_reg[0] = data_rate | MAG3110_ACTIVE_MODE;
    ctrl_reg[1] = MAG3110_AUTO_MRST_EN;

    return mag3110_write(sensor, MAG3110_CTRL_REG1, ctrl_reg, sizeof(ctrl_reg));
}

esp_err_t mag3110_start_raw(mag3110_handle_t sensor, const mag3110_data_rate_t data_rate)
{
    uint8_t ctrl_reg[2];

    ctrl_reg[0] = data_rate | MAG3110_ACTIVE_MODE;
    ctrl_reg[1] = MAG3110_AUTO_MRST_EN | MAG3110_RAW_DATA;

    return mag3110_write(sensor, MAG3110_CTRL_REG1, ctrl_reg, sizeof(ctrl_reg));
}

esp_err_t mag3110_stop(mag3110_handle_t sensor)
{
    const uint8_t ctrl_reg1 = MAG3110_STANDBY_MODE;
    return mag3110_write(sensor, MAG3110_CTRL_REG1, &ctrl_reg1, 1);
}

esp_err_t mag3110_get_deviceid(mag3110_handle_t sensor, uint8_t *const deviceid)
{
    return mag3110_read(sensor, MAG3110_WHO_AM_I, deviceid, 1);
}

esp_err_t mag3110_get_magnetic_induction(mag3110_handle_t sensor, mag3110_result_t *const mag_induction)
{
    esp_err_t ret;
    uint8_t result_buffer[6];
    ret = mag3110_read(sensor, MAG3110_OUT_X_MSB, result_buffer, sizeof(result_buffer));
    if (ESP_OK != ret) {
        return ret;
    }

    mag_induction->x = ((uint16_t)result_buffer[0] << 8) | (uint16_t)result_buffer[1];
    mag_induction->y = ((uint16_t)result_buffer[2] << 8) | (uint16_t)result_buffer[3];
    mag_induction->z = ((uint16_t)result_buffer[4] << 8) | (uint16_t)result_buffer[5];

    return ret;
}

static void mag3110_timer_callback(void *arg)
{
    mag3110_dev_t *sens = (mag3110_dev_t *) arg;
    mag3110_result_t mag_data; // raw data from the magnetometer
    int16_t *axis_data = &mag_data.x; // we will iterate through struct members, so pointer to it is useful

    mag3110_get_magnetic_induction((mag3110_handle_t)arg, &mag_data);

    // compare the new data to stored min/max values
    for (int i = 0; i < 3; i++, axis_data++) {
        if (*axis_data > sens->max[i]) {
            sens->max[i] = *axis_data;
        }
        if (*axis_data < sens->min[i]) {
            sens->min[i] = *axis_data;
        }
    }
}

esp_err_t mag3110_calibrate(mag3110_handle_t sensor, const uint32_t cal_duration_ms)
{
    esp_err_t ret;
    uint8_t ctrl_reg[2];
    mag3110_dev_t *sens = (mag3110_dev_t *) sensor;

    if (NULL == sensor) {
        return ESP_ERR_INVALID_ARG;
    }

    // initialize calibration values to their starting point
    for (int i = 0; i < 3; i++) {
        sens->max[i] = INT16_MIN;
        sens->min[i] = INT16_MAX;
    }

    // reset MAG3110 to factory default
    ctrl_reg[0] = MAG3110_STANDBY_MODE;
    ctrl_reg[1] = 0;
    ret = mag3110_write(sensor, MAG3110_CTRL_REG1, ctrl_reg, sizeof(ctrl_reg));
    assert(ESP_OK == ret);

    // setup sensor for calibration mode
    ctrl_reg[0] = MAG3110_DR_OS_80_16 | MAG3110_ACTIVE_MODE; // fastest data-rate
    ctrl_reg[1] = MAG3110_AUTO_MRST_EN | MAG3110_RAW_DATA; // raw-data
    ret = mag3110_write(sensor, MAG3110_CTRL_REG1, ctrl_reg, sizeof(ctrl_reg));
    assert(ESP_OK == ret);

    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for start-up

    const esp_timer_create_args_t cal_timer_config = {
        .callback = mag3110_timer_callback,
        .arg = sensor,
        .name = "MAG3110 calibration timer",
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
        .skip_unhandled_events = true,
#endif
        .dispatch_method = ESP_TIMER_TASK
    };
    esp_timer_handle_t cal_timer = NULL;
    ret = esp_timer_create(&cal_timer_config, &cal_timer);
    assert(ESP_OK == ret);
    ESP_LOGI("MAG3110", "Entering calibration loop");
    ret = esp_timer_start_periodic(cal_timer, 12500); // 12.5ms
    assert(ESP_OK == ret);

    // Let the ESP_Timer collect the data
    // User must rotate the MAG3110 in all axis during this time
    vTaskDelay(cal_duration_ms / portTICK_PERIOD_MS);

    ESP_LOGI("MAG3110", "Exiting calibration loop");
    ret = esp_timer_stop(cal_timer);
    assert(ESP_OK == ret);
    ret = esp_timer_delete(cal_timer);
    assert(ESP_OK == ret);

    // reset MAG3110 to default state
    ctrl_reg[0] = MAG3110_STANDBY_MODE;
    ctrl_reg[1] = 0;
    ret = mag3110_write(sensor, MAG3110_CTRL_REG1, ctrl_reg, sizeof(ctrl_reg));
    assert(ESP_OK == ret);

    // calculate the offsets and load it to MAG3110
    int16_t offset[3]; // result of the calibration
    uint8_t offset_data[6]; // byte stream to MAG3110

    for (int i = 0; i < 3; i++) {
        offset[i] = (sens->max[i] + sens->min[i]) / 2;

        // offset register is 15 bit wide; see datasheet Chapter 5.3.1
        offset_data[2 * i] = ((uint16_t)offset[i] & 0xFF00) >> 7;
        offset_data[2 * i + 1] = ((uint16_t)offset[i] & 0x00FF) << 1;
    }
    ret = mag3110_write(sensor, MAG3110_OFF_X_MSB, offset_data, sizeof(offset_data));
    assert(ESP_OK == ret);

    ESP_LOGD("MAG3110", "offset data %i %i %i", offset[0], offset[1], offset[2]);

    vTaskDelay(100 / portTICK_PERIOD_MS); // Wait for shutdown

    return ret;
}
