/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "hts221.h"
#include "hts221_reg.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define HTS221_I2C_ADDRESS    ((uint8_t)0x5F) // HTS221 constant address

typedef struct {
    i2c_port_t bus;
    uint16_t dev_addr;
    bool initialized;

    // Data-ready (DRDY) related variables
    TaskHandle_t drdy_task_handle;
    gpio_num_t drdy_pin;
    hts221_drdylevel_t drdy_level;
    hts221_drdy_callback_t drdy_callback;

    // calibration coefficients
    struct {
        // temperature calibration
        int16_t t0_out;
        int16_t t1_out;
        int16_t t0_degc;
        int16_t t1_degc;

        // humidity calibration
        int16_t h0_rh;
        int16_t h1_rh;
        int16_t h0_t0_out;
        int16_t h1_t0_out;
    } calibration_data;
} hts221_dev_t;

static void IRAM_ATTR drdy_isr(void *args)
{
    hts221_dev_t *sens = (hts221_dev_t *)args;
    BaseType_t xYieldRequired;

    vTaskNotifyGiveFromISR(sens->drdy_task_handle, &xYieldRequired);
    if (xYieldRequired) {
        portYIELD_FROM_ISR();
    }
}

static void drdy_task(void *args)
{
    hts221_dev_t *sens = (hts221_dev_t *)args;

    const gpio_config_t drdy_pin_config = {
        .intr_type = sens->drdy_level ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = BIT64(sens->drdy_pin),
        .pull_down_en = false,
        .pull_up_en = false
    };

    ESP_ERROR_CHECK(gpio_isr_handler_add(sens->drdy_pin, drdy_isr, args));
    ESP_ERROR_CHECK(gpio_config(&drdy_pin_config));

    // perform dummy read to reset DRDY status
    int16_t temperature, humidity;
    hts221_get_humidity(args, &humidity);
    hts221_get_temperature(args, &temperature);

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for new data from ISR
        hts221_get_humidity(args, &humidity);
        hts221_get_temperature(args, &temperature);
        if (sens->drdy_callback != NULL) {
            sens->drdy_callback(humidity, temperature);
        }
    }
    vTaskDelete(NULL);
}

static esp_err_t hts221_write(hts221_handle_t sensor, const uint8_t reg_start_addr, const uint8_t *const data_buf, const uint8_t data_len)
{
    hts221_dev_t *sens = (hts221_dev_t *) sensor;
    esp_err_t  ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr | 0x80, true); // enable sequential write
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data_buf, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(sens->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static inline esp_err_t hts221_write_byte(hts221_handle_t sensor, uint8_t const reg_addr, const uint8_t data)
{
    return hts221_write(sensor, reg_addr, &data, 1);
}

static esp_err_t hts221_read(hts221_handle_t sensor, const uint8_t reg_start_addr, uint8_t *const data_buf, const uint8_t data_len)
{
    hts221_dev_t *sens = (hts221_dev_t *) sensor;
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, sens->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_start_addr | 0x80, true); // enable sequential write
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

static inline esp_err_t hts221_read_byte(hts221_handle_t sensor, const uint8_t reg, uint8_t *const data)
{
    return hts221_read(sensor, reg, data, 1);
}

esp_err_t hts221_get_deviceid(hts221_handle_t sensor, uint8_t *const deviceid)
{
    esp_err_t ret = hts221_read_byte(sensor, HTS221_WHO_AM_I_REG, deviceid);
    return ret;
}

esp_err_t hts221_set_config(hts221_handle_t sensor, const hts221_config_t *const hts221_config)
{
    esp_err_t ret;

    // AV_CONF
    uint8_t tmp = (uint8_t)hts221_config->avg_h | (uint8_t)hts221_config->avg_t;
    ret = hts221_write_byte(sensor, HTS221_AV_CONF_REG, tmp);
    if (ESP_OK != ret) {
        return ret;
    }

    // CTRL_REG1
    ret = hts221_read_byte(sensor, HTS221_CTRL_REG1, &tmp);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp &= ~(HTS221_BDU_MASK | HTS221_ODR_MASK);
    tmp |= (uint8_t)hts221_config->odr;
    tmp |= ((uint8_t)hts221_config->bdu_status) << HTS221_BDU_BIT;
    ret = hts221_write_byte(sensor, HTS221_CTRL_REG1, tmp);
    if (ESP_OK != ret) {
        return ret;
    }

    return ret;
}

esp_err_t hts221_init(hts221_handle_t sensor, const hts221_config_t *const hts221_config)
{
    esp_err_t ret;
    uint8_t who_am_i;
    hts221_dev_t *dev = (hts221_dev_t *)sensor;
    uint8_t cal_data_buffer[HTS221_CALIBRATION_DATA_SIZE];

    // check if device is responding
    ret = hts221_get_deviceid(sensor, &who_am_i);
    if (ESP_OK != ret) {
        return ret;
    } else if (HTS221_WHO_AM_I_VAL != who_am_i) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    // load calibration data to RAM
    ret = hts221_read(sensor, HTS221_CALIBRATION_DATA_START, cal_data_buffer, HTS221_CALIBRATION_DATA_SIZE);
    assert(ESP_OK == ret);

    // temperature constants
    uint16_t temp;
    temp = (((uint16_t)(cal_data_buffer[5] & 0x03)) << 8) | ((uint16_t)cal_data_buffer[2]);
    dev->calibration_data.t0_degc = temp >> 3;
    temp = (((uint16_t)(cal_data_buffer[5] & 0x0C)) << 6) | ((uint16_t)cal_data_buffer[3]);
    dev->calibration_data.t1_degc = temp >> 3;
    dev->calibration_data.t0_out = (((uint16_t)cal_data_buffer[13]) << 8) | (uint16_t)cal_data_buffer[12];
    dev->calibration_data.t1_out = (((uint16_t)cal_data_buffer[15]) << 8) | (uint16_t)cal_data_buffer[14];

    if ((dev->calibration_data.t1_out - dev->calibration_data.t0_out) == 0) {
        return ESP_FAIL;
    }

    // humidity constants
    dev->calibration_data.h0_rh = cal_data_buffer[0] >> 1;
    dev->calibration_data.h1_rh = cal_data_buffer[1] >> 1;
    dev->calibration_data.h0_t0_out = (int16_t)(((uint16_t)cal_data_buffer[7]) << 8) | (uint16_t)cal_data_buffer[6];
    dev->calibration_data.h1_t0_out = (int16_t)(((uint16_t)cal_data_buffer[11]) << 8) | (uint16_t)cal_data_buffer[10];
    if (dev->calibration_data.h1_t0_out - dev->calibration_data.h0_t0_out == 0) {
        return ESP_FAIL;
    }

    // configure and activate it
    ret = hts221_set_config(sensor, hts221_config);
    assert(ESP_OK == ret);
    ret = hts221_set_activate(sensor);
    assert(ESP_OK == ret);
    dev->initialized = true;

    return ret;
}

esp_err_t hts221_get_config(hts221_handle_t sensor, hts221_config_t *const hts221_config)
{
    uint8_t tmp;
    esp_err_t ret;

    ret = hts221_read(sensor, HTS221_AV_CONF_REG, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    hts221_config->avg_h = (hts221_avgh_t)(tmp & HTS221_AVGH_MASK);
    hts221_config->avg_t = (hts221_avgt_t)(tmp & HTS221_AVGT_MASK);

    ret = hts221_read(sensor, HTS221_CTRL_REG1, &tmp, 1);
    if (ESP_OK != ret) {
        return ret;
    }
    hts221_config->odr = (hts221_odr_t)(tmp & HTS221_ODR_MASK);
    hts221_config->bdu_status = (bool)((tmp & HTS221_BDU_MASK) >> HTS221_BDU_BIT);
    return ESP_OK;
}

static esp_err_t hts221_set_reg_field(hts221_handle_t sensor, const uint8_t reg_addr, const uint8_t mask, const uint8_t shift, const uint8_t val)
{
    uint8_t tmp;

    esp_err_t ret = hts221_read_byte(sensor, reg_addr, &tmp);
    if (ESP_OK != ret) {
        return ret;
    }
    tmp &= ~mask;
    tmp |= ((uint8_t)val) << shift;
    ret = hts221_write_byte(sensor, reg_addr, tmp);
    return ret;
}

esp_err_t hts221_set_activate(hts221_handle_t sensor)
{
    return hts221_set_reg_field(sensor, HTS221_CTRL_REG1, HTS221_PD_MASK, HTS221_PD_BIT, 1);
}

esp_err_t hts221_set_powerdown(hts221_handle_t sensor)
{
    return hts221_set_reg_field(sensor, HTS221_CTRL_REG1, HTS221_PD_MASK, HTS221_PD_BIT, 0);
}

esp_err_t hts221_set_odr(hts221_handle_t sensor, const hts221_odr_t odr)
{
    return hts221_set_reg_field(sensor, HTS221_CTRL_REG1, HTS221_ODR_MASK, 0, (uint8_t)odr);
}

esp_err_t hts221_set_avgh(hts221_handle_t sensor, const hts221_avgh_t avgh)
{
    return hts221_set_reg_field(sensor, HTS221_AV_CONF_REG, HTS221_AVGH_MASK, 0, (uint8_t)avgh);
}

esp_err_t hts221_set_avgt(hts221_handle_t sensor, const hts221_avgt_t avgt)
{
    return hts221_set_reg_field(sensor, HTS221_AV_CONF_REG, HTS221_AVGT_MASK, 0, (uint8_t)avgt);
}

esp_err_t hts221_set_bdumode(hts221_handle_t sensor, const bool status)
{
    return hts221_set_reg_field(sensor, HTS221_CTRL_REG1, HTS221_BDU_MASK, HTS221_BDU_BIT, status ? 1 : 0);
}

esp_err_t hts221_set_heaterstate(hts221_handle_t sensor, const bool status)
{
    return hts221_set_reg_field(sensor, HTS221_CTRL_REG2, HTS221_HEATER_MASK, HTS221_HEATER_BIT, status ? 1 : 0);
}

esp_err_t hts221_start_oneshot(hts221_handle_t sensor)
{
    return hts221_set_reg_field(sensor, HTS221_CTRL_REG2, HTS221_ONE_SHOT_MASK, 0, 1);
}

esp_err_t hts221_get_humidity(hts221_handle_t sensor, int16_t *const humidity)
{
    esp_err_t ret;
    hts221_dev_t *dev = (hts221_dev_t *)sensor;
    uint8_t buffer[2];

    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = hts221_read(sensor, HTS221_HR_OUT_L_REG, buffer, 2);
    assert(ESP_OK == ret);
    int16_t h_out = (int16_t)(((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

    int32_t tmp_32 = ((int32_t)(h_out - dev->calibration_data.h0_t0_out)) * ((int32_t)(dev->calibration_data.h1_rh - dev->calibration_data.h0_rh) * 10);
    *humidity = tmp_32 / (int32_t)(dev->calibration_data.h1_t0_out - dev->calibration_data.h0_t0_out)  + dev->calibration_data.h0_rh * 10;
    if (*humidity > 1000) {
        *humidity = 1000;
    }
    return ret;
}

esp_err_t hts221_get_temperature(hts221_handle_t sensor, int16_t *const temperature)
{
    esp_err_t ret;
    hts221_dev_t *dev = (hts221_dev_t *)sensor;
    uint8_t buffer[2];

    if (!dev->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ret = hts221_read(sensor, HTS221_TEMP_OUT_L_REG, buffer, 2);
    assert(ESP_OK == ret);
    int16_t t_out = (((uint16_t)buffer[1]) << 8) | (uint16_t)buffer[0];

    int32_t tmp_32 = ((int32_t)(t_out - dev->calibration_data.t0_out)) * ((int32_t)(dev->calibration_data.t1_degc - dev->calibration_data.t0_degc) * 10);
    *temperature = tmp_32 / (int32_t)(dev->calibration_data.t1_out - dev->calibration_data.t0_out) + dev->calibration_data.t0_degc * 10;
    return ret;
}

esp_err_t hts221_drdy_enable(hts221_handle_t sensor, const hts221_drdy_config_t *const config)
{
    esp_err_t ret;
    hts221_dev_t *sens = (hts221_dev_t *)sensor;

    if (config->drdy_callback == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!sens->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    // store parameters that are needed in a new FreeRTOS task
    sens->drdy_pin      = config->drdy_pin;
    sens->drdy_callback = config->drdy_callback;
    sens->drdy_level    = config->irq_level;

    // Create FreeRTOS task - interrupt allocation should be done in pinned to core task
    BaseType_t freertos_ret = xTaskCreatePinnedToCore(
                                  drdy_task, "HTS221 DRDY", 2048, sensor, config->drdy_task_priority, &sens->drdy_task_handle, 0
                              );
    if (pdPASS != freertos_ret) {
        return ESP_ERR_NO_MEM;
    }

    // CTRL_REG3
    uint8_t tmp = ((uint8_t)config->irq_level);
    tmp |= (uint8_t)config->irq_output_type;
    tmp |= 1 << HTS221_DRDY_BIT;
    ret = hts221_write_byte(sensor, HTS221_CTRL_REG3, tmp);
    if (ESP_OK != ret) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t hts221_drdy_disable(hts221_handle_t sensor)
{
    hts221_dev_t *sens = (hts221_dev_t *)sensor;

    // Delete FreeRTOS task and disable GPIO interrupt
    if (sens->drdy_task_handle != NULL) {
        gpio_isr_handler_remove(sens->drdy_pin);
        vTaskDelete(sens->drdy_task_handle);
        sens->drdy_task_handle = NULL;
        return hts221_write_byte(sensor, HTS221_CTRL_REG3, 0); // Disable the DRDY pin
    } else {
        return ESP_ERR_INVALID_STATE;
    }
}

hts221_handle_t hts221_create(const i2c_port_t port)
{
    hts221_dev_t *sensor = (hts221_dev_t *) calloc(1, sizeof(hts221_dev_t));
    sensor->bus = port;
    sensor->dev_addr = HTS221_I2C_ADDRESS  << 1;
    sensor->initialized = false;
    sensor->drdy_task_handle = NULL;
    return (hts221_handle_t) sensor;
}

void hts221_delete(hts221_handle_t sensor)
{
    hts221_dev_t *sens = (hts221_dev_t *) sensor;

    // disable DRDY interrupt if it was enabled
    if (sens->drdy_task_handle != NULL) {
        hts221_drdy_disable(sensor);
    }
    free(sens);
}
