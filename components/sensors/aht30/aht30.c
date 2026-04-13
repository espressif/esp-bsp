/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "esp_check.h"
#include "iot_sensor_hub.h"
#include "aht30.h"

#define I2C_CLK_SPEED 400000

/*******************************************************************************
* Types definitions
*******************************************************************************/

typedef struct {
    i2c_master_dev_handle_t i2c_handle;
} aht30_dev_t;

/*******************************************************************************
* Function definitions
*******************************************************************************/
static esp_err_t aht30_write(aht30_handle_t sensor, const uint8_t *data_buf, const uint8_t data_len);
static esp_err_t aht30_receive(const aht30_handle_t sensor, uint8_t *data_buf, const uint8_t data_len);
static uint8_t aht30_calc_crc(uint8_t *data, uint8_t length);

/*******************************************************************************
* Local variables
*******************************************************************************/
static const char *TAG = "AHT30";

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t aht30_create(i2c_master_bus_handle_t i2c_bus, const uint8_t dev_addr, aht30_handle_t *handle_ret)
{
    ESP_RETURN_ON_FALSE(i2c_bus != NULL, ESP_ERR_INVALID_ARG, TAG, "I2C handle can't be NULL");
    ESP_RETURN_ON_FALSE(handle_ret != NULL, ESP_ERR_INVALID_ARG, TAG, "AHT30 handle can't be NULL");

    esp_err_t ret = ESP_OK;

    // Allocate memory and initialize the driver object
    aht30_dev_t *sensor = (aht30_dev_t *) calloc(1, sizeof(aht30_dev_t));
    ESP_RETURN_ON_FALSE(sensor != NULL, ESP_ERR_NO_MEM, TAG, "Not enough memory");

    const i2c_device_config_t i2c_dev_cfg = {
        .device_address = dev_addr,
        .scl_speed_hz = I2C_CLK_SPEED,
    };

    ret = i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &sensor->i2c_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add a new I2C device");
        aht30_delete(sensor);
        return ret;
    }
    assert(sensor->i2c_handle);

    ESP_LOGD(TAG, "Successfully created a AHT30 device");
    *handle_ret = sensor;
    return ret;
}

void aht30_delete(aht30_handle_t sensor)
{
    aht30_dev_t *sens = (aht30_dev_t *) sensor;

    if (sens->i2c_handle) {
        i2c_master_bus_rm_device(sens->i2c_handle);
    }

    free(sens);
}

esp_err_t aht30_get_temperature_humidity_value(aht30_handle_t sensor, float *temperature, float *humidity)
{
    ESP_RETURN_ON_FALSE(sensor != NULL, ESP_ERR_INVALID_ARG, TAG, "AHT30 handle can't be NULL");
    ESP_RETURN_ON_FALSE(temperature != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the busy variable can't be NULL");
    ESP_RETURN_ON_FALSE(humidity != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the busy variable can't be NULL");

    esp_err_t ret = ESP_OK;

    // Start measurement command specified by the manufacturer
    uint8_t start_measurement[] = {0xAC, 0x33, 0x00};

    ret = aht30_write(sensor, start_measurement, sizeof(start_measurement));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start AHT30 measurement");
        return ret;
    }

    bool busy;
    uint8_t max_tries = 18;
    // Wait for the measurement to finish
    do {
        ret = aht30_get_busy(sensor, &busy);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read AHT30 busy status");
            return ret;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    } while (--max_tries && busy);

    if (busy) {
        ESP_LOGE(TAG, "AHT30 timeout of measurement");
        return ESP_FAIL;
    }

    uint8_t crc_result;
    uint8_t receive_buf[7];
    ESP_RETURN_ON_ERROR(aht30_receive(sensor, receive_buf, sizeof(receive_buf)), TAG, "Failed to receive measurement data");

    crc_result = aht30_calc_crc(receive_buf, sizeof(receive_buf) - 1);
    ESP_RETURN_ON_FALSE(crc_result == receive_buf[sizeof(receive_buf) - 1], ESP_FAIL, TAG,
                        "Result of CRC calculation does not match");

    uint32_t raw_value;
    raw_value = (receive_buf[3] & 0x0F) << 16 | receive_buf[4] << 8 | receive_buf[5];
    *temperature = (float)raw_value * 200 / (1 << 20) - 50;

    raw_value = receive_buf[1] << 12 | receive_buf[2] << 4 | receive_buf[3] >> 4;
    *humidity = (float)raw_value * 100 / (1 << 20) ;

    return ret;
}

esp_err_t aht30_get_busy(aht30_handle_t sensor, bool *busy)
{
    ESP_RETURN_ON_FALSE(sensor != NULL, ESP_ERR_INVALID_ARG, TAG, "AHT30 handle can't be NULL");
    ESP_RETURN_ON_FALSE(busy != NULL, ESP_ERR_INVALID_ARG, TAG, "Pointer to the busy variable can't be NULL");
    uint8_t data = 0;

    ESP_RETURN_ON_ERROR(aht30_receive(sensor, &data, 1), TAG, "Read sensor busy failed");

    *busy = (data & BIT7);
    return ESP_OK;
}

/*******************************************************************************
* Private functions
*******************************************************************************/
static esp_err_t aht30_write(const aht30_handle_t sensor, const uint8_t *data_buf, const uint8_t data_len)
{
    aht30_dev_t *sens = (aht30_dev_t *) sensor;
    assert(sens);

    return i2c_master_transmit(sens->i2c_handle, data_buf, data_len, -1);
}

static esp_err_t aht30_receive(const aht30_handle_t sensor, uint8_t *data_buf, const uint8_t data_len)
{
    aht30_dev_t *sens = (aht30_dev_t *) sensor;
    assert(sens);

    return i2c_master_receive(sens->i2c_handle, data_buf, data_len, -1);
}

static uint8_t aht30_calc_crc(uint8_t *data, uint8_t length)
{
    uint8_t i;
    uint8_t byte;
    uint8_t crc = 0xFF;
    for (byte = 0; byte < length; byte++) {
        crc ^= (data[byte]);
        for (i = 8; i > 0; --i) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/*******************************************************************************
* Private functions for sensor hub implementation
*******************************************************************************/

static aht30_handle_t sensor_hub_aht30_handle;

esp_err_t aht30_impl_init(bus_handle_t bus_handle, uint8_t addr)
{
    esp_err_t ret;

    ret = aht30_create(bus_handle, addr, &sensor_hub_aht30_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create AHT30 with error %s", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t aht30_impl_deinit(void)
{
    aht30_delete(sensor_hub_aht30_handle);
    sensor_hub_aht30_handle = NULL;
    return ESP_OK;
}

esp_err_t aht30_impl_test(void)
{
    bool busy;
    return aht30_get_busy(sensor_hub_aht30_handle, &busy);
}

esp_err_t aht30_impl_acquire_humidity(float *humidity)
{
    float temperature;
    return aht30_get_temperature_humidity_value(sensor_hub_aht30_handle, &temperature, humidity);
}

esp_err_t aht30_impl_acquire_temperature(float *temperature)
{
    float humidity;
    return aht30_get_temperature_humidity_value(sensor_hub_aht30_handle, temperature, &humidity);
}

static humiture_impl_t aht30_impl = {
    .init = aht30_impl_init,
    .deinit = aht30_impl_deinit,
    .test = aht30_impl_test,
    .acquire_humidity = aht30_impl_acquire_humidity,
    .acquire_temperature = aht30_impl_acquire_temperature,
};

SENSOR_HUB_DETECT_FN(HUMITURE_ID, sensor_hub_aht30, &aht30_impl);
