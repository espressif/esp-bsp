/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"

#define AHT30_I2C_ADDRESS         0x38

typedef void *aht30_handle_t;

/**
 * @brief Create and init sensor object
 *
 * @param[in]  i2c_bus    I2C bus handle. Obtained from i2c_new_master_bus()
 * @param[in]  dev_addr   I2C device address of sensor.
 * @param[out] handle_ret Handle to created AHT30 driver object
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_NO_MEM Not enough memory for the driver
 *     - Others Error from underlying I2C driver
 */
esp_err_t aht30_create(i2c_master_bus_handle_t i2c_bus, const uint8_t dev_addr, aht30_handle_t *handle_ret);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of AHT30
 */
void aht30_delete(aht30_handle_t sensor);

/**
 * @brief Read temperature and humidity values
 *
 * @param sensor        object handle of AHT30
 * @param temperature   temperature measurement
 * @param humidity      humidity measurement
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t aht30_get_temperature_humidity_value(aht30_handle_t sensor, float *temperature, float *humidity);

/**
 * @brief Read busy status
 *
 * @param sensor    object handle of AHT30
 * @param busy      busy status
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t aht30_get_busy(aht30_handle_t sensor, bool *busy);

#ifdef __cplusplus
}
#endif
