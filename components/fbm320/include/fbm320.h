/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief FBM320 driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"

/**
 * @brief FMB320 configurable I2C address
 */
#define FBM320_I2C_ADDRESS_0   0x6Cu // ADDR pin low
#define FBM320_I2C_ADDRESS_1   0x6Du // ADDR pin high

/**
* @brief Device Identification value
*/
#define FBM320_WHO_AM_I_VAL 0x42u

typedef enum {
    FBM320_MEAS_PRESS_OSR_1024  = 0x34, /* 2.5ms wait for measurement */
    FBM320_MEAS_PRESS_OSR_2048  = 0x74, /* 3.7ms wait for measurement */
    FBM320_MEAS_PRESS_OSR_4096  = 0xB4, /* 6ms wait for measurement */
    FBM320_MEAS_PRESS_OSR_8192  = 0xF4  /* 11ms wait for measurement */
} fbm320_measure_mode_t;

typedef void *fbm320_handle_t;

/**
 * @brief Create sensor object and return a sensor handle
 *
 * @param     port     I2C port number
 * @param[in] dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
fbm320_handle_t fbm320_create(i2c_port_t port, const uint16_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of fbm320
 */
void fbm320_delete(fbm320_handle_t sensor);

/**
 * @brief Get device identification of FBM320
 *
 * @param sensor object handle of FBM320
 * @param[out] deviceid a pointer to device ID value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t fbm320_get_deviceid(fbm320_handle_t sensor, uint8_t *const deviceid);

/**
 * @brief Init barometer FBM320
 *
 * This function will only load calibration data from ROM of FBM320
 *
 * @param sensor object handle of FBM320
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t fbm320_init(fbm320_handle_t sensor);

/**
 * @brief Get pressure and temperature from FBM320
 *
 * This function triggers and reads out raw measurements of temperature and pressure.
 * Then it calculates real pressure and temperature based on the calibration constants (stored in sensor's ROM).
 *
 * @param sensor
 * @param[in] meas_mode Oversampling ratio of pressure measurement
 * @param[out] temperature Measured temperature in 0.01[deg C]
 * @param[out] pressure Measured pressure in [Pa]
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t fbm320_get_data(fbm320_handle_t sensor, const fbm320_measure_mode_t meas_mode, int32_t *const temperature, int32_t *const pressure);

#ifdef __cplusplus
}
#endif
