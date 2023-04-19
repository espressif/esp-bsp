/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief MAG3110 driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"

/**
* @brief Device Identification value
*/
#define MAG3110_WHO_AM_I_VAL 0xC4u

typedef void *mag3110_handle_t;
typedef struct {
    int16_t x, y, z;
} mag3110_result_t;

/**
 * @brief Data rate and oversampling settings
 *
 * Example:
 *  - MAG3110_DR_OS_80_16 means 80Hz output data rate and 16x oversampling
 *  - MAG3110_DR_OS_1_25_64 means 1.25Hz output data rate and 64x oversampling
 *
 * @see MAG3110 datasheet Table 32.
 */
typedef enum {
    MAG3110_DR_OS_80_16 = 0x00,
    MAG3110_DR_OS_40_32 = 0x08,
    MAG3110_DR_OS_20_64 = 0x10,
    MAG3110_DR_OS_10_128 = 0x18,
    MAG3110_DR_OS_40_16 = 0x20,
    MAG3110_DR_OS_20_32 = 0x28,
    MAG3110_DR_OS_10_64 = 0x30,
    MAG3110_DR_OS_5_128 = 0x38,
    MAG3110_DR_OS_20_16 = 0x40,
    MAG3110_DR_OS_10_32 = 0x48,
    MAG3110_DR_OS_5_64 = 0x50,
    MAG3110_DR_OS_2_5_128 = 0x58,
    MAG3110_DR_OS_10_16 = 0x60,
    MAG3110_DR_OS_5_32 = 0x68,
    MAG3110_DR_OS_2_5_64 = 0x70,
    MAG3110_DR_OS_1_25_128 = 0x78,
    MAG3110_DR_OS_5_16 = 0x80,
    MAG3110_DR_OS_2_5_32 = 0x88,
    MAG3110_DR_OS_1_25_64 = 0x90,
    MAG3110_DR_OS_0_63_128 = 0x98,
    MAG3110_DR_OS_2_5_16 = 0xA0,
    MAG3110_DR_OS_1_25_32 = 0xA8,
    MAG3110_DR_OS_0_63_64 = 0xB0,
    MAG3110_DR_OS_0_31_128 = 0xB8,
    MAG3110_DR_OS_1_25_16 = 0xC0,
    MAG3110_DR_OS_0_63_32 = 0xC8,
    MAG3110_DR_OS_0_31_64 = 0xD0,
    MAG3110_DR_OS_0_16_128 = 0xD8,
    MAG3110_DR_OS_0_63_16 = 0xE0,
    MAG3110_DR_OS_0_31_32 = 0xE8,
    MAG3110_DR_OS_0_16_64 = 0xF0,
    MAG3110_DR_OS_0_08_128 = 0xF8
} mag3110_data_rate_t;

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param[in] port I2C number
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
mag3110_handle_t mag3110_create(const i2c_port_t port);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of mag3110
 */
void mag3110_delete(mag3110_handle_t sensor);

/**
 * @brief Start mag3110 sensor to measure
 *
 * @param sensor object handle of mag3110
 * @param[in] data_rate Data rate and oversampling settings
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mag3110_start(mag3110_handle_t sensor, const mag3110_data_rate_t data_rate);

/**
 * @brief Start mag3110 sensor in raw mode
 *
 * This function will ignore user offset register values that are set in mag3110_calibrate() function,
 * thus it is not needed to calibrate the sensor to run it in the raw mode.
 *
 * @param sensor object handle of mag3110
 * @param[in] data_rate Data rate and oversampling settings
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mag3110_start_raw(mag3110_handle_t sensor, const mag3110_data_rate_t data_rate);

/**
 * @brief Stop MAG3110 measurement
 *
 * @param sensor object handle of mag3110
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mag3110_stop(mag3110_handle_t sensor);

/**
 * @brief Get device identification of MAG3110
 *
 * @param sensor object handle of MAG3110
 * @param[out] deviceid a pointer to device ID value
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t mag3110_get_deviceid(mag3110_handle_t sensor, uint8_t *const deviceid);

/**
 * @brief Read MAG3110 output register and return them in [0.1uT]
 *
 * @param sensor object handle of mag3110
 * @param[out] mag_induction Pointer where the results is going to be saved in units of 0.1uT
 * @return
 *     - ESP_OK Success
 *     - Else   Fail
 */
esp_err_t mag3110_get_magnetic_induction(mag3110_handle_t sensor, mag3110_result_t *const mag_induction);

/**
 * @brief Calibrate MAG3110 for a hard-iron offset
 *
 * This function will detect offsets in MAG3110 reading (caused by hard-iron or PCB influence) and store it in MAG3110 registers.
 *
 * During the calibration, the user must keep rotating the sensor in every axis to guarantee correct result.
 *
 * @param sensor MAG3110 handle
 * @param[in] cal_duration_ms Calibration duration in [ms]
 * @return
 *     - ESP_OK Success
 *     - Else   Fail
 */
esp_err_t mag3110_calibrate(mag3110_handle_t sensor, const uint32_t cal_duration_ms);

#ifdef __cplusplus
}
#endif
