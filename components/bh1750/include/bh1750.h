/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"

typedef enum {
    BH1750_CONTINUE_1LX_RES       = 0x10,   /*!< Command to set measure mode as Continuously H-Resolution mode*/
    BH1750_CONTINUE_HALFLX_RES    = 0x11,   /*!< Command to set measure mode as Continuously H-Resolution mode2*/
    BH1750_CONTINUE_4LX_RES       = 0x13,   /*!< Command to set measure mode as Continuously L-Resolution mode*/
    BH1750_ONETIME_1LX_RES        = 0x20,   /*!< Command to set measure mode as One Time H-Resolution mode*/
    BH1750_ONETIME_HALFLX_RES     = 0x21,   /*!< Command to set measure mode as One Time H-Resolution mode2*/
    BH1750_ONETIME_4LX_RES        = 0x23,   /*!< Command to set measure mode as One Time L-Resolution mode*/
} bh1750_measure_mode_t;

#define BH1750_I2C_ADDRESS_DEFAULT   (0x23)
typedef void *bh1750_handle_t;

/**
 * @brief Set bh1750 as power down mode (low current)
 *
 * @param sensor object handle of bh1750
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t bh1750_power_down(bh1750_handle_t sensor);

/**
 * @brief Set bh1750 as power on mode
 *
 * @param sensor object handle of bh1750
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t bh1750_power_on(bh1750_handle_t sensor);

/**
 * @brief Get light intensity from bh1750
 *
 * @param     sensor object handle of bh1750
 * @param[in] cmd_measure the instruction to set measurement mode
 *
 * @note
 *        You should call this funtion to set measurement mode before call bh1750_get_data() to acquire data.
 *        If you set onetime mode, you just can get one measurement result.
 *        If you set continuous mode, you can call bh1750_get_data() to acquire data repeatedly.
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t bh1750_set_measure_mode(bh1750_handle_t sensor, const bh1750_measure_mode_t cmd_measure);

/**
 * @brief Get light intensity from BH1750
 *
 * Returns light intensity in [lx] corrected by typical BH1750 Measurement Accuracy (= 1.2).
 *
 * @see BH1750 datasheet Rev. D page 2
 *
 * @note
 *        You should acquire data from the sensor after the measurement time is over,
 *        so take care of measurement time in different modes.
 *
 * @param      sensor object handle of bh1750
 * @param[out] data light intensity value got from bh1750 in [lx]
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t bh1750_get_data(bh1750_handle_t sensor, float *const data);

/**
 * @brief Set measurement time
 *
 * This function is used to adjust BH1750 sensitivity, i.e. compensating influence from optical window.
 *
 * @see BH1750 datasheet Rev. D page 11
 *
 * @param     sensor object handle of bh1750
 * @param[in] measure_time measurement time
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t bh1750_set_measure_time(bh1750_handle_t sensor, const uint8_t measure_time);

/**
 * @brief Create and init sensor object and return a sensor handle
 *
 * @param     port     I2C port number
 * @param[in] dev_addr I2C device address of sensor
 *
 * @return
 *     - NULL Fail
 *     - Others Success
 */
bh1750_handle_t bh1750_create(i2c_port_t port, const uint16_t dev_addr);

/**
 * @brief Delete and release a sensor object
 *
 * @param sensor object handle of bh1750
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t bh1750_delete(bh1750_handle_t sensor);

#ifdef __cplusplus
}
#endif
