/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * mpu60xx.h
 *
 *  Created on: 15-Oct-2024
 *      Author: Rohan Jeet <jeetrohan9@gmail.com>
 */

#ifndef __MPU_60XX_H
#define __MPU_60XX_H


#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "mpu60xx_types.h"
#include "mpu60xx_regs_bits.h"
#include "mpu60xx_interrupts.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief get float values of accelerometer, gyrometer and temperature sensor readings from MPU60xx
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[out] sensor_read structure to hold the values read
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log
 */
esp_err_t mpu60xx_read_sensor(mpu60xx_handle_t mpu_handle, mpu60xx_reading_t *sensor_read);

/**
 * @brief get raw values of accelerometer, gyrometer and temperature sensor readings from MPU60xx
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[out] sensor_raw_read structure to hold the raw values read
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log
 */
esp_err_t mpu60xx_read_sensor_raw (mpu60xx_handle_t mpu_handle, mpu60xx_reading_raw_t *sensor_raw_read);



/**
 * @brief initialize MPU60xx and also the handle used to interact with it.
 *
 * @param[in] config_handle initial configuration of MPU60xx
 *
 * @param[out] mpu_handle Handle of mpu60xx
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log
 */
esp_err_t mpu60xx_init(mpu60xx_init_config_t config_handle, mpu60xx_handle_t mpu_handle);



/**
* @brief create an MPU60xx device handle
*
* @param[in] bus_handle I2C master bus handle, with which this device will be associated
*
* @param[in] scl_speed_hz I2C communication speed used by this device
*
* @param[in] mpu60xx_address I2C address of this device
*
* @return
*      - NULL: Failed to create MPU60xx device handle
*      - other error codes : from the underlying i2c driver, check log
*
*/
mpu60xx_handle_t mpu60xx_create ( i2c_master_bus_handle_t bus_handle, uint8_t mpu60xx_address);


/**
* @brief free the resources associated with MPU60xx device
*
* @param[in] mpu60xx_handler address of MPU60xx device handle
*
* @return
*      - ESP_OK: successfully removed the handle resources
*      - ESP_ERR_INVALID_ARG: NULL value passed as handler
*      - other error codes : failed to remove, check log
*/
esp_err_t mpu60xx_remove (mpu60xx_handle_t *mpu60xx_handler);


#ifdef __cplusplus
}
#endif

#endif // __MPU_60XX_H
