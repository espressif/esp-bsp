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


#ifdef __cplusplus
extern "C" {
#endif



/**
 * @brief Set accelerometer resolution
 * @note called by "mpu60xx_init",but provided for further flexibility
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[in] range Accelerometer range options
 *
 * @return
 *      - ESP_OK: accelerometer resolution successfully set
 *      - other error codes : from the underlying i2c driver, check log
 */
esp_err_t mpu60xx_setAccelerometerRange(mpu60xx_handle_t mpu_handle, mpu60xx_accel_range_t range);



/**
 * @brief Set gyrometer resolution
 * @note called by "mpu60xx_init",but provided for further flexibility
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[in] range Gyrometer range options
 *
 * @return
 *      - ESP_OK: gyrometer resolution successfully set
 *      - other error codes : from the underlying i2c driver, check log
 *
 *
 */
esp_err_t mpu60xx_setGyrometerRange(mpu60xx_handle_t mpu_handle, mpu60xx_gyro_range_t range);



/**
 * @brief Set sampling rate
 * @note called by "mpu60xx_init",but provided for further flexibility
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[in] sample_rate Sampling rate of mpu60xx
 *
 * @param[in] dlpf_bw Low pass frequency
 * @note using value other than MPU60XX_BAND_260_HZ (default), puts an upper limit of 1KHz on 'sample_rate',
 * @note the default behavior allows 'sample_rate' upper limit of
 *       gyrometer as 8KHz and accelerometer readings as 1KHz.
 *       Corresponding accelerometer readings are repeated for samples, if 'sample_rate'>1KHz.
 *
 * @return
 *      - ESP_OK: sampling rate successfully set
 *      - other error codes : from the underlying i2c driver, check log
 */
esp_err_t mpu60xx_setSampleRate ( mpu60xx_handle_t mpu_handle, uint32_t sample_rate, mpu60xx_lowpass_t dlpf_bw);



/**
 * @brief enable event detection
 *
 * @param[in] mpu_handle MPU60xx device handle.
 *
 * @param[in] mdc Configuration for event detection.
 *
 * @param[in] interrupt_configuration configuration for event detection interrupt behavior.
 * @note  can be NULL, use "mpu60xx_en_EventDetection" for polling.
 *
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register.
 *      - other: failed to set value, check log.
 */
esp_err_t mpu60xx_Interrupt_pin_configuration (mpu60xx_handle_t mpu_handle, mpu60xx_intrpt_config_t *interrupt_configuration);

/**
 * @brief enable event detection
 *
 * @param[in] mpu_handle MPU60xx device handle.
 *
 * @param[in] mdc Configuration for event detection.
 *
 * @param [in] event interrupt to be enabled.
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register.
 *      - other: failed to set value, check log.
 */
esp_err_t mpu60xx_en_EventDetection(mpu60xx_handle_t mpu_handle, mpu60xx_event_detect_config_t *mdc, mpu60xx_event_t event);



/**
 * @brief enable/disable event detection interrupt generation on MPU60xx.
 * @note internally called by mpu60xx_en_EventDetection but provided for flexibility.
 *
 * @param[in] mpu_handle MPU60xx device handle.
 *
 * @param[in] active if 'true' enables interrupt generation,
 *                   if 'false' disables interrupt generation.
 *
 * @param [in] event interrupt to be enabled/disabled.
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register.
 *      - other: failed to set value, check log.
 */
esp_err_t mpu60xx_enEventDetectInterrupt(mpu60xx_handle_t mpu_handle, bool active, mpu60xx_event_t event);

/**
 * @brief read event detect interrupt status
 * @note if "MPU60XX_INTERRUPT_LATCH_UNTIL_CLEARED" was used in "mpu60xx_intrpt_config_t"
 *       this function needs to be called to clear the interrupt flag on mpu60xx,
 *       otherwise further interrupts will not be notified
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[out] status 'true' if event detected otherwise 'false'
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log
 */
esp_err_t mpu60xx_getEventInterruptStatus(mpu60xx_handle_t mpu_handle, bool *status);

/**
 * @brief read Motion detect interrupt status
 * @note if "MPU60XX_INTERRUPT_LATCH_UNTIL_CLEARED" was used in "mpu60xx_intrpt_config_t"
 *       this function needs to be called to clear the interrupt flag on mpu60xx,
 *       otherwise further interrupts will not be notified
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[out] status 'true' if event detected otherwise 'false'
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log
 */
esp_err_t mpu60xx_getMotionInterruptStatus(mpu60xx_handle_t mpu_handle, bool *status);

/**
 * @brief read Zero motion detect interrupt status
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[out] status 'true' if event detected otherwise 'false'
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log
 */
esp_err_t mpu60xx_getZeroMotionInterruptStatus(mpu60xx_handle_t mpu_handle, bool *status);

/**
 * @brief read Free Fall motion detect interrupt status
 * @note if "MPU60XX_INTERRUPT_LATCH_UNTIL_CLEARED" was used in "mpu60xx_intrpt_config_t"
 *       this function needs to be called to clear the interrupt flag on mpu60xx,
 *       otherwise further interrupts will not be notified
 *
 * @param[in] mpu_handle MPU60xx device handle
 *
 * @param[out] status 'true' if event detected otherwise 'false'
 *
 * @return
 *      - ESP_OK: successfully read the Interrupt status register
 *      - other: failed to read value, check log
 */
esp_err_t mpu60xx_getFreeFallMotionInterruptStatus(mpu60xx_handle_t mpu_handle, bool *status);



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
* @param[in] mpu60xx_handler address of
 MPU60xx device handle
*
*/
void mpu60xx_remove (mpu60xx_handle_t *mpu60xx_handler);


#ifdef __cplusplus
}
#endif

#endif // __MPU_60XX_H
