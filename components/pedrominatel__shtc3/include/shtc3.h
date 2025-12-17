/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief shtc3 driver
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c_master.h"

// SHTC3 I2C address
#define SHTC3_I2C_ADDR          ((uint8_t)0x70) // I2C address of SHTC3 sensor
#define SHTC3_WARMUP_US         (240)           // Warm-up for SHTC3 sensor in us (typical 180, max 240)

// SHTC3 register addresses write only
typedef enum {
    SHTC3_REG_READ_ID       = 0xEFC8, // Read ID register
    SHTC3_REG_WAKE          = 0x3517, // Wake up sensor
    SHTC3_REG_SLEEP         = 0xB098, // Put sensor to sleep
    SHTC3_REG_SOFT_RESET    = 0x805D  // Soft reset
} shtc3_register_w_t;

// SHTC3 register addresses read-write
typedef enum {
    SHTC3_REG_T_CSE_NM  = 0x7CA2, // Temperature first with clock stretching enabled in normal mode
    SHTC3_REG_RH_CSE_NM = 0x5C24, // Humidity first with clock stretching enabled in normal mode
    SHTC3_REG_T_CSE_LM  = 0x6458, // Temperature first with clock stretching enabled in low power mode
    SHTC3_REG_RH_CSE_LM = 0x44DE, // Humidity first with clock stretching enabled in low power mode
    SHTC3_REG_T_CSD_NM  = 0x7866, // Temperature first with clock stretching disabled in normal mode
    SHTC3_REG_RH_CSD_NM = 0x58E0, // Humidity first with clock stretching disabled in normal mode
    SHTC3_REG_T_CSD_LM  = 0x609C, // Temperature first with clock stretching disabled in low power mode
    SHTC3_REG_RH_CSD_LM = 0x401A  // Humidity first with clock stretching disabled in low power mode
} shtc3_register_rw_t;

/**
 * @brief Creates a handle for the SHTC3 device on the specified I2C bus.
 *
 * This function initializes and returns a handle for the SHTC3 sensor device
 * connected to the given I2C bus. The device address and communication speed
 * must be specified.
 *
 * @param bus_handle Handle to the I2C bus where the SHTC3 device is connected.
 * @param dev_addr I2C address of the SHTC3 device.
 * @param dev_speed Communication speed for the I2C device.
 * @return Handle to the SHTC3 device, or NULL if creation failed.
 */
i2c_master_dev_handle_t shtc3_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed);

/**
 * @brief Deletes the SHTC3 device instance.
 *
 * This function releases any resources associated with the SHTC3 device
 * instance identified by the provided I2C master device handle.
 *
 * @param dev_handle The handle to the I2C master device associated with the SHTC3 device.
 * 
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Failed to delete the device
 */
esp_err_t shtc3_device_delete(i2c_master_dev_handle_t dev_handle);

/**
 * @brief Retrieve temperature and humidity readings from the SHTC3 sensor.
 *
 * This function communicates with the SHTC3 sensor over I2C to obtain the current
 * temperature and humidity measurements.
 *
 * @param[in] dev_handle Handle to the I2C master device.
 * @param[in] reg Register to read from.
 * @param[out] data1 Pointer to a float where the temperature reading will be stored.
 * @param[out] data2 Pointer to a float where the humidity reading will be stored.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 *     - ESP_FAIL: Communication with the sensor failed
 */
esp_err_t shtc3_get_th(i2c_master_dev_handle_t dev_handle, shtc3_register_rw_t reg, float *data1, float *data2);

/**
 * @brief Retrieve the ID from the SHTC3 sensor.
 *
 * This function communicates with the SHTC3 sensor over I2C to obtain its unique identifier.
 *
 * @param dev_handle Handle to the I2C master device.
 * @param id Pointer to a buffer where the ID will be stored.
 * 
 * @return 
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid argument
 *     - ESP_FAIL: Communication with the sensor failed
 */
esp_err_t shtc3_get_id(i2c_master_dev_handle_t dev_handle, uint8_t *id);

#ifdef __cplusplus
}
#endif
