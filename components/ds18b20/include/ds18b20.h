/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "onewire_device.h"
#include "ds18b20_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Type of DS18B20 device handle
 */
typedef struct ds18b20_device_t *ds18b20_device_handle_t;

/**
 * @brief DS18B20 configuration
 */
typedef struct {
} ds18b20_config_t;

/**
 * @brief Create a new DS18B20 device from the enumeration result
 *
 * @note The enumeration result contains an abstracted 1-Wire device,
 *       this function is going to wrap it into a specific DS18B20 device.
 *
 * @param[in] device Abstracted 1-Wire device handle
 * @param[in] config DS18B20 configuration
 * @param[out] ret_ds18b20 Returned DS18B20 device handle
 * @return
 *      - ESP_OK: Create DS18B20 device successfully
 *      - ESP_ERR_INVALID_ARG: Create DS18B20 device failed due to invalid argument
 *      - ESP_ERR_NO_MEM: Create DS18B20 device failed due to out of memory
 *      - ESP_ERR_NOT_SUPPORTED: Create DS18B20 device failed because the device is unknown (e.g. a wrong family ID code)
 *      - ESP_FAIL: Create DS18B20 device failed due to other reasons
 */
esp_err_t ds18b20_new_device_from_enumeration(onewire_device_t *device, const ds18b20_config_t *config, ds18b20_device_handle_t *ret_ds18b20);

/**
 * @brief Create a new DS18B20 device from the 1-Wire bus without enumeration
 *
 * @note This function assumes that the device is a DS18B20 device and there is only one device on the bus.
 *
 * @param[in] bus 1-Wire bus handle
 * @param[in] config DS18B20 configuration
 * @param[out] ret_ds18b20 Returned DS18B20 device handle
 * @return
 *      - ESP_OK: Create DS18B20 device successfully
 *      - ESP_ERR_INVALID_ARG: Create DS18B20 device failed due to invalid argument
 *      - ESP_ERR_NO_MEM: Create DS18B20 device failed due to out of memory
 *      - ESP_FAIL: Create DS18B20 device failed due to other reasons
 */
esp_err_t ds18b20_new_device_from_bus(onewire_bus_handle_t bus, const ds18b20_config_t *config, ds18b20_device_handle_t *ret_ds18b20);

/**
 * @brief Delete DS18B20 device
 *
 * @param ds18b20 DS18B20 device handle
 * @return
 *      - ESP_OK: Delete DS18B20 device successfully
 *      - ESP_ERR_INVALID_ARG: Delete DS18B20 device failed due to invalid argument
 *      - ESP_FAIL: Delete DS18B20 device failed due to other reasons
 */
esp_err_t ds18b20_del_device(ds18b20_device_handle_t ds18b20);

/**
 * @brief Set the temperature conversion resolution for a single DS18B20 device
 *
 * @param[in] ds18b20 DS18B20 device handle
 * @param[in] resolution resolution of DS18B20's temperature conversion
 * @return
 *      - ESP_OK: Set resolution successfully
 *      - ESP_ERR_INVALID_ARG: Set resolution failed due to invalid argument
 *      - ESP_FAIL: Set resolution failed due to other reasons
 */
esp_err_t ds18b20_set_resolution(ds18b20_device_handle_t ds18b20, ds18b20_resolution_t resolution);

/**
 * @brief Trigger temperature conversion for a single DS18B20 device
 *
 * @note After send the trigger command, the DS18B20 will start temperature conversion.
 *       This function will delay for some while, to ensure the temperature conversion won't be interrupted.
 *
 * @param[in] ds18b20 DS18B20 device handle
 * @return
 *      - ESP_OK: Trigger temperature conversion successfully
 *      - ESP_ERR_INVALID_ARG: Trigger temperature conversion failed due to invalid argument
 *      - ESP_FAIL: Trigger temperature conversion failed due to other reasons
 */
esp_err_t ds18b20_trigger_temperature_conversion(ds18b20_device_handle_t ds18b20);

/**
 * @brief Trigger temperature conversion for all DS18B20 sensors on the same bus
 *
 * @note After send the trigger command, all the DS18B20 devices will start temperature conversion.
 *       This function will delay for some while, to ensure the temperature conversion won't be interrupted.
 *
 * @param[in] bus 1-Wire bus handle
 * @return
 *      - ESP_OK: Trigger temperature conversion successfully
 *      - ESP_ERR_INVALID_ARG: Trigger temperature conversion failed due to invalid argument
 *      - ESP_FAIL: Trigger temperature conversion failed due to other reasons
 */
esp_err_t ds18b20_trigger_temperature_conversion_for_all(onewire_bus_handle_t bus);

/**
 * @brief Get temperature from a single DS18B20 device
 *
 * @param[in] ds18b20 DS18B20 device handle
 * @param[out] temperature conversion result from DS18B20
 * @return
 *      - ESP_OK: Get temperature successfully
 *      - ESP_ERR_INVALID_ARG: Get temperature failed due to invalid argument
 *      - ESP_ERR_INVALID_CRC: Get temperature failed due to CRC check error
 *      - ESP_FAIL: Get temperature failed due to other reasons
 */
esp_err_t ds18b20_get_temperature(ds18b20_device_handle_t ds18b20, float *temperature);

/**
 * @brief Get the address of the DS18B20 device
 *
 * @param[in] ds18b20 DS18B20 device handle
 * @param[out] ret_address Pointer to store the device address
 * @return
 *      - ESP_OK: Get device address successfully
 *      - ESP_ERR_INVALID_ARG: Get device address failed due to invalid argument
 */
esp_err_t ds18b20_get_device_address(ds18b20_device_handle_t ds18b20, onewire_device_address_t *ret_address);

#ifdef __cplusplus
}
#endif
