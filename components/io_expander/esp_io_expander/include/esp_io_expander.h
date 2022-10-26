/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IO Expander Device Type
 *
 */
typedef struct esp_io_expander_t esp_io_expander_t;
typedef esp_io_expander_t *esp_io_expander_handle_t;

/**
 * @brief IO Expander Configuration Type
 *
 */
typedef struct {
    uint8_t io_amount;                      /*!< Amout of device's IO, must be smaller than 32 */
    struct {
        uint8_t dir_out_bit_zero : 1;       /*!< If the direction of IO is output, the corresponding bit of the direction register is 0 */
        uint8_t input_high_bit_zero : 1;    /*!< If the input level of IO is high, the corresponding bit of the input register is 0 */
        uint8_t output_high_bit_zero : 1;   /*!< If the output level of IO is high, the corresponding bit of the output register is 0 */
    } flags;
    /* Don't support with interrupt mode yet, will be added soon */
} esp_io_expander_config_t;

struct esp_io_expander_t {

    /**
     * @brief Read value from input register (mandatory)
     *
     * @note The value represents the input level from IO
     * @note If there are multiple input registers in the device, their values should be spliced together in order to form the `value`.
     *
     * @param handle: IO Expander handle
     * @param value: Register's value
     *
     * @return
     *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
     */
    esp_err_t (*read_input_reg)(esp_io_expander_handle_t handle, uint32_t *value);

    /**
     * @brief Write value to output register (mandatory)
     *
     * @note The value represents the output level to IO
     * @note If there are multiple input registers in the device, their values should be spliced together in order to form the `value`.
     *
     * @param handle: IO Expander handle
     * @param value: Register's value
     *
     * @return
     *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
     */
    esp_err_t (*write_output_reg)(esp_io_expander_handle_t handle, uint32_t value);

    /**
     * @brief Read value from output register (mandatory)
     *
     * @note The value represents the expected output level to IO
     * @note This function can be implemented by reading the physical output register, or simply by reading a variable that record the output value (more faster)
     * @note If there are multiple input registers in the device, their values should be spliced together in order to form the `value`.
     *
     * @param handle: IO Expander handle
     * @param value: Register's value
     *
     * @return
     *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
     */
    esp_err_t (*read_output_reg)(esp_io_expander_handle_t handle, uint32_t *value);

    /**
     * @brief Write value to direction register (mandatory)
     *
     * @note The value represents the diection of IO
     * @note If there are multiple input registers in the device, their values should be spliced together in order to form the `value`.
     *
     * @param handle: IO Expander handle
     * @param value: Register's value
     *
     * @return
     *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
     */
    esp_err_t (*write_direction_reg)(esp_io_expander_handle_t handle, uint32_t value);

    /**
     * @brief Read value from directioin register (mandatory)
     *
     * @note The value represents the expected direction of IO
     * @note This function can be implemented by reading the physical direction register, or simply by reading a variable that record the direction value (more faster)
     * @note If there are multiple input registers in the device, their values should be spliced together in order to form the `value`.
     *
     * @param handle: IO Expander handle
     * @param value: Register's value
     *
     * @return
     *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
     */
    esp_err_t (*read_direction_reg)(esp_io_expander_handle_t handle, uint32_t *value);

    /**
     * @brief Reset the device to its initial state (mandatory)
     *
     * @note This function will reset all device's registers
     *
     * @param handle: IO Expander handle
     *
     * @return
     *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
     */
    esp_err_t (*reset)(esp_io_expander_t *handle);

    /**
     * @brief Delete device (mandatory)
     *
     * @param handle: IO Expander handle
     *
     * @return
     *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
     */
    esp_err_t (*del)(esp_io_expander_t *handle);

    /**
     * @brief Configuration structure
     */
    esp_io_expander_config_t config;
};

/**
 * @brief Set the direction of a specific IO
 *
 * @param handle: IO Exapnder handle
 * @param pin_num: IO num, valid from 0 to (`io_amount` - 1)
 * @param is_output: true - output, false - input
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_set_dir(esp_io_expander_handle_t handle, uint8_t pin_num, bool is_output);

/**
 * @brief Get the direction of a specific IO
 *
 * @param handle: IO Exapnder handle
 * @param pin_num: IO num, valid from 0 to (`io_amount` - 1)
 * @param is_output: true - output, false - input
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_get_dir(esp_io_expander_handle_t handle, uint8_t pin_num, bool *is_output);

/**
 * @brief Set the output level of a specific IO
 *
 * @note Specific IO must be in input mode first, otherwise this function will return an error
 *
 * @param handle: IO Exapnder handle
 * @param pin_num: IO num, valid from 0 to (`io_amount` - 1)
 * @param level: 0 - Low level, 1 - High level
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_set_level(esp_io_expander_handle_t handle, uint8_t pin_num, uint8_t level);

/**
 * @brief Get the intput level of a specific IO
 *
 * @param handle: IO Exapnder handle
 * @param pin_num: IO num, valid from 0 to (`io_amount` - 1)
 * @param level: 0 - Low level, 1 - High level
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_get_level(esp_io_expander_handle_t handle, uint8_t pin_num, uint8_t *level);

/**
 * @brief Show the current status of each IO of the device, including direction, input level and output level.
 *
 * @param handle: IO Exapnder handle
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_show_state(esp_io_expander_handle_t handle);

/**
 * @brief Reset the device to its initial status
 *
 * @note This function will reset all device's registers
 *
 * @param handle: IO Expander handle
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_reset(esp_io_expander_handle_t handle);

/**
 * @brief Delete device
 *
 * @param handle: IO Expander handle
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_del(esp_io_expander_handle_t handle);

#ifdef __cplusplus
}
#endif
