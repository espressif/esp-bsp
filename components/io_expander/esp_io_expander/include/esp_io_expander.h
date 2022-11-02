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
typedef struct esp_io_expander_s esp_io_expander_t;
typedef esp_io_expander_t *esp_io_expander_handle_t;

/**
 * @brief IO Expander Pin Num
 *
 */
typedef enum {
    IO_EXPANDER_PIN_NUM_0  = 0,
    IO_EXPANDER_PIN_NUM_1  = 1,
    IO_EXPANDER_PIN_NUM_2  = 2,
    IO_EXPANDER_PIN_NUM_3  = 3,
    IO_EXPANDER_PIN_NUM_4  = 4,
    IO_EXPANDER_PIN_NUM_5  = 5,
    IO_EXPANDER_PIN_NUM_6  = 6,
    IO_EXPANDER_PIN_NUM_7  = 7,
    IO_EXPANDER_PIN_NUM_8  = 8,
    IO_EXPANDER_PIN_NUM_9  = 9,
    IO_EXPANDER_PIN_NUM_10 = 10,
    IO_EXPANDER_PIN_NUM_11 = 11,
    IO_EXPANDER_PIN_NUM_12 = 12,
    IO_EXPANDER_PIN_NUM_13 = 13,
    IO_EXPANDER_PIN_NUM_14 = 14,
    IO_EXPANDER_PIN_NUM_15 = 15,
    IO_EXPANDER_PIN_NUM_16 = 16,
    IO_EXPANDER_PIN_NUM_17 = 17,
    IO_EXPANDER_PIN_NUM_18 = 18,
    IO_EXPANDER_PIN_NUM_19 = 19,
    IO_EXPANDER_PIN_NUM_20 = 20,
    IO_EXPANDER_PIN_NUM_21 = 21,
    IO_EXPANDER_PIN_NUM_26 = 26,
    IO_EXPANDER_PIN_NUM_27 = 27,
    IO_EXPANDER_PIN_NUM_28 = 28,
    IO_EXPANDER_PIN_NUM_29 = 29,
    IO_EXPANDER_PIN_NUM_30 = 30,
    IO_EXPANDER_PIN_NUM_31 = 31,
    IO_EXPANDER_PIN_NUM_MAX,
} esp_io_expander_pin_num_t;

/**
 * @brief IO Expander Pin direction
 *
 */
typedef enum {
    IO_EXPANDER_INPUT,          /*!< Input direction */
    IO_EXPANDER_OUTPUT,         /*!< Output dircetion */
} esp_io_expander_dir_t;

/**
 * @brief IO Expander Configuration Type
 *
 */
typedef struct {
    uint8_t io_count;                      /*!< Count of device's IO, must be less or equal than `IO_EXPANDER_PIN_NUM_MAX` */
    struct {
        uint8_t dir_out_bit_zero : 1;       /*!< If the direction of IO is output, the corresponding bit of the direction register is 0 */
        uint8_t input_high_bit_zero : 1;    /*!< If the input level of IO is high, the corresponding bit of the input register is 0 */
        uint8_t output_high_bit_zero : 1;   /*!< If the output level of IO is high, the corresponding bit of the output register is 0 */
    } flags;
    /* Don't support with interrupt mode yet, will be added soon */
} esp_io_expander_config_t;

struct esp_io_expander_s {

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
    esp_err_t (*reset)(esp_io_expander_handle_t handle);

    /**
     * @brief Delete device (mandatory)
     *
     * @param handle: IO Expander handle
     *
     * @return
     *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
     */
    esp_err_t (*del)(esp_io_expander_handle_t handle);

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
 * @param direction: IO direction (only support input or output now)
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_set_dir(esp_io_expander_handle_t handle, esp_io_expander_pin_num_t pin_num, esp_io_expander_dir_t direction);

/**
 * @brief Set the output level of a specific IO
 *
 * @note The IO must be in output mode first, otherwise this function will return the error `ESP_ERR_INVALID_STATE`
 *
 * @param handle: IO Exapnder handle
 * @param pin_num: IO num, valid from 0 to (`io_amount` - 1)
 * @param level: 0 - Low level, 1 - High level
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_set_level(esp_io_expander_handle_t handle, esp_io_expander_pin_num_t pin_num, uint8_t level);

/**
 * @brief Get the intput level of a specific IO
 *
 * @note This function can be called when the IO is in input mode or output mode
 *
 * @param handle: IO Exapnder handle
 * @param pin_num: IO num, valid from 0 to (`io_amount` - 1)
 * @param level: 0 - Low level, 1 - High level
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_get_level(esp_io_expander_handle_t handle, esp_io_expander_pin_num_t pin_num, uint8_t *level);

/**
 * @brief Print the current status of each IO of the device, including direction, input level and output level.
 *
 * @param handle: IO Exapnder handle
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_print_state(esp_io_expander_handle_t handle);

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
