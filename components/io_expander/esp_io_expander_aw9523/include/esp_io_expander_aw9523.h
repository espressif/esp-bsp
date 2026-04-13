/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP IO expander: AW9523
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "esp_io_expander.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create a AW9523 IO expander object
 *
 * @param[in]  i2c_bus    I2C bus handle. Obtained from `i2c_new_master_bus()`
 * @param[in]  dev_addr   I2C device address of chip. Can be `ESP_IO_EXPANDER_I2C_AW9523_ADDRESS_XX`.
 * @param[out] handle_ret Handle to created IO expander object
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_new_i2c_aw9523(i2c_master_bus_handle_t i2c_bus, uint32_t dev_addr,
        esp_io_expander_handle_t *handle_ret);

/**
 * @brief I2C address of the AW9523
 *
 * The 8-bit address format is as follows:
 *
 *                (Slave Address)
 *     ┌─────────────────┷─────────────────┐
 *  ┌─────┐─────┐─────┐─────┐─────┐─────┐─────┐─────┐
 *  |  1  |  0  |  1  |  1  |  0  | AD1 | AD0 | R/W |
 *  └─────┘─────┘─────┘─────┘─────┘─────┘─────┘─────┘
 *     └────────┯────────────-─┘    └──┯────┘
 *           (Fixed)        (Hareware Selectable)
 *
 * And the 7-bit slave address is the most important data for users.
 * For example, if a chip's AD0,AD1 are connected to GND, it's 7-bit slave address is 1011000b(0x58).
 * Then users can use `ESP_IO_EXPANDER_I2C_AW9523_ADDRESS_000` to init it.
 */
#define ESP_IO_EXPANDER_I2C_AW9523_ADDRESS_00    (0x58)
#define ESP_IO_EXPANDER_I2C_AW9523_ADDRESS_01    (0x59)
#define ESP_IO_EXPANDER_I2C_AW9523_ADDRESS_10    (0x5A)
#define ESP_IO_EXPANDER_I2C_AW9523_ADDRESS_11    (0x5B)

#ifdef __cplusplus
}
#endif
