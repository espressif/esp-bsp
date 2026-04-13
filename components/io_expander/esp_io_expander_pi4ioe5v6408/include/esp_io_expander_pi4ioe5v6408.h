/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP IO expander: PI4IOE5V6408
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
 * @brief I2C address of the PI4IOE5V6408
 *
 * The device supports two I2C addresses:
 * - 0x43 when ADDR pin is low
 * - 0x44 when ADDR pin is high
 */
#define ESP_IO_EXPANDER_I2C_PI4IOE5V6408_ADDRESS_LOW     (0x43)
#define ESP_IO_EXPANDER_I2C_PI4IOE5V6408_ADDRESS_HIGH    (0x44)

/**
 * @brief Create a PI4IOE5V6408 IO expander object
 *
 * @param[in]  i2c_bus    I2C bus handle. Obtained from `i2c_new_master_bus()`
 * @param[in]  dev_addr   I2C device address of chip. Can be `ESP_IO_EXPANDER_I2C_PI4IOE5V6408_ADDRESS_XXX`.
 * @param[out] handle_ret Handle to created IO expander object
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_new_i2c_pi4ioe5v6408(i2c_master_bus_handle_t i2c_bus, uint32_t dev_addr,
        esp_io_expander_handle_t *handle_ret);

#ifdef __cplusplus
}
#endif
