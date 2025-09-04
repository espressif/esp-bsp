/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
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
 * @brief PI4IOE5V6408 register configuration
 */
typedef struct {
    uint8_t io_dir;        /*!< Direction register value (0=input, 1=output) */
    uint8_t out_h_im;      /*!< Output high impedance register value */
    uint8_t pull_sel;      /*!< Pull-up/down select register value (0=down, 1=up) */
    uint8_t pull_en;       /*!< Pull-up/down enable register value (0=disable, 1=enable) */
    uint8_t in_def_sta;    /*!< Input default status register value (0xFF=skip) */
    uint8_t int_mask;      /*!< Interrupt mask register value (0xFF=skip) */
    uint8_t out_set;       /*!< Output set register value */
} esp_io_expander_pi4ioe5v6408_config_t;

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
esp_err_t esp_io_expander_new_i2c_pi4ioe5v6408(i2c_master_bus_handle_t i2c_bus, uint32_t dev_addr, esp_io_expander_handle_t *handle_ret);

/**
 * @brief Configure PI4IOE5V6408 with complete register settings
 *
 * This function provides access to all PI4IOE5V6408 specific registers
 * that are not covered by the standard io_expander interface.
 *
 * @param[in] handle Handle to PI4IOE5V6408 device
 * @param[in] config Pointer to configuration
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_pi4ioe5v6408_config_registers(esp_io_expander_handle_t handle, const esp_io_expander_pi4ioe5v6408_config_t *config);

#ifdef __cplusplus
}
#endif
