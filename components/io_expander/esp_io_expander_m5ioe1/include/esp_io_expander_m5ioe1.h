/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP IO expander: M5IOE1
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
 * @brief Create an M5IOE1 IO expander object
 *
 * @note The chip names its pins IO1..IO14. This driver maps them onto
 *       `esp_io_expander_pin_num_t` from zero: `IO_EXPANDER_PIN_NUM_0` is IO1
 *       and `IO_EXPANDER_PIN_NUM_13` is IO14.
 * @note Initialization configures all 14 pins as inputs, push-pull, with
 *       pull-up and pull-down disabled. The chip powers up with open-drain
 *       outputs; use `esp_io_expander_set_output_mode()` to select open-drain.
 * @note ADC, PWM, NeoPixel, the temperature sensor and GPIO interrupts are not
 *       implemented.
 *
 * @param[in]  i2c_bus    I2C bus handle. Obtained from `i2c_new_master_bus()`
 * @param[in]  dev_addr   I2C device address of chip. Can be `ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_XX`.
 * @param[out] handle_ret Handle to created IO expander object
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_new_i2c_m5ioe1(i2c_master_bus_handle_t i2c_bus, uint32_t dev_addr,
        esp_io_expander_handle_t *handle_ret);

/**
 * @brief I2C address of the M5IOE1
 *
 * The 7-bit address is selected by the IO7 level at power-up and can be
 * 0x6F..0x76. Some M5Stack products ship firmware that answers at 0x4F or 0x50.
 *
 * The driver communicates at 100 kHz, which is the chip's power-on I2C speed.
 */
#define ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_6F    (0x6F)
#define ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_70    (0x70)
#define ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_71    (0x71)
#define ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_72    (0x72)
#define ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_73    (0x73)
#define ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_74    (0x74)
#define ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_75    (0x75)
#define ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_76    (0x76)
#define ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_4F    (0x4F)
#define ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_50    (0x50)

#ifdef __cplusplus
}
#endif
