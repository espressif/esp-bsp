/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP LCD touch: CST820
 */

#pragma once

#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create a new CST820 touch driver
 *
 * @note The I2C bus and panel IO must be initialized before calling this function.
 *
 * @param io LCD panel IO handle created by `esp_lcd_new_panel_io_i2c()`
 * @param config Touch controller configuration
 * @param tp Returned touch controller handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if an argument is invalid
 *      - ESP_ERR_NO_MEM if memory allocation fails
 *      - Other error codes propagated from GPIO or panel IO operations
 */
esp_err_t esp_lcd_touch_new_i2c_cst820(const esp_lcd_panel_io_handle_t io,
                                       const esp_lcd_touch_config_t *config,
                                       esp_lcd_touch_handle_t *tp);

/**
 * @brief Prepare the CST820 for monitor (standby) mode (~10 uA, touch wake)
 *
 * Monitor mode is the low-power mode in which the controller keeps scanning
 * the panel at a low frequency. On a touch (or a predefined standby gesture)
 * it autonomously returns to dynamic mode and pulses the INT pin, which can
 * wake the host from light/deep sleep. This is the mode to use for
 * "wake the product by touching the screen" scenarios; in standby a touch
 * keeps INT fully functional.
 *
 * This function deliberately sends no undocumented register write. Per the
 * CST820 datasheet the controller enters standby automatically when no touch
 * is detected for 2 s, and that path keeps INT wake fully functional, so the
 * host can simply stop polling and go to sleep. Typical low-power sequence:
 *
 * 1. esp_lcd_touch_cst820_enter_monitor_mode(tp)
 * 2. arm the INT GPIO as SoC wake source (e.g. gpio_wakeup_enable() +
 *    esp_sleep_enable_gpio_wakeup() for light sleep)
 * 3. esp_light_sleep_start() - resumes when the panel is touched
 * 4. read the touch report; optionally esp_lcd_touch_cst820_exit_monitor_mode()
 *
 * @note Allow up to 2 s from the last touch for the controller to reach
 *       standby (datasheet auto-standby timeout). If EVT verifies a forced
 *       standby command for this firmware, this function may start sending
 *       it; the API contract stays unchanged.
 *
 * @param tp Touch controller handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if tp is NULL
 */
esp_err_t esp_lcd_touch_cst820_enter_monitor_mode(esp_lcd_touch_handle_t tp);

/**
 * @brief Force the CST820 back to dynamic mode from monitor (standby) mode
 *
 * Per the datasheet, standby is exited either by detecting a touch or by a
 * reset; this function takes the reset path so the host does not have to
 * wait for a touch. Waits Tron = 100 ms after the reset pulse and clears
 * cached points.
 *
 * @note A reset GPIO is mandatory; without one this function returns
 *       ESP_ERR_NOT_SUPPORTED (the controller still exits standby on its
 *       own as soon as the panel is touched).
 *
 * @param tp Touch controller handle
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if tp is NULL
 *      - ESP_ERR_NOT_SUPPORTED if no reset GPIO was configured
 *      - Other error codes propagated from GPIO or panel IO operations
 */
esp_err_t esp_lcd_touch_cst820_exit_monitor_mode(esp_lcd_touch_handle_t tp);

/**
 * @brief Read one or more CST820 registers (raw fallback channel)
 *
 * The public CST820 datasheet does not document the register map, so some
 * controller features (gesture report bytes, auto-standby control, multi-key
 * reports) have no structured driver API. This raw accessor lets users
 * validate firmware-variant behavior directly. Use with care: no locking is
 * added beyond what the panel IO layer already provides.
 *
 * @param tp Touch controller handle
 * @param reg Register address
 * @param data Read buffer
 * @param len Number of bytes to read (must be > 0)
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if tp or data is NULL
 *      - ESP_ERR_INVALID_SIZE if len is zero
 *      - I2C error codes propagated from the panel IO layer
 */
esp_err_t esp_lcd_touch_cst820_read_reg(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief Write one CST820 register (raw fallback channel)
 *
 * See esp_lcd_touch_cst820_read_reg() for the rationale and caveats.
 *
 * @param tp Touch controller handle
 * @param reg Register address
 * @param data Value to write
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG if tp is NULL
 *      - I2C error codes propagated from the panel IO layer
 */
esp_err_t esp_lcd_touch_cst820_write_reg(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t data);

/** Default 7-bit I2C address of the CST820 controller. */
#define ESP_LCD_TOUCH_IO_I2C_CST820_ADDRESS    (0x15)

/**
 * @brief Default I2C panel IO configuration for CST820
 */
#define ESP_LCD_TOUCH_IO_I2C_CST820_CONFIG()              \
    {                                                     \
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_CST820_ADDRESS,  \
        .scl_speed_hz = 400000,                           \
        .control_phase_bytes = 1,                         \
        .dc_bit_offset = 0,                               \
        .lcd_cmd_bits = 8,                                \
        .lcd_param_bits = 0,                              \
        .flags =                                          \
        {                                                 \
            .disable_control_phase = 1,                   \
        }                                                 \
    }

#ifdef __cplusplus
}
#endif
