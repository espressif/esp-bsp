/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create a new FT5x06 touch driver
 *
 * @note The I2C communication should be initialized before use this function.
 *
 * @param config: Touch configuration
 * @param out_touch: Touch instance handle
 * @return
 *      - ESP_OK                    on success
 *      - ESP_ERR_NO_MEM            if there is no memory for allocating main structure
 *      - ESP_ERR_INVALID_ARG       if there is used bad I2C number or GPIO
 */
esp_err_t esp_lcd_touch_new_i2c_ft5x06(const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch);


#ifdef __cplusplus
}
#endif
