/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

/* Called only for gpio_num >= GPIO_NUM_MAX (virtual expander pins). */
esp_err_t esp_io_expander_gpio_wrapper_set_level(gpio_num_t gpio_num, uint32_t level);
int esp_io_expander_gpio_wrapper_get_level(gpio_num_t gpio_num);
esp_err_t esp_io_expander_gpio_wrapper_set_direction(gpio_num_t gpio_num, gpio_mode_t mode);
esp_err_t esp_io_expander_gpio_wrapper_set_pull_mode(gpio_num_t gpio_num, gpio_pull_mode_t pull);
esp_err_t esp_io_expander_gpio_wrapper_configure_pin(gpio_num_t gpio_num, const gpio_config_t *config);
esp_err_t esp_io_expander_gpio_wrapper_reset_pin(gpio_num_t gpio_num);
