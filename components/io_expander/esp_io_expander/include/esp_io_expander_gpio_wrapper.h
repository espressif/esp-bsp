/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_io_expander.h"

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_IO_EXPANDER_ENABLE_GPIO_API_WRAPPER
/**
 * @brief Append a range of GPIOs additional to the existing embedded GPIO range, given IO expander handler
 *
 * @param handler: IO Expander handle
 * @param start_io_num: Start GPIO number that maps to the IO expander range
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_gpio_wrapper_append_handler(esp_io_expander_handle_t handler, uint32_t start_io_num);

/**
 * @brief Remove a range of already appended GPIOs, given IO expander handler
 *
 * @param handler: IO Expander handle
 *
 * @return
 *      - ESP_OK: Success, otherwise returns ESP_ERR_xxx
 */
esp_err_t esp_io_expander_gpio_wrapper_remove_handler(esp_io_expander_handle_t handler);
#endif // CONFIG_IO_EXPANDER_ENABLE_GPIO_API_WRAPPER

#ifdef __cplusplus
}
#endif
