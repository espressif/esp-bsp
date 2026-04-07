/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_err_check.h"
#include "esp_io_expander_tca9554.h"
#include "esp_io_expander_gpio_wrapper.h"

#include "bsp/esp32_s3_korvo_2.h"

static esp_io_expander_handle_t io_expander = NULL;  // IO Expander

esp_io_expander_handle_t bsp_io_expander_init(void)
{
    if (io_expander) {
        return io_expander;
    }
    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());

    // Here we try to initialize TCA9554 first, if it fails, we try to initialize TCA9554A
    if (i2c_master_probe(bsp_i2c_get_handle(), BSP_IO_EXPANDER_ADDRESS_A, 100) == ESP_OK) {
        BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_new_i2c_tca9554(bsp_i2c_get_handle(),
                                   BSP_IO_EXPANDER_ADDRESS_A, &io_expander));
    } else if (i2c_master_probe(bsp_i2c_get_handle(), BSP_IO_EXPANDER_ADDRESS, 100) == ESP_OK) {
        BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_new_i2c_tca9554(bsp_i2c_get_handle(),
                                   BSP_IO_EXPANDER_ADDRESS, &io_expander));
    }
#if CONFIG_IO_EXPANDER_ENABLE_GPIO_API_WRAPPER
    // Map expander pins to virtual GPIO numbers starting at GPIO_NUM_MAX
    esp_io_expander_gpio_wrapper_append_handler(io_expander, GPIO_NUM_MAX);
#endif

    return io_expander;
}
