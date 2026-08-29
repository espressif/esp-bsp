/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_err_check.h"
#include "esp_io_expander_pi4ioe5v6408.h"

#include "bsp/m5stack_tab5.h"

static esp_io_expander_handle_t io_expander = NULL;  // IO Expander
static esp_io_expander_handle_t io_expander1 = NULL;  // IO Expander

esp_io_expander_handle_t bsp_io_expander_init(void)
{
    if (io_expander) {
        return io_expander;
    }
    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());

    BSP_ERROR_CHECK_RETURN_NULL(esp_io_expander_new_i2c_pi4ioe5v6408(bsp_i2c_get_handle(),
                                BSP_IO_EXPANDER_ADDRESS, &io_expander));

    return io_expander;
}

esp_io_expander_handle_t bsp_io_expander1_init(void)
{
    if (io_expander1) {
        return io_expander1;
    }
    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());

    BSP_ERROR_CHECK_RETURN_NULL(esp_io_expander_new_i2c_pi4ioe5v6408(bsp_i2c_get_handle(),
                                BSP_IO_EXPANDER_ADDRESS_1, &io_expander1));

    return io_expander1;
}
