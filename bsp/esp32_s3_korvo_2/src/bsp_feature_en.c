/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"

#include "bsp/esp32_s3_korvo_2.h"

/* Feature enable */
esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable)
{
    esp_err_t ret = ESP_OK;

    switch (feature) {
    case BSP_FEATURE_LCD: {
        esp_io_expander_handle_t io_expander = bsp_io_expander_init();
        ret |= esp_io_expander_set_dir(io_expander, BSP_LCD_EN, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_expander, BSP_LCD_EN, !enable);
        break;
    }
    case BSP_FEATURE_TOUCH: {
        esp_io_expander_handle_t io_expander = bsp_io_expander_init();
        ret |= esp_io_expander_set_dir(io_expander, BSP_TOUCH_EN, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_expander, BSP_TOUCH_EN, !enable);
        break;
    }
    }

    return ret;
}
