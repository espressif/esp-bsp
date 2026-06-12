/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"

#include "bsp/esp_sensairshuttle.h"

/* Feature enable */
esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable)
{
    esp_err_t ret = ESP_OK;

    switch (feature) {
    case BSP_FEATURE_LCD: {
        const gpio_config_t io_config = {
            .pin_bit_mask = BIT64(BSP_LCD_EN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&io_config));
        gpio_set_level(BSP_LCD_EN, !enable);
        break;
    }
    case BSP_FEATURE_SPEAKER: {
        const gpio_config_t io_config = {
            .pin_bit_mask = BIT64(BSP_SPEAKER_EN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&io_config));
        gpio_set_level(BSP_SPEAKER_EN, enable);
        break;
    }
    }

    return ret;
}
