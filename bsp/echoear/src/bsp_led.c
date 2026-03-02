/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_err_check.h"
#include "led_indicator_strips.h"
#include "led_indicator_gpio.h"
#include "led_indicator_ledc.h"
#include "led_indicator_rgb.h"

#include "bsp/echoear.h"
extern blink_step_t const *bsp_led_blink_defaults_lists[];


static led_indicator_gpio_config_t bsp_leds_gpio_config[] = {
    {
        .is_active_level_high = 0,
        .gpio_num = BSP_LED_1_IO,
    },
};


static const led_indicator_config_t bsp_leds_config = {
    .blink_lists = bsp_led_blink_defaults_lists,
    .blink_list_num = BSP_LED_MAX,
};

esp_err_t bsp_led_indicator_create(led_indicator_handle_t led_array[], int *led_cnt, int led_array_size)
{
    esp_err_t ret = ESP_OK;
    if ((led_array_size < BSP_LED_NUM) ||
            (led_array == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (led_cnt) {
        *led_cnt = 0;
    }
    for (int i = 0; i < BSP_LED_NUM; i++) {
        ret = led_indicator_new_gpio_device(&bsp_leds_config, &bsp_leds_gpio_config[i], &led_array[i]);
        BSP_ERROR_CHECK_RETURN_ERR(ret);
        if (led_cnt) {
            (*led_cnt)++;
        }
    }
    return ret;
}

esp_err_t bsp_led_set(led_indicator_handle_t handle, const bool on)
{
    if (on) {
        led_indicator_start(handle, BSP_LED_ON);
    } else {
        led_indicator_start(handle, BSP_LED_OFF);
    }

    return ESP_OK;
}
