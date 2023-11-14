/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "led_indicator_blink_default.h"

static const char *TAG = "example";

#if CONFIG_BSP_LEDS_NUM > 0
static int example_sel_effect = BSP_LED_BREATHE_SLOW;
static led_indicator_handle_t leds[BSP_LED_NUM];
#endif

#if CONFIG_BSP_BUTTONS_NUM > 0
static void btn_handler(void *button_handle, void *usr_data)
{
    int button_pressed = (int)usr_data;
    ESP_LOGI(TAG, "Button pressed: %d. ", button_pressed);

#if CONFIG_BSP_LEDS_NUM > 0
    led_indicator_stop(leds[0], example_sel_effect);

    if (button_pressed == 0) {
        example_sel_effect++;
        if (example_sel_effect >= BSP_LED_MAX) {
            example_sel_effect = BSP_LED_ON;
        }
    }

    ESP_LOGI(TAG, "Changed LED blink effect: %d.", example_sel_effect);
    led_indicator_start(leds[0], example_sel_effect);
#endif
}
#endif

void app_main(void)
{
#if CONFIG_BSP_BUTTONS_NUM > 0
    /* Init buttons */
    button_handle_t btns[BSP_BUTTON_NUM];
    ESP_ERROR_CHECK(bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM));
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        ESP_ERROR_CHECK(iot_button_register_cb(btns[i], BUTTON_PRESS_DOWN, btn_handler, (void *) i));
    }
#endif

#if CONFIG_BSP_LEDS_NUM > 0
    /* Init LEDs */
    ESP_ERROR_CHECK(bsp_led_indicator_create(leds, NULL, BSP_LED_NUM));

    /* Set LED color for first LED (only for addressable RGB LEDs) */
    led_indicator_set_rgb(leds[0], SET_IRGB(0, 0x00, 0x64, 0x64));

    /*
    Start effect for each LED
    (predefined: BSP_LED_ON, BSP_LED_OFF, BSP_LED_BLINK_FAST, BSP_LED_BLINK_SLOW, BSP_LED_BREATHE_FAST, BSP_LED_BREATHE_SLOW)
    */
    led_indicator_start(leds[0], BSP_LED_BREATHE_SLOW);
#endif
}
