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

#include "bsp/esp_sensairshuttle.h"
extern blink_step_t const *bsp_led_blink_defaults_lists[];


static const led_strip_config_t bsp_leds_rgb_strip_config = {
    .strip_gpio_num = BSP_LED_RGB_IO,   // The GPIO that connected to the LED strip's data line
    .max_leds = 1,            // The number of LEDs in the strip,
    .led_model = LED_MODEL_WS2812,      // LED strip model
    .flags.invert_out = false,          // whether to invert the output signal
};

#if CONFIG_BSP_LED_RGB_BACKEND_RMT
static const led_strip_rmt_config_t bsp_leds_rgb_rmt_config = {
    .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
    .resolution_hz = 10 * 1000 * 1000,     // RMT counter clock frequency = 10MHz
    .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
};
#elif CONFIG_BSP_LED_RGB_BACKEND_SPI
static led_strip_spi_config_t bsp_leds_rgb_spi_config = {
    .spi_bus = SPI1_HOST,
    .flags.with_dma = true,
};
#else
#error "unsupported LED strip backend"
#endif

static led_indicator_strips_config_t bsp_leds_rgb_config = {
    .led_strip_cfg = bsp_leds_rgb_strip_config,
#if CONFIG_BSP_LED_RGB_BACKEND_RMT
    .led_strip_driver = LED_STRIP_RMT,
    .led_strip_rmt_cfg = bsp_leds_rgb_rmt_config,
#elif CONFIG_BSP_LED_RGB_BACKEND_SPI
    .led_strip_driver = LED_STRIP_SPI,
    .led_strip_spi_cfg = bsp_leds_rgb_spi_config,
#endif
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
        ret = led_indicator_new_strips_device(&bsp_leds_config, &bsp_leds_rgb_config, &led_array[i]);
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
