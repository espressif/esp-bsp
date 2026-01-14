/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_io_expander_tca9554.h"
#include "esp_io_expander_gpio_wrapper.h"

static const char *TAG = "example";

#define I2C_HOST  0

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* Pins are compatible with board esp32_s3_korvo_2 */
#define EXAMPLE_PIN_NUM_SDA           17
#define EXAMPLE_PIN_NUM_SCL           18

void app_main(void)
{
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_HOST,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    ESP_LOGI(TAG, "Initialize IO Expander TCA9554");
    esp_io_expander_handle_t io_expander_handle = NULL; // IO expander tca9554 handle
    if (esp_io_expander_new_i2c_tca9554(i2c_bus, ESP_IO_EXPANDER_I2C_TCA9554A_ADDRESS_000, &io_expander_handle) == ESP_OK
            && io_expander_handle) {
        if (esp_io_expander_gpio_wrapper_append_handler(io_expander_handle, GPIO_NUM_MAX) == ESP_OK) {
            ESP_LOGI(TAG, "Registered IO Expander to GPIOs. IOs from %d", GPIO_NUM_MAX);
        } else {
            ESP_LOGE(TAG, "Failed to register IO Expander to GPIOs.");
        }
    } else {
        ESP_LOGE(TAG, "Failed to initialize TCA9554 IO Expander");
        return;
    }

    const uint8_t io_led1 = GPIO_NUM_MAX + 6; // IO Expander pin 6 (RED)
    const uint8_t io_led2 = GPIO_NUM_MAX + 7; // IO Expander pin 7 (BLUE)

    ESP_LOGI(TAG, "Set IOs output");
    gpio_set_direction(io_led1, GPIO_MODE_OUTPUT);
    gpio_set_direction(io_led2, GPIO_MODE_OUTPUT);

    while (1) {
        ESP_LOGI(TAG, "LED1 set ON");
        gpio_set_level(io_led1, 1);

        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "LED2 set ON");
        gpio_set_level(io_led2, 1);

        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "LED1 set OFF");
        gpio_set_level(io_led1, 0);

        vTaskDelay(pdMS_TO_TICKS(500));

        ESP_LOGI(TAG, "LED2 set OFF");
        gpio_set_level(io_led2, 0);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
