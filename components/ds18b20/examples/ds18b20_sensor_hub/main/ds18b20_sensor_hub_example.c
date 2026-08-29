/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "onewire_bus.h"
#include "ds18b20.h"
#include "iot_sensor_hub.h"

static sensor_handle_t sensor_handle = NULL;

#if CONFIG_EXAMPLE_ONEWIRE_ENABLE_INTERNAL_PULLUP
#define EXAMPLE_ONEWIRE_ENABLE_INTERNAL_PULLUP 1
#else
#define EXAMPLE_ONEWIRE_ENABLE_INTERNAL_PULLUP 0
#endif

#define EXAMPLE_ONEWIRE_UART_PORT_NUM   CONFIG_EXAMPLE_ONEWIRE_UART_PORT_NUM
#define EXAMPLE_ONEWIRE_BUS_GPIO        CONFIG_EXAMPLE_ONEWIRE_BUS_GPIO
#define EXAMPLE_ONEWIRE_MAX_DS18B20     CONFIG_EXAMPLE_ONEWIRE_MAX_DS18B20
#define EXAMPLE_MEASUREMENT_PERIOD      CONFIG_EXAMPLE_MEASUREMENT_PERIOD

static const char *TAG = "example";

void app_main(void)
{
    // install 1-wire bus
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = EXAMPLE_ONEWIRE_BUS_GPIO,
        .flags = {
            .en_pull_up = EXAMPLE_ONEWIRE_ENABLE_INTERNAL_PULLUP,
        }
    };

    onewire_bus_uart_config_t uart_config = {
        .uart_port_num = EXAMPLE_ONEWIRE_UART_PORT_NUM,
    };
    ESP_ERROR_CHECK(onewire_new_bus_uart(&bus_config, &uart_config, &bus));
    ESP_LOGI(TAG, "1-Wire bus installed on GPIO%d by UART backend (UART%d)",
             EXAMPLE_ONEWIRE_BUS_GPIO, EXAMPLE_ONEWIRE_UART_PORT_NUM);

    sensor_config_t config = {
        .type = HUMITURE_ID,
        .mode = MODE_POLLING,
        .min_delay = EXAMPLE_MEASUREMENT_PERIOD,
        .bus = bus,
        // The address is unused because DS18B20 sensor hub implementation expects only a single device on a bus
        .addr = 22,
    };

    ESP_ERROR_CHECK(iot_sensor_create("sensor_hub_ds18b20", &config, &sensor_handle));
    ESP_ERROR_CHECK(iot_sensor_start(sensor_handle));
}
