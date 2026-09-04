/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_lcd_touch_cst820.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include "unity_test_runner.h"

/* Test wiring and coordinate limits are set in menuconfig so the same test
 * app can be used on different boards. */
#define TEST_TOUCH_I2C_PORT       CONFIG_TEST_TOUCH_I2C_PORT
#define TEST_TOUCH_I2C_SDA        ((gpio_num_t)CONFIG_TEST_TOUCH_I2C_SDA)
#define TEST_TOUCH_I2C_SCL        ((gpio_num_t)CONFIG_TEST_TOUCH_I2C_SCL)
#define TEST_TOUCH_GPIO_INT       ((gpio_num_t)CONFIG_TEST_TOUCH_GPIO_INT)
#define TEST_TOUCH_GPIO_RST       ((gpio_num_t)CONFIG_TEST_TOUCH_GPIO_RST)
#define TEST_TOUCH_H_RES          CONFIG_TEST_TOUCH_X_MAX
#define TEST_TOUCH_V_RES          CONFIG_TEST_TOUCH_Y_MAX

/* Shared bring-up for tests that need a live controller. */
static void cst820_test_open(i2c_master_bus_handle_t *i2c_bus,
                             esp_lcd_panel_io_handle_t *touch_io,
                             esp_lcd_touch_handle_t *touch)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = TEST_TOUCH_I2C_PORT,
        .sda_io_num = TEST_TOUCH_I2C_SDA,
        .scl_io_num = TEST_TOUCH_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    TEST_ESP_OK(i2c_new_master_bus(&bus_config, i2c_bus));

    const esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_TOUCH_IO_I2C_CST820_CONFIG();
    TEST_ESP_OK(esp_lcd_new_panel_io_i2c(*i2c_bus, &io_config, touch_io));

    const esp_lcd_touch_config_t touch_config = {
        .x_max = TEST_TOUCH_H_RES,
        .y_max = TEST_TOUCH_V_RES,
        .rst_gpio_num = TEST_TOUCH_GPIO_RST,
        .int_gpio_num = TEST_TOUCH_GPIO_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
    };
    TEST_ESP_OK(esp_lcd_touch_new_i2c_cst820(*touch_io, &touch_config, touch));
}

static void cst820_test_close(i2c_master_bus_handle_t i2c_bus,
                              esp_lcd_panel_io_handle_t touch_io,
                              esp_lcd_touch_handle_t touch)
{
    TEST_ESP_OK(esp_lcd_touch_del(touch));
    TEST_ESP_OK(esp_lcd_panel_io_del(touch_io));
    TEST_ESP_OK(i2c_del_master_bus(i2c_bus));
}

TEST_CASE("CST820 initializes over I2C", "[cst820][i2c]")
{
    i2c_master_bus_handle_t i2c_bus = NULL;
    esp_lcd_panel_io_handle_t touch_io = NULL;
    esp_lcd_touch_handle_t touch = NULL;

    cst820_test_open(&i2c_bus, &touch_io, &touch);
    cst820_test_close(i2c_bus, touch_io, touch);
}

TEST_CASE("CST820 monitor APIs reject NULL handles", "[cst820][no-hw]")
{
    /* No hardware needed: every implementation validates the handle before
     * touching the bus or any GPIO. */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_lcd_touch_cst820_enter_monitor_mode(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_lcd_touch_cst820_exit_monitor_mode(NULL));

    /* Framework wrappers must route to the same validation. */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_lcd_touch_enter_sleep(NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_lcd_touch_exit_sleep(NULL));
}

TEST_CASE("CST820 raw register accessors reject NULL handles", "[cst820][no-hw]")
{
    uint8_t byte = 0;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_lcd_touch_cst820_read_reg(NULL, 0x00, &byte, 1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, esp_lcd_touch_cst820_write_reg(NULL, 0x00, 0x00));
}


TEST_CASE("CST820 monitor mode wakes the host on touch", "[cst820][hw][waketest]")
{
    i2c_master_bus_handle_t i2c_bus = NULL;
    esp_lcd_panel_io_handle_t touch_io = NULL;
    esp_lcd_touch_handle_t touch = NULL;

    cst820_test_open(&i2c_bus, &touch_io, &touch);

    /* Standby tier: the controller keeps scanning at low frequency and
     * pulses INT when the panel is touched (datasheet standby mode). */
    TEST_ESP_OK(esp_lcd_touch_cst820_enter_monitor_mode(touch));
    /* Arm INT (active low) as the light-sleep wake source. Level-based
     * wake: if EVT shows the standby IRQ pulse is too short to be sampled,
     * revisit the wake configuration. */
    TEST_ESP_OK(gpio_wakeup_enable(TEST_TOUCH_GPIO_INT, GPIO_INTR_LOW_LEVEL));
    TEST_ESP_OK(esp_sleep_enable_gpio_wakeup());

    printf("cst820: entering light sleep, touch the panel to wake up...\n");
    vTaskDelay(pdMS_TO_TICKS(100));   /* Let the console line drain first */

    TEST_ESP_OK(esp_light_sleep_start());

    /* Execution resumes here after the touch. */
    TEST_ASSERT(esp_sleep_get_wakeup_causes() & BIT(ESP_SLEEP_WAKEUP_GPIO));
    printf("cst820: woke up by touch interrupt\n");

    TEST_ESP_OK(gpio_wakeup_disable(TEST_TOUCH_GPIO_INT));
    TEST_ESP_OK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO));

    /* Best effort: the touch that woke the host may already be released by
     * the time the host reads it. */
    if (esp_lcd_touch_read_data(touch) == ESP_OK) {
        esp_lcd_touch_point_data_t points[CONFIG_ESP_LCD_TOUCH_MAX_POINTS];
        uint8_t point_count = 0;
        if (esp_lcd_touch_get_data(touch, points, &point_count,
                                   CONFIG_ESP_LCD_TOUCH_MAX_POINTS) == ESP_OK && point_count > 0) {
            printf("cst820: wake report: id=%u x=%u y=%u\n",
                   points[0].track_id, points[0].x, points[0].y);
        }
    }

    /* Force the controller back to dynamic mode without waiting for another
     * touch (reset path; a touch would have exited standby on its own). */
    TEST_ESP_OK(esp_lcd_touch_cst820_exit_monitor_mode(touch));

    cst820_test_close(i2c_bus, touch_io, touch);
}

void app_main(void)
{
    printf("CST820 test application\n");
    unity_run_menu();
}
