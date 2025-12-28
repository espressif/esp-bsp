/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2025 Frédéric Nadeau
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "unity_test_utils_memory.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_io_expander_ch422g.h"

// Pinout for ESP32-S3-Touch-LCD-4.3
#define I2C_MASTER_SCL_IO   9           /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO   8           /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM      I2C_NUM_1   /*!< I2C port number for master dev */
/*!< I2C address of slave dev */

#define TEST_LOOP_CNT       10
#define TEST_LOOP_DELAY_MS  500
#define TEST_OUTPUT_PINS    (IO_EXPANDER_PIN_NUM_2)

static const char *TAG = "ch422g test";
static esp_io_expander_handle_t io_expander = NULL;
static i2c_master_bus_handle_t i2c_handle = NULL;

static void i2c_bus_init(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_handle);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

static void i2c_bus_deinit(void)
{
    esp_err_t ret = i2c_del_master_bus(i2c_handle);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C uninstall returned error");
}

static void i2c_dev_ch422g_init(void)
{
    esp_err_t ret = esp_io_expander_new_i2c_ch422g(i2c_handle, &io_expander);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "CH422G create returned error");
}

static void i2c_dev_ch422g_deinit(void)
{
    esp_err_t ret = esp_io_expander_del(io_expander);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "CH422G delete returned error");
}

TEST_CASE("IO expander ch422g test", "[ch422g]")
{
    i2c_bus_init();
    i2c_dev_ch422g_init();

    esp_err_t ret;
    /* Test output level function */
    ret = esp_io_expander_set_dir(io_expander, TEST_OUTPUT_PINS, IO_EXPANDER_OUTPUT);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    // Print state
    ret = esp_io_expander_print_state(io_expander);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    for (int i = 0; i < TEST_LOOP_CNT; i++) {
        // Set level to 0
        ESP_LOGI(TAG, "Set level to 0");
        ret = esp_io_expander_set_level(io_expander, TEST_OUTPUT_PINS, 0);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        vTaskDelay(pdMS_TO_TICKS(TEST_LOOP_DELAY_MS / 2));
        // Set level to 1
        ESP_LOGI(TAG, "Set level to 1");
        ret = esp_io_expander_set_level(io_expander, TEST_OUTPUT_PINS, 1);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        vTaskDelay(pdMS_TO_TICKS(TEST_LOOP_DELAY_MS / 2));
    }

    i2c_dev_ch422g_deinit();
    i2c_bus_deinit();
    vTaskDelay(10); // Give FreeRTOS some time to free its resources
}

#define TEST_MEMORY_LEAK_THRESHOLD  (500)

void setUp(void)
{
    unity_utils_set_leak_level(TEST_MEMORY_LEAK_THRESHOLD);
    unity_utils_record_free_mem();
}

void tearDown(void)
{
    unity_utils_evaluate_leaks();
}

void app_main(void)
{
    /**
     *  ______   __    __  __    __   ______    ______    ______  
     * /      \ |  \  |  \|  \  |  \ /      \  /      \  /      \ 
     *|  $$$$$$\| $$  | $$| $$  | $$|  $$$$$$\|  $$$$$$\|  $$$$$$\
     *| $$   \$$| $$__| $$| $$__| $$ \$$__| $$ \$$__| $$| $$ __\$$
     *| $$      | $$    $$| $$    $$ /      $$ /      $$| $$|    \
     *| $$   __ | $$$$$$$$ \$$$$$$$$|  $$$$$$ |  $$$$$$ | $$ \$$$$
     *| $$__/  \| $$  | $$      | $$| $$_____ | $$_____ | $$__| $$
     * \$$    $$| $$  | $$      | $$| $$     \| $$     \ \$$    $$
     *  \$$$$$$  \$$   \$$       \$$ \$$$$$$$$ \$$$$$$$$  \$$$$$$ 
     *
     */
    printf("  ______   __    __  __    __   ______    ______    ______\r\n");
    printf(" /      \\ |  \\  |  \\|  \\  |  \\ /      \\  /      \\  /      \\\r\n");
    printf("|  $$$$$$\\| $$  | $$| $$  | $$|  $$$$$$\\|  $$$$$$\\|  $$$$$$\\\r\n");
    printf("| $$   \\$$| $$__| $$| $$__| $$ \\$$__| $$ \\$$__| $$| $$ __\\$$\r\n");
    printf("| $$      | $$    $$| $$    $$ /      $$ /      $$| $$|    \\\r\n");
    printf("| $$   __ | $$$$$$$$ \\$$$$$$$$|  $$$$$$ |  $$$$$$ | $$ \\$$$$\r\n");
    printf("| $$__/  \\| $$  | $$      | $$| $$_____ | $$_____ | $$__| $$\r\n");
    printf(" \\$$    $$| $$  | $$      | $$| $$     \\| $$     \\ \\$$    $$\r\n");
    printf("  \\$$$$$$  \\$$   \\$$       \\$$ \\$$$$$$$$ \\$$$$$$$$  \\$$$$$$\r\n");
    unity_run_menu();
}
