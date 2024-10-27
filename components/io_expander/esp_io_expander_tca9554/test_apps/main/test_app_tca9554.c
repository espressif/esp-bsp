/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
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
#include "esp_io_expander_tca9554.h"

// Pinout for ESP32-S3-LCD-Ev-Board
#define I2C_MASTER_SCL_IO   48          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO   47          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM      I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_ADDRESS         ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000
/*!< I2C address of slave dev */

#define TEST_LOOP_CNT       10
#define TEST_LOOP_DELAY_MS  500
#define TEST_OUTPUT_PINS    (IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1)
#define TEST_INPUT_PINS     (IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3)

static const char *TAG = "tca9554 test";
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

static void i2c_dev_tca9554_init(void)
{
    esp_err_t ret = esp_io_expander_new_i2c_tca9554(i2c_handle, I2C_ADDRESS, &io_expander);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "TCA9554 create returned error");
}

static void i2c_dev_tca9554_deinit(void)
{
    esp_err_t ret = esp_io_expander_del(io_expander);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "TCA9554 delete returned error");
}

TEST_CASE("IO expander tca9554 test", "[tca9554]")
{
    i2c_bus_init();
    i2c_dev_tca9554_init();

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

    /* Test output level function */
    uint32_t input_level_mask = 0;
    ret = esp_io_expander_set_dir(io_expander, TEST_INPUT_PINS, IO_EXPANDER_INPUT);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    // Print state
    ret = esp_io_expander_print_state(io_expander);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    for (int i = 0; i < TEST_LOOP_CNT; i++) {
        // Get input level
        ret = esp_io_expander_get_level(io_expander, TEST_INPUT_PINS, &input_level_mask);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "Input level mask: 0x%02" PRIX32, input_level_mask);
        vTaskDelay(pdMS_TO_TICKS(TEST_LOOP_DELAY_MS));
    }

    i2c_dev_tca9554_deinit();
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
     *  ________   ______    ______    ______   _______   _______  __    __    ___   ______   ___
     * |        \ /      \  /      \  /      \ |       \ |       \|  \  |  \  /   \ /      \ |   \
     *  \$$$$$$$$|  $$$$$$\|  $$$$$$\|  $$$$$$\| $$$$$$$ | $$$$$$$| $$  | $$ /  $$$|  $$$$$$\ \$$$\
     *    | $$   | $$   \$$| $$__| $$| $$__/ $$| $$____  | $$____ | $$__| $$|  $$  | $$__| $$   \$$\
     *    | $$   | $$      | $$    $$ \$$    $$| $$    \ | $$    \| $$    $$| $$   | $$    $$   | $$
     *    | $$   | $$   __ | $$$$$$$$ _\$$$$$$$ \$$$$$$$\ \$$$$$$$\\$$$$$$$$| $$   | $$$$$$$$   | $$
     *    | $$   | $$__/  \| $$  | $$|  \__/ $$|  \__| $$|  \__| $$     | $$ \$$\_ | $$  | $$ _/  $$
     *    | $$    \$$    $$| $$  | $$ \$$    $$ \$$    $$ \$$    $$     | $$  \$$ \| $$  | $$|   $$
     *     \$$     \$$$$$$  \$$   \$$  \$$$$$$   \$$$$$$   \$$$$$$       \$$   \$$$ \$$   \$$ \$$$
     *
     */
    printf(" ________   ______    ______    ______   _______   _______  __    __    ___   ______   ___\r\n");
    printf("|        \\ /      \\  /      \\  /      \\ |       \\ |       \\|  \\  |  \\  /   \\ /      \\ |   \\\r\n");
    printf(" \\$$$$$$$$|  $$$$$$\\|  $$$$$$\\|  $$$$$$\\| $$$$$$$ | $$$$$$$| $$  | $$ /  $$$|  $$$$$$\\ \\$$$\\\r\n");
    printf("   | $$   | $$   \\$$| $$__| $$| $$__/ $$| $$____  | $$____ | $$__| $$|  $$  | $$__| $$   \\$$\\\r\n");
    printf("   | $$   | $$      | $$    $$ \\$$    $$| $$    \\ | $$    \\| $$    $$| $$   | $$    $$   | $$\r\n");
    printf("   | $$   | $$   __ | $$$$$$$$ _\\$$$$$$$ \\$$$$$$$\\ \\$$$$$$$\\\\$$$$$$$$| $$   | $$$$$$$$   | $$\r\n");
    printf("   | $$   | $$__/  \\| $$  | $$|  \\__/ $$|  \\__| $$|  \\__| $$     | $$ \\$$\\_ | $$  | $$ _/  $$\r\n");
    printf("   | $$    \\$$    $$| $$  | $$ \\$$    $$ \\$$    $$ \\$$    $$     | $$  \\$$ \\| $$  | $$|   $$\r\n");
    printf("    \\$$     \\$$$$$$  \\$$   \\$$  \\$$$$$$   \\$$$$$$   \\$$$$$$       \\$$   \\$$$ \\$$   \\$$ \\$$$\r\n");
    unity_run_menu();
}
