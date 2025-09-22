/*
 * SPDX-FileCopyrightText: Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdio.h>

#include <stdio.h>
#include "unity.h"
#include "driver/i2c_master.h"
#include "bh1750.h"
#include "esp_system.h"
#include "esp_log.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "unity_test_utils_memory.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SCL_IO 26      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25       /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */

static i2c_master_bus_handle_t i2c_handle = NULL;

TEST_CASE("Sensor bh1750 test", "[bh1750]")
{
    bh1750_handle_t bh1750 = NULL;

    esp_err_t ret = bh1750_create(i2c_handle, BH1750_I2C_ADDRESS_DEFAULT, &bh1750);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL_MESSAGE(bh1750, "bh1750 create returned NULL");

    ret = bh1750_power_on(bh1750);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    float lux;
    for (int i = 0; i < 10; i++) {
        ret = bh1750_set_measure_mode(bh1750, BH1750_ONETIME_1LX_RES);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        vTaskDelay(100); // Wait for measurement to be done

        ret = bh1750_get_data(bh1750, &lux);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        printf("Lux: %.2f\n", lux);
    }

    bh1750_delete(bh1750);
    vTaskDelay(10); // Give FreeRTOS some time to free its resources
}

#define TEST_MEMORY_LEAK_THRESHOLD  (500)

void setUp(void)
{
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_handle);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");

    unity_utils_set_leak_level(TEST_MEMORY_LEAK_THRESHOLD);
    unity_utils_record_free_mem();
}

void tearDown(void)
{
    unity_utils_evaluate_leaks();
    esp_err_t ret = i2c_del_master_bus(i2c_handle);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void app_main(void)
{
    unity_run_menu();
}
