/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "driver/i2c_master.h"
#include "bmi270.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_log.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "unity_test_utils_memory.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_MASTER_SDA_IO GPIO_NUM_31
#define I2C_MASTER_SCL_IO GPIO_NUM_32
#define I2C_MASTER_NUM I2C_NUM_0

#define TEST_MEMORY_LEAK_THRESHOLD  (108)

static const char *TAG = "BMI270 test";

static i2c_master_bus_handle_t i2c_handle = NULL;
static bmi270_handle_t *bmi270 = NULL;

static void bmi270_sensor_init()
{
    esp_err_t ret;

    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };

    ret = i2c_new_master_bus(&bus_config, &i2c_handle);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "Failed to add new I2C master");

    const bmi270_driver_config_t bmi270_driver = {
        .addr = BMI270_I2C_ADDRESS_L,
        .interface = BMI270_USE_I2C,
        .i2c_bus = i2c_handle
    };
    ret = bmi270_create(&bmi270_driver, &bmi270);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL_MESSAGE(bmi270, "Initialized BMI270 handle is NULL");
}

TEST_CASE("BMI270 memory leak test", "[bmi270]")
{
    unity_utils_set_leak_level(TEST_MEMORY_LEAK_THRESHOLD);
    unity_utils_record_free_mem();

    esp_err_t ret;
    float x, y, z;

    bmi270_sensor_init();

    const bmi270_config_t bmi270_config = {
        .acce_odr = BMI270_ACC_ODR_100_HZ,
        .acce_range = BMI270_ACC_RANGE_4_G,
        .gyro_odr = BMI270_GYR_ODR_100_HZ,
        .gyro_range = BMI270_GYR_RANGE_1000_DPS
    };

    ret = bmi270_start(bmi270, &bmi270_config);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = bmi270_get_acce_data(bmi270, &x, &y, &z);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = bmi270_get_gyro_data(bmi270, &x, &y, &z);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = bmi270_stop(bmi270);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = bmi270_delete(bmi270);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = i2c_del_master_bus(i2c_handle);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    unity_utils_evaluate_leaks();
}

TEST_CASE("BMI270 unintialized handle", "[bmi270]")
{
    esp_err_t ret;
    uint8_t chip_id;
    bmi270_handle_t bmi270_handle;
    bmi270_handle.initialized = false;

    const bmi270_config_t bmi270_config = {
        .acce_odr = BMI270_ACC_ODR_100_HZ,
        .acce_range = BMI270_ACC_RANGE_4_G,
        .gyro_odr = BMI270_GYR_ODR_100_HZ,
        .gyro_range = BMI270_GYR_RANGE_1000_DPS
    };

    ret = bmi270_start(&bmi270_handle, &bmi270_config);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);

    ret = bmi270_stop(&bmi270_handle);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);

    ret = bmi270_get_chip_id(&bmi270_handle, &chip_id);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);

    ret = bmi270_set_acce_odr(&bmi270_handle, BMI270_ACC_ODR_100_HZ);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);

    ret = bmi270_set_acce_range(&bmi270_handle, BMI270_ACC_RANGE_4_G);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);

    ret = bmi270_set_gyro_odr(&bmi270_handle, BMI270_GYR_ODR_100_HZ);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);

    ret = bmi270_set_gyro_range(&bmi270_handle, BMI270_GYR_RANGE_1000_DPS);
    TEST_ASSERT_NOT_EQUAL(ESP_OK, ret);
}

void setUp(void)
{

}

void tearDown(void)
{

}

void app_main(void)
{
    ESP_LOGI(TAG, "Running tests with [bmi270] tag");
    UNITY_BEGIN();
    unity_run_tests_by_tag("[bmi270]", false);
    UNITY_END();
}
