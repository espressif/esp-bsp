/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "driver/i2c_master.h"
#include "icm42670.h"
#include "esp_system.h"
#include "esp_log.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "unity_test_utils_memory.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Pinout for ESP32-S3-BOX
#define I2C_MASTER_SCL_IO 18       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 8       /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */

static const char *TAG = "icm42670 test";
static icm42670_handle_t icm42670 = NULL;
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

static void i2c_sensor_icm42670_init(void)
{
    esp_err_t ret;

    i2c_bus_init();
    ret = icm42670_create(i2c_handle, ICM42670_I2C_ADDRESS, &icm42670);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_NOT_NULL_MESSAGE(icm42670, "icm42670 create returned NULL");

    /* Configuration of the accelerometer and gyroscope */
    const icm42670_cfg_t imu_cfg = {
        .acce_fs = ACCE_FS_2G,
        .acce_odr = ACCE_ODR_400HZ,
        .gyro_fs = GYRO_FS_2000DPS,
        .gyro_odr = GYRO_ODR_400HZ,
    };
    ret = icm42670_config(icm42670, &imu_cfg);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

TEST_CASE("Sensor icm42670 test", "[icm42670]")
{
    esp_err_t ret;
    icm42670_value_t acc, gyro;
    float temperature;

    i2c_sensor_icm42670_init();

    /* Set accelerometer and gyroscope to ON */
    ret = icm42670_acce_set_pwr(icm42670, ACCE_PWR_LOWNOISE);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = icm42670_gyro_set_pwr(icm42670, GYRO_PWR_LOWNOISE);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    for (int i = 0; i < 100; i++) {
        vTaskDelay(pdMS_TO_TICKS(50));
        ret = icm42670_get_acce_value(icm42670, &acc);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = icm42670_get_gyro_value(icm42670, &gyro);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ret = icm42670_get_temp_value(icm42670, &temperature);
        TEST_ASSERT_EQUAL(ESP_OK, ret);
        ESP_LOGI(TAG, "acc_x:%.2f, acc_y:%.2f, acc_z:%.2f, gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f temp: %.1f",
                 acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z, temperature);
    }

    icm42670_delete(icm42670);
    ret = i2c_del_master_bus(i2c_handle);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
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
    unity_run_menu();
}
