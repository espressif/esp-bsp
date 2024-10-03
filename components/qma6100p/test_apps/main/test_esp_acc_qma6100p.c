/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "qma6100p.h"
#include "esp_system.h"
#include "esp_log.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "unity_test_utils_memory.h"

#define I2C_MASTER_SCL_IO 5       /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 4       /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "qma6100p test";
static qma6100p_handle_t qma6100p = NULL;

/**
 * @brief i2c master initialization
 */
static void i2c_bus_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C config returned error");

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "I2C install returned error");
}

/**
 * @brief i2c master initialization
 */
static void i2c_sensor_qma6100p_init(void)
{
    esp_err_t ret;

    i2c_bus_init();
    qma6100p = qma6100p_create(I2C_MASTER_NUM, QMA6100P_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(qma6100p, "QMA6100P create returned NULL");

    ret = qma6100p_wake_up(qma6100p);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

TEST_CASE("Sensor qma6100p test", "[qma6100p][iot][sensor]")
{
    esp_err_t ret;
    uint8_t qma6100p_deviceid;
    qma6100p_acce_value_t acce;

    i2c_sensor_qma6100p_init();

    ret = qma6100p_get_deviceid(qma6100p, &qma6100p_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(QMA6100P_WHO_AM_I_VAL, qma6100p_deviceid);

    ret = qma6100p_get_acce(qma6100p, &acce);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f\n", acce.acce_x, acce.acce_y, acce.acce_z);

    qma6100p_delete(qma6100p);
    ret = i2c_driver_delete(I2C_MASTER_NUM);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

#define TEST_MEMORY_LEAK_THRESHOLD  (300)

static size_t before_free_8bit;
static size_t before_free_32bit;

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
     *   ___  __  __    _    __   _  ___   ___  ____
     *  / _ \|  \/  |  / \  / /_ / |/ _ \ / _ \|  _ \
     * | | | | |\/| | / _ \| '_ \| | | | | | | | |_) |
     * | |_| | |  | |/ ___ \ (_) | | |_| | |_| |  __/
     *  \__\_\_|  |_/_/   \_\___/|_|\___/ \___/|_|
    */

    printf("  ___  __  __    _    __   _  ___   ___  ____  \r\n");
    printf(" / _ \\|  \\/  |  / \\  / /_ / |/ _ \\ / _ \\|  _ \\ \r\n");
    printf("| | | | |\\/| | / _ \\| '_ \\| | | | | | | | |_) |\r\n");
    printf("| |_| | |  | |/ ___ \\ (_) | | |_| | |_| |  __/ \r\n");
    printf(" \\__\\_\\_|  |_/_/   \\_\\___/|_|\\___/ \\___/|_|    \r\n");
    unity_run_menu();
}
