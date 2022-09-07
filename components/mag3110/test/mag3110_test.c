/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "mag3110.h"
#include "esp_system.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 26      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "mag3110 test";
static mag3110_handle_t mag3110 = NULL;

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
static void i2c_sensor_mag3110_init(void)
{
    i2c_bus_init();
    mag3110 = mag3110_create(I2C_MASTER_NUM);
    TEST_ASSERT_NOT_NULL_MESSAGE(mag3110, "mag3110 create returned NULL");
}

TEST_CASE("Sensor mag3110 test", "[mag3110][iot][sensor]")
{
    esp_err_t ret;
    uint8_t mag3110_deviceid;
    mag3110_result_t mag_induction;

    i2c_sensor_mag3110_init();

    ret = mag3110_get_deviceid(mag3110, &mag3110_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(MAG3110_WHO_AM_I_VAL, mag3110_deviceid, "Who Am I register does not contain expected data");

    ret = mag3110_start(mag3110, MAG3110_DR_OS_10_128);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = mag3110_get_magnetic_induction(mag3110, &mag_induction);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "mag_x:%i, mag_y:%i, mag_z:%i", mag_induction.x, mag_induction.y, mag_induction.z);

    mag3110_delete(mag3110);
    ret = i2c_driver_delete(I2C_MASTER_NUM);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}
