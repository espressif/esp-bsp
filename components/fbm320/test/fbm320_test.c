/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "fbm320.h"
#include "esp_system.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 26      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "fbm320 test";
static fbm320_handle_t fbm320 = NULL;

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
static void i2c_sensor_fbm320_init(void)
{
    i2c_bus_init();
    fbm320 = fbm320_create(I2C_MASTER_NUM, FBM320_I2C_ADDRESS_1);
    TEST_ASSERT_NOT_NULL_MESSAGE(fbm320, "FBM320 create returned NULL");
}

TEST_CASE("Sensor fbm320 test", "[fbm320][iot][sensor]")
{
    esp_err_t ret;
    uint8_t fbm320_deviceid;
    int32_t temperature, pressure;

    i2c_sensor_fbm320_init();

    ret = fbm320_get_deviceid(fbm320, &fbm320_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(FBM320_WHO_AM_I_VAL, fbm320_deviceid, "Who Am I register does not contain expected data");

    ret = fbm320_init(fbm320);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = fbm320_get_data(fbm320, FBM320_MEAS_PRESS_OSR_1024, &temperature, &pressure);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ESP_LOGI(TAG, "pressure: %.1f kPa, temperature: %.1f degC", (float)pressure / 1000, (float)temperature / 100);

    fbm320_delete(fbm320);
    ret = i2c_driver_delete(I2C_MASTER_NUM);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}
