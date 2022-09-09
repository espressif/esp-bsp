/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "hts221.h"
#include "esp_system.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 26      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "hts221 test";
static hts221_handle_t hts221 = NULL;

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
static void i2c_sensor_hts221_init(void)
{
    i2c_bus_init();
    hts221 = hts221_create(I2C_MASTER_NUM);
    TEST_ASSERT_NOT_NULL_MESSAGE(hts221, "HTS221 create returned NULL");
}

TEST_CASE("Sensor hts221 test", "[hts221][iot][sensor]")
{
    esp_err_t ret;
    uint8_t hts221_deviceid;
    int16_t temperature;
    int16_t humidity;

    const hts221_config_t hts221_config_write = {
        .avg_h = HTS221_AVGH_32,
        .avg_t = HTS221_AVGT_16,
        .odr = HTS221_ODR_1HZ,
        .bdu_status = true,
    };

    hts221_config_t hts221_config_read;

    i2c_sensor_hts221_init();

    ret = hts221_get_deviceid(hts221, &hts221_deviceid);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(HTS221_WHO_AM_I_VAL, hts221_deviceid, "Who Am I register does not contain expected data");

    ret = hts221_init(hts221, &hts221_config_write);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    ret = hts221_get_config(hts221, &hts221_config_read);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    TEST_ASSERT_EQUAL(hts221_config_write.avg_h, hts221_config_read.avg_h);
    TEST_ASSERT_EQUAL(hts221_config_write.avg_t, hts221_config_read.avg_t);
    TEST_ASSERT_EQUAL(hts221_config_write.odr, hts221_config_read.odr);
    TEST_ASSERT_EQUAL(hts221_config_write.bdu_status, hts221_config_read.bdu_status);

    ret = hts221_get_humidity(hts221, &humidity);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "humidity value is: %2.2f %%", (float)humidity / 10);

    ret = hts221_get_temperature(hts221, &temperature);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "temperature value is: %2.2f degC", (float)temperature / 10);

    hts221_delete(hts221);
    ret = i2c_driver_delete(I2C_MASTER_NUM);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}
