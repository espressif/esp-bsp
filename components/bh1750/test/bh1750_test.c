/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "bh1750.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 26      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "bh1750 test";
static bh1750_handle_t bh1750 = NULL;

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

void bh1750_init(void)
{
    i2c_bus_init();
    bh1750 = bh1750_create(I2C_MASTER_NUM, BH1750_I2C_ADDRESS_DEFAULT);
    TEST_ASSERT_NOT_NULL_MESSAGE(bh1750, "BH1750 create returned NULL");
}

TEST_CASE("Sensor BH1750 test", "[bh1750][iot][sensor]")
{
    esp_err_t ret;
    bh1750_measure_mode_t cmd_measure;
    float bh1750_data;

    bh1750_init();

    ret = bh1750_power_on(bh1750);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    // one-shot mode
    cmd_measure = BH1750_ONETIME_4LX_RES;
    ret = bh1750_set_measure_mode(bh1750, cmd_measure);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    vTaskDelay(30 / portTICK_PERIOD_MS);

    ret = bh1750_get_data(bh1750, &bh1750_data);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "bh1750 val(one time mode): %f\n", bh1750_data);

    // continous mode
    cmd_measure = BH1750_CONTINUE_4LX_RES;
    ret = bh1750_set_measure_mode(bh1750, cmd_measure);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    vTaskDelay(30 / portTICK_PERIOD_MS);

    ret = bh1750_get_data(bh1750, &bh1750_data);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ESP_LOGI(TAG, "bh1750 val(continuously mode): %f\n", bh1750_data);

    // clean-up
    ret = bh1750_delete(bh1750);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = i2c_driver_delete(I2C_MASTER_NUM);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}
