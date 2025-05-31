/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "driver/i2c_master.h"
#include "bh1750.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 26      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "bh1750 test";
static bh1750_handle_t bh1750 = NULL;
i2c_master_bus_handle_t my_i2c_bus_handle = NULL;

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    printf("Requesting I2C bus handle\n");
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &my_i2c_bus_handle));
    printf("I2C bus handle acquired\n");
}
void bh1750_init(void)
{
    i2c_master_init();
    bh1750 = bh1750_create(my_i2c_bus_handle, BH1750_I2C_ADDRESS_DEFAULT);
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
    ret = bh1750_delete(&bh1750);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
    ret = i2c_del_master_bus(my_i2c_bus_handle);
    TEST_ASSERT_EQUAL(ESP_OK, ret);
}

void app_main(void)
{
    printf("\n=== BH1750 Sensor Test Menu ===\n");
    unity_run_menu();  // Run test selection menu in flash monitor
}
