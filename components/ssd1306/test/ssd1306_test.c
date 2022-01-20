/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "unity.h"

#include "esp_log.h"

#include "ssd1306.h"
#include "ssd1306_fonts.h"

#define I2C_MASTER_SCL_IO 26      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 25      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

static const char *TAG = "ssd1306 test";

static ssd1306_handle_t dev = NULL;
static bool test_running;

time_t g_now;

/******** Test Function ****************/
esp_err_t ssd1306_show_time(ssd1306_handle_t dev)
{
    struct tm timeinfo;
    char strftime_buf[64];
    time(&g_now);
    g_now++;
    setenv("TZ", "GMT-8", 1);
    tzset();
    localtime_r(&g_now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);

    ssd1306_draw_3216char(dev, 0, 16, strftime_buf[11]);
    ssd1306_draw_3216char(dev, 16, 16, strftime_buf[12]);
    ssd1306_draw_3216char(dev, 32, 16, strftime_buf[13]);
    ssd1306_draw_3216char(dev, 48, 16, strftime_buf[14]);
    ssd1306_draw_3216char(dev, 64, 16, strftime_buf[15]);
    ssd1306_draw_1616char(dev, 80, 32, strftime_buf[16]);
    ssd1306_draw_1616char(dev, 96, 32, strftime_buf[17]);
    ssd1306_draw_1616char(dev, 112, 32, strftime_buf[18]);

    char *day = strftime_buf;
    day[3] = '\0';
    ssd1306_draw_string(dev, 87, 16, (const uint8_t *)day, 14, 1);
    ssd1306_draw_string(dev, 0, 52, (const uint8_t *)"MUSIC", 12, 0);
    ssd1306_draw_string(dev, 52, 52, (const uint8_t *)"MENU", 12, 0);
    ssd1306_draw_string(dev, 98, 52, (const uint8_t *)"PHONE", 12, 0);

    return ssd1306_refresh_gram(dev);
}

esp_err_t ssd1306_show_signs(ssd1306_handle_t dev)
{
    ssd1306_clear_screen(dev, 0x00);

    ssd1306_draw_bitmap(dev, 0, 2, &c_chSingal816[0], 16, 8);
    ssd1306_draw_bitmap(dev, 24, 2, &c_chBluetooth88[0], 8, 8);
    ssd1306_draw_bitmap(dev, 40, 2, &c_chMsg816[0], 16, 8);
    ssd1306_draw_bitmap(dev, 64, 2, &c_chGPRS88[0], 8, 8);
    ssd1306_draw_bitmap(dev, 90, 2, &c_chAlarm88[0], 8, 8);
    ssd1306_draw_bitmap(dev, 112, 2, &c_chBat816[0], 16, 8);

    return ssd1306_refresh_gram(dev);
}
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

static void dev_ssd1306_initialization(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "OLED initialization");
    i2c_bus_init();
    dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
    TEST_ASSERT_NOT_NULL_MESSAGE(dev, "SSD1306 create returned error");

    ret = ssd1306_refresh_gram(dev);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "SSD1306 refresh gram returned error");

    ret = ssd1306_show_signs(dev);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "SSD1306 show signs returned error");
}

static void ssd1306_test_task(void *pvParameters)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "OLED task start");
    dev_ssd1306_initialization();
    while (test_running) {
        ret = ssd1306_show_time(dev);
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "SSD1306 show time returned error");

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    ESP_LOGI(TAG, "OLED cleaning-up...");

    ssd1306_clear_screen(dev, 0);

    ret = ssd1306_refresh_gram(dev);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, ret, "SSD1306 refresh gram returned error");

    ssd1306_delete(dev);
    ret = i2c_driver_delete(I2C_MASTER_NUM);
    TEST_ASSERT_EQUAL(ESP_OK, ret);

    vTaskDelete(NULL);
}

TEST_CASE("Device ssd1306 test", "[ssd1306][iot][device]")
{
    test_running = true;
    xTaskCreate(&ssd1306_test_task, "ssd1306_test_task", 2048 * 2, NULL, 5, NULL);
    vTaskDelay(10000 / portTICK_PERIOD_MS); // run the test for 10 seconds
    test_running = false;                 // stop the test
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // give the kernel some time to clean-up
}
