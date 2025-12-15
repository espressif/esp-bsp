/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

#include "driver/i2c_types.h"
#include "icm42670.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 2
#define I2C_NUM -1
#define I2C_SDA 7
#define I2C_SCL 8

static led_strip_handle_t led_strip;

static icm42670_handle_t imu = NULL;

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

static void imu_init (void) {

    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    ESP_ERROR_CHECK(icm42670_create(bus_handle, ICM42670_I2C_ADDRESS, &imu));
    if (imu) {
        /* Configuration of the accelerometer and gyroscope */
        const icm42670_cfg_t imu_cfg = {
            .acce_fs = ACCE_FS_2G,
            .acce_odr = ACCE_ODR_400HZ,
            .gyro_fs = GYRO_FS_2000DPS,
            .gyro_odr = GYRO_ODR_400HZ,
        };
        ESP_ERROR_CHECK(icm42670_config(imu, &imu_cfg));

        /* Set accelerometer and gyroscope to ON */
        icm42670_acce_set_pwr(imu, ACCE_PWR_LOWNOISE);
        icm42670_gyro_set_pwr(imu, GYRO_PWR_LOWNOISE);
    }
}

static void imu_read(void)
{
    icm42670_value_t acce_val;
    icm42670_get_acce_value(imu, &acce_val);
    ESP_LOGI(TAG, "ACCE val: %.2f, %.2f, %.2f", acce_val.x, acce_val.y, acce_val.z);

    icm42670_value_t gyro_val;
    icm42670_get_gyro_value(imu, &gyro_val);
    ESP_LOGI(TAG, "GYRO val: %.2f, %.2f, %.2f", gyro_val.x, gyro_val.y, gyro_val.z);

    float val;
    icm42670_get_temp_value(imu, &val);
    ESP_LOGI(TAG, "TEMP val: %.2f", val);

    complimentary_angle_t angle;
    icm42670_complimentory_filter(imu, &acce_val, &gyro_val, &angle);
    ESP_LOGI(TAG, "Angle roll: %.2f pitch: %.2f ", angle.roll, angle.pitch);
}

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    imu_init();

    while (1) {
        imu_read();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
