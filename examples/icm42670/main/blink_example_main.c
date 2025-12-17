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
#include "shtc3.h"

#include "iot_sensor_hub.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO 2
#define I2C_NUM -1
#define I2C_SDA 7
#define I2C_SCL 8

static led_strip_handle_t led_strip;

static sensor_handle_t imu_sensor_handle = NULL;

static sensor_handle_t humiture_sensor_handle = NULL;

static i2c_master_bus_handle_t bus_handle;

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

void sensor_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
    sensor_data_t *sensor_data = (sensor_data_t *)event_data;

    switch (id) {
    case SENSOR_STARTED:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x STARTED",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr);
        break;
    case SENSOR_STOPED:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x STOPPED",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr);
        break;
    case SENSOR_HUMI_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x HUMI_DATA_READY - "
                 "humiture=%.2f",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->humidity);
        break;
    case SENSOR_TEMP_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x TEMP_DATA_READY - "
                 "temperature=%.2f\n",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->temperature);
        break;
    case SENSOR_ACCE_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x ACCE_DATA_READY - "
                 "acce_x=%.2f, acce_y=%.2f, acce_z=%.2f\n",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->acce.x, sensor_data->acce.y, sensor_data->acce.z);
        break;
    case SENSOR_GYRO_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x GYRO_DATA_READY - "
                 "gyro_x=%.2f, gyro_y=%.2f, gyro_z=%.2f\n",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->gyro.x, sensor_data->gyro.y, sensor_data->gyro.z);
        break;
    case SENSOR_LIGHT_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x LIGHT_DATA_READY - "
                 "light=%.2f",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->light);
        break;
    case SENSOR_RGBW_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x RGBW_DATA_READY - "
                 "r=%.2f, g=%.2f, b=%.2f, w=%.2f\n",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->rgbw.r, sensor_data->rgbw.r, sensor_data->rgbw.b, sensor_data->rgbw.w);
        break;
    case SENSOR_UV_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x UV_DATA_READY - "
                 "uv=%.2f, uva=%.2f, uvb=%.2f\n",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->uv.uv, sensor_data->uv.uva, sensor_data->uv.uvb);
        break;
    default:
        ESP_LOGI(TAG, "Timestamp = %" PRIi64 " - event id = %" PRIi32, sensor_data->timestamp, id);
        break;
    }
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

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    const sensor_config_t config = {
        .bus = bus_handle,            /*which bus sensors will connect to*/
        .type = IMU_ID,               /*sensor type*/
        .addr = ICM42670_I2C_ADDRESS, /*sensor addr*/
        .mode = MODE_POLLING,              /*data acquire mode*/
        .min_delay = 100         /*data acquire period*/
    };

    iot_sensor_create("virtual_icm42670", &config, &imu_sensor_handle);
    iot_sensor_handler_register(imu_sensor_handle, sensor_event_handler, NULL);
    iot_sensor_start(imu_sensor_handle);
    
}

static void humiture_init (void) {

    const sensor_config_t config = {
        .bus = bus_handle,            /*which bus sensors will connect to*/
        .type = HUMITURE_ID,          /*sensor type*/
        .addr = SHTC3_I2C_ADDR,       /*sensor addr*/
        .mode = MODE_POLLING,         /*data acquire mode*/
        .min_delay = 500         /*data acquire period*/
    };

    iot_sensor_create("virtual_shtc3", &config, &humiture_sensor_handle);
    iot_sensor_handler_register(humiture_sensor_handle, sensor_event_handler, NULL);
    iot_sensor_start(humiture_sensor_handle);
    
}

void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led();
    imu_init();
    humiture_init();

    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
