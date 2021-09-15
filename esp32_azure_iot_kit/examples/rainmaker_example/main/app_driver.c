// Copyright 2015-2021 Espressif Systems (Shanghai) CO LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sdkconfig.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h> 
#include "esp_log.h"
#include <app_reset.h>
#include "app_priv.h"

static const char *TAG = "app_driver";

/* This is the button that is used for toggling the power */
#define BUTTON_GPIO          BSP_BUTTON_IO
#define BUTTON_ACTIVE_LEVEL  0

static esp_timer_handle_t sensor_timer;

static ssd1306_handle_t ssd1306_dev = NULL;
static hts221_handle_t hts221_dev = NULL;

#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10

static float s_temperature = 0.0;
static float s_humidity = 0.0;

static void app_hts221_init()
{
    hts221_dev = hts221_create(BSP_I2C_NUM);

    const hts221_config_t hts221_config = {
        .avg_h = HTS221_AVGH_32, .avg_t = HTS221_AVGT_16, .odr = HTS221_ODR_1HZ, .bdu_status = true
    };
    hts221_init(hts221_dev, &hts221_config);
}

void app_ssd1306_init()
{
    ssd1306_dev = ssd1306_create(BSP_I2C_NUM, SSD1306_I2C_ADDRESS);
    ssd1306_clear_screen(ssd1306_dev, 0x00);
    ssd1306_refresh_gram(ssd1306_dev);
}

void app_show_env_data(void)
{
    char data_str[10] = {0};

    ssd1306_clear_screen(ssd1306_dev, 0x00);
    ssd1306_refresh_gram(ssd1306_dev);
    ssd1306_draw_string(ssd1306_dev, 4, 0, (const uint8_t *)"Temp & Humid", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)"Temp C:", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)"RH:", 16, 1);

    ESP_LOGI(TAG, "temperature: %.1f, humidity: %.1f", s_temperature, s_humidity);

    sprintf(data_str, "%.1f", s_temperature);
    ssd1306_draw_string(ssd1306_dev, 70, 16, (const uint8_t *)data_str, 16, 1);
    sprintf(data_str, "%.1f", s_humidity);
    ssd1306_draw_string(ssd1306_dev, 70, 32, (const uint8_t *)data_str, 16, 1);

    ssd1306_refresh_gram(ssd1306_dev);
}

void app_show_priv_key_msg(void)
{
    ssd1306_draw_string(ssd1306_dev, 0, 0, (const uint8_t *)"Espressif Demo", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)"BSP Rainmaker", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)"Generating the", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 48, (const uint8_t *)"private key...", 16, 1);
    ssd1306_refresh_gram(ssd1306_dev);
}

void app_show_prov_msg(void)
{
    ssd1306_draw_string(ssd1306_dev, 0, 0, (const uint8_t *)"Espressif Demo", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)"BSP Rainmaker", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)"Starting", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 48, (const uint8_t *)"Provisioning", 16, 1);
    ssd1306_refresh_gram(ssd1306_dev);
}

void app_update_sensors_data(void)
{
    int16_t temp;
    int16_t humi;

    hts221_get_temperature(hts221_dev, &temp);
    hts221_get_humidity(hts221_dev, &humi);

    s_temperature = (float)temp / 10;
    s_humidity = (float)humi / 10;

    app_show_env_data();
}

static void app_sensor_update(void *priv)
{
    app_update_sensors_data();
    
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(temp_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
            esp_rmaker_float(s_temperature));

    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(humid_sensor_device, ESP_RMAKER_PARAM_TEMPERATURE),
            esp_rmaker_float(s_humidity));
}

float app_get_current_temperature()
{
    return s_temperature;
}

float app_get_current_humidity()
{
    return s_humidity;
}

esp_err_t app_sensor_init(void)
{
    esp_timer_create_args_t sensor_timer_conf = {
        .callback = app_sensor_update,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "app_sensor_update_tm"
    };
    if (esp_timer_create(&sensor_timer_conf, &sensor_timer) == ESP_OK) {
        esp_timer_start_periodic(sensor_timer, REPORTING_PERIOD * 1000000U);
        return ESP_OK;
    }
    return ESP_FAIL;
}

void app_start_data_collect(void)
{
    app_sensor_init();
    app_update_sensors_data();
}

void app_driver_init()
{
    // Init BSP and sensors
    bsp_leds_init();
    bsp_buzzer_init();
    bsp_i2c_init();
    app_ssd1306_init();
    app_hts221_init();

    bsp_led_set(BSP_LED_WIFI, false);
    bsp_led_set(BSP_LED_AZURE, false);

    app_update_sensors_data();
    app_show_priv_key_msg();

    app_reset_button_register(app_reset_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL),
                WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);
}
