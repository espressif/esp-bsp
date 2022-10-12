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

#include <stdio.h>
#include "hts221.h"
#include "mpu6050.h"
#include "fbm320.h"
#include "mag3110.h"
#include "bh1750.h"
#include "ssd1306.h"
#include "esp32_azure_iot_kit.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdmmc_cmd.h" // for sdmmc_card_print_info
#include "esp_idf_version.h" // for backward compatibility of esp-timer

static const char *TAG = "BSP - Azure IoT kit";

static bh1750_handle_t bh1750_dev = NULL;
static hts221_handle_t hts221_dev = NULL;
static ssd1306_handle_t ssd1306_dev = NULL;
static mpu6050_handle_t mpu6050_dev = NULL;
static fbm320_handle_t fbm320_dev = NULL;
static mag3110_handle_t mag3110_dev = NULL;

static QueueHandle_t q_page_num;
static uint8_t g_page_num = 0;

static mpu6050_acce_value_t acce;
static mpu6050_gyro_value_t gyro;
static complimentary_angle_t complimentary_angle;

void ssd1306_show_signs(ssd1306_handle_t dev)
{
    ssd1306_draw_bitmap(dev, 0, 2, &c_chSingal816[0], 16, 8);
    ssd1306_draw_bitmap(dev, 24, 2, &c_chBluetooth88[0], 8, 8);
    ssd1306_draw_bitmap(dev, 40, 2, &c_chMsg816[0], 16, 8);
    ssd1306_draw_bitmap(dev, 64, 2, &c_chGPRS88[0], 8, 8);
    ssd1306_draw_bitmap(dev, 90, 2, &c_chAlarm88[0], 8, 8);
    ssd1306_draw_bitmap(dev, 112, 2, &c_chBat816[0], 16, 8);
}

void app_ssd1306_init()
{
    ssd1306_dev = ssd1306_create(BSP_I2C_NUM, SSD1306_I2C_ADDRESS);
    ssd1306_clear_screen(ssd1306_dev, 0x00);
    ssd1306_show_signs(ssd1306_dev);
    ssd1306_refresh_gram(ssd1306_dev);
}

void bh1750_init()
{
    bh1750_dev = bh1750_create(BSP_I2C_NUM, BH1750_I2C_ADDRESS_DEFAULT);
    bh1750_power_on(bh1750_dev);
    bh1750_set_measure_mode(bh1750_dev, BH1750_CONTINUE_4LX_RES);
}

void app_hts221_init()
{
    hts221_dev = hts221_create(BSP_I2C_NUM);

    const hts221_config_t hts221_config = {
        .avg_h = HTS221_AVGH_32, .avg_t = HTS221_AVGT_16, .odr = HTS221_ODR_1HZ, .bdu_status = true
    };
    hts221_init(hts221_dev, &hts221_config);
}

void mpu6050_init()
{
    mpu6050_dev = mpu6050_create(BSP_I2C_NUM, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050_dev);
}

void app_fbm320_init()
{
    fbm320_dev = fbm320_create(BSP_I2C_NUM, FBM320_I2C_ADDRESS_1);
    fbm320_init(fbm320_dev);
}

void app_mag3110_init()
{
    // MAG3110 is started after its calibration in app_main
    mag3110_dev = mag3110_create(BSP_I2C_NUM);
}

void ssd1306_show_env_data(void)
{
    int16_t temp;
    int16_t humi;
    float lumi;
    char data_str[10] = {0};

    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)"Temp:", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)"Humi:", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 48, (const uint8_t *)"Lumi:", 16, 1);

    hts221_get_temperature(hts221_dev, &temp);
    hts221_get_humidity(hts221_dev, &humi);
    bh1750_get_data(bh1750_dev, &lumi);
    ESP_LOGI(TAG, "temperature: %.1f, humidity: %.1f, luminance: %.1f", (float)temp / 10, (float)humi / 10, lumi);

    sprintf(data_str, "%.1f", (float)temp / 10);
    ssd1306_draw_string(ssd1306_dev, 70, 16, (const uint8_t *)data_str, 16, 1);
    sprintf(data_str, "%.1f", (float)humi / 10);
    ssd1306_draw_string(ssd1306_dev, 70, 32, (const uint8_t *)data_str, 16, 1);
    sprintf(data_str, "%.1f", lumi);
    ssd1306_draw_string(ssd1306_dev, 70, 48, (const uint8_t *)data_str, 16, 1);

    ssd1306_refresh_gram(ssd1306_dev);
}

void ssd1306_show_acce_data(void)
{
    char data_str[10] = {0};

    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)"Acce_x:", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)"Acce_y:", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 48, (const uint8_t *)"Acce_z:", 16, 1);

    ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f", acce.acce_x, acce.acce_y, acce.acce_z);

    sprintf(data_str, "%.2f", acce.acce_x);
    ssd1306_draw_string(ssd1306_dev, 70, 16, (const uint8_t *)data_str, 16, 1);
    sprintf(data_str, "%.2f", acce.acce_y);
    ssd1306_draw_string(ssd1306_dev, 70, 32, (const uint8_t *)data_str, 16, 1);
    sprintf(data_str, "%.2f", acce.acce_z);
    ssd1306_draw_string(ssd1306_dev, 70, 48, (const uint8_t *)data_str, 16, 1);

    ssd1306_refresh_gram(ssd1306_dev);
}

void ssd1306_show_gyro_data(void)
{
    char data_str[10] = {0};

    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)"Gyro_x:", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)"Gyro_y:", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 48, (const uint8_t *)"Gyro_z:", 16, 1);

    ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

    sprintf(data_str, "%.2f", gyro.gyro_x);
    ssd1306_draw_string(ssd1306_dev, 70, 16, (const uint8_t *)data_str, 16, 1);
    sprintf(data_str, "%.2f", gyro.gyro_y);
    ssd1306_draw_string(ssd1306_dev, 70, 32, (const uint8_t *)data_str, 16, 1);
    sprintf(data_str, "%.2f", gyro.gyro_z);
    ssd1306_draw_string(ssd1306_dev, 70, 48, (const uint8_t *)data_str, 16, 1);

    ssd1306_refresh_gram(ssd1306_dev);
}

void ssd1306_show_complimentary_angle(void)
{
    char data_str[10] = {0};

    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)"Roll:", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)"Pitch:", 16, 1);

    ESP_LOGI(TAG, "roll:%.2f, pitch:%.2f", complimentary_angle.roll, complimentary_angle.pitch);

    sprintf(data_str, "%.2f", complimentary_angle.roll);
    ssd1306_draw_string(ssd1306_dev, 70, 16, (const uint8_t *)data_str, 16, 1);
    sprintf(data_str, "%.2f", complimentary_angle.pitch);
    ssd1306_draw_string(ssd1306_dev, 70, 32, (const uint8_t *)data_str, 16, 1);

    ssd1306_refresh_gram(ssd1306_dev);
}

void ssd1306_show_barometer_data(void)
{
    float pressure, temperature;
    char data_str[10] = {0};

    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)"Press:", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)"Temp:", 16, 1);

    int32_t real_p, real_t;

    if (ESP_OK == fbm320_get_data(fbm320_dev, FBM320_MEAS_PRESS_OSR_1024, &real_t, &real_p)) {
        pressure = (float)real_p / 1000;
        temperature = (float)real_t / 100;
        ESP_LOGI(TAG, "pressure: %.1f, temperature: %.1f", pressure, temperature);

        sprintf(data_str, "%.1f", pressure);
        ssd1306_draw_string(ssd1306_dev, 70, 16, (const uint8_t *)data_str, 16, 1);
        sprintf(data_str, "%.1f", temperature);
        ssd1306_draw_string(ssd1306_dev, 70, 32, (const uint8_t *)data_str, 16, 1);

        ssd1306_refresh_gram(ssd1306_dev);
    }
}

void ssd1306_show_magmeter_data(void)
{
    mag3110_result_t mag_induction;
    char data_str[10] = {0};

    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)"Mag_x:", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)"Mag_y:", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 48, (const uint8_t *)"Mag_z:", 16, 1);

    mag3110_get_magnetic_induction(mag3110_dev, &mag_induction);
    ESP_LOGI(TAG, "mag_x:%i, mag_y:%i, mag_z:%i", mag_induction.x, mag_induction.y, mag_induction.z);

    sprintf(data_str, "%5i", mag_induction.x);
    ssd1306_draw_string(ssd1306_dev, 70, 16, (const uint8_t *)data_str, 16, 1);
    sprintf(data_str, "%5i", mag_induction.y);
    ssd1306_draw_string(ssd1306_dev, 70, 32, (const uint8_t *)data_str, 16, 1);
    sprintf(data_str, "%5i", mag_induction.z);
    ssd1306_draw_string(ssd1306_dev, 70, 48, (const uint8_t *)data_str, 16, 1);

    ssd1306_refresh_gram(ssd1306_dev);
}

void ssd1306_show_task(void *pvParameters)
{
    uint8_t page_num = 0;
    ssd1306_clear_screen(ssd1306_dev, 0x00);
    ssd1306_show_signs(ssd1306_dev);
    while (1) {
        if (xQueueReceive(q_page_num, &page_num, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
            ssd1306_clear_screen(ssd1306_dev, 0x00);
            ssd1306_show_signs(ssd1306_dev);
        }

        switch (page_num) {
        case 0:
            ssd1306_show_env_data();
            break;
        case 1:
            ssd1306_show_acce_data();
            break;
        case 2:
            ssd1306_show_gyro_data();
            break;
        case 3:
            ssd1306_show_complimentary_angle();
            break;
        case 4:
            ssd1306_show_barometer_data();
            break;
        case 5:
            ssd1306_show_magmeter_data();
            break;
        default:
            break;
        }
    }
}

void mpu6050_read(void *pvParameters)
{
    mpu6050_get_acce(mpu6050_dev, &acce);
    mpu6050_get_gyro(mpu6050_dev, &gyro);
    mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle);
}

void app_main(void)
{
    // Init all board components
    bsp_i2c_init();
    app_ssd1306_init();
    bh1750_init();
    app_hts221_init();
    mpu6050_init();
    app_fbm320_init();
    app_mag3110_init();
    bsp_leds_init();
    bsp_buzzer_init();

    // Mount uSD card, light up WiFi LED on success
    if (ESP_OK == bsp_sdcard_mount()) {
        bsp_led_set(BSP_LED_WIFI, true); // Signal successful SD card access
        sdmmc_card_print_info(stdout, bsp_sdcard);
        FILE *f = fopen(BSP_MOUNT_POINT "/hello.txt", "w");
        fprintf(f, "Hello %s!\n", bsp_sdcard->cid.name);
        fclose(f);
        bsp_sdcard_unmount();
    }

    // Start magnetometer calibrating procedure
    // Rotate the board in every axis for 10 seconds
    ssd1306_draw_string(ssd1306_dev, 0, 16, (const uint8_t *)"Magnetometer", 16, 1);
    ssd1306_draw_string(ssd1306_dev, 0, 32, (const uint8_t *)"calibration", 16, 1);
    ssd1306_refresh_gram(ssd1306_dev);
    mag3110_calibrate(mag3110_dev, 10000);
    ssd1306_draw_string(ssd1306_dev, 0, 48, (const uint8_t *)"Done!", 16, 1);
    ssd1306_refresh_gram(ssd1306_dev);
    mag3110_start(mag3110_dev, MAG3110_DR_OS_10_128); // Magnetometer is stopped after calibration; it must be started here

    // Create FreeRTOS tasks and queues
    q_page_num = xQueueCreate(10, sizeof(uint8_t));
    xTaskCreate(ssd1306_show_task, "ssd1306_show_task", 2048 * 2, NULL, 5, NULL);

    // In order to get accurate calculation of complimentary angle we need fast reading (5ms)
    // FreeRTOS resolution is 10ms, so esp_timer is used
    const esp_timer_create_args_t cal_timer_config = {
        .callback = mpu6050_read,
        .arg = NULL,
        .name = "MPU6050 timer",
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 3, 0)
        .skip_unhandled_events = true,
#endif
        .dispatch_method = ESP_TIMER_TASK
    };
    esp_timer_handle_t cal_timer = NULL;
    esp_timer_create(&cal_timer_config, &cal_timer);
    esp_timer_start_periodic(cal_timer, 5000); // 5ms

    // Scan button presses
    while (1) {
        if (bsp_button_get()) {
            // Turn on LED and Buzzer when button is pressed
            bsp_led_set(BSP_LED_AZURE, true);
            bsp_buzzer_set(true);

            if (++g_page_num >= 6) {
                g_page_num = 0;
            }
            xQueueSend(q_page_num, &g_page_num, 0);
        } else {
            bsp_led_set(BSP_LED_AZURE, false);
            bsp_buzzer_set(false);
        }

        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
