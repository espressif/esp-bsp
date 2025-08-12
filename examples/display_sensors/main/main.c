/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief BSP Sensors Example
 * @details Display sensor data on a monochrome screen (LVGL)
 * @example https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_sensors-
 */

#include <stdio.h>
#include "hts221.h"
#include "mpu6050.h"
#include "fbm320.h"
#include "mag3110.h"
#include "bh1750.h"
#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdmmc_cmd.h" // for sdmmc_card_print_info
#include "esp_idf_version.h" // for backward compatibility of esp-timer

// Enable SD card test
#define EXAMPLE_TEST_SD_CARD 0

static const char *TAG = "example";

// Display
static lv_disp_t *disp = NULL;
static lv_obj_t *main_screen = NULL;
static lv_obj_t *main_label = NULL;

static bh1750_handle_t bh1750_dev = NULL;
static hts221_handle_t hts221_dev = NULL;
static mpu6050_handle_t mpu6050_dev = NULL;
static fbm320_handle_t fbm320_dev = NULL;
static mag3110_handle_t mag3110_dev = NULL;

static QueueHandle_t q_page_num;
static uint8_t g_page_num = 0;

static mpu6050_acce_value_t acce;
static mpu6050_gyro_value_t gyro;
static complimentary_angle_t complimentary_angle;

static void display_show_signs(void)
{
    bsp_display_lock(0);
    lv_obj_clean(main_screen);
    lv_obj_t *label = lv_label_create(main_screen);
    lv_label_set_text_static(label, LV_SYMBOL_WIFI"   "LV_SYMBOL_USB"   "LV_SYMBOL_BELL"   "LV_SYMBOL_GPS"   "LV_SYMBOL_BATTERY_EMPTY);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(label, lv_display_get_physical_horizontal_resolution(disp));
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
    bsp_display_unlock();
}

static void bh1750_init()
{
    bh1750_dev = bh1750_create(BSP_I2C_NUM, BH1750_I2C_ADDRESS_DEFAULT);
    bh1750_power_on(bh1750_dev);
    bh1750_set_measure_mode(bh1750_dev, BH1750_CONTINUE_4LX_RES);
}

static void app_hts221_init()
{
    hts221_dev = hts221_create(BSP_I2C_NUM);

    const hts221_config_t hts221_config = {
        .avg_h = HTS221_AVGH_32, .avg_t = HTS221_AVGT_16, .odr = HTS221_ODR_1HZ, .bdu_status = true
    };
    hts221_init(hts221_dev, &hts221_config);
}

static void mpu6050_init()
{
    mpu6050_dev = mpu6050_create(BSP_I2C_NUM, MPU6050_I2C_ADDRESS);
    mpu6050_config(mpu6050_dev, ACCE_FS_4G, GYRO_FS_500DPS);
    mpu6050_wake_up(mpu6050_dev);
}

static void app_fbm320_init()
{
    fbm320_dev = fbm320_create(BSP_I2C_NUM, FBM320_I2C_ADDRESS_1);
    fbm320_init(fbm320_dev);
}

static void app_mag3110_init()
{
    // MAG3110 is started after its calibration in app_main
    mag3110_dev = mag3110_create(BSP_I2C_NUM);
}

static void display_show_env_data(void)
{
    int16_t temp;
    int16_t humi;
    float lumi;

    hts221_get_temperature(hts221_dev, &temp);
    hts221_get_humidity(hts221_dev, &humi);
    bh1750_get_data(bh1750_dev, &lumi);
    ESP_LOGI(TAG, "temperature: %.1f, humidity: %.1f, luminance: %.1f", (float)temp / 10, (float)humi / 10, lumi);

    bsp_display_lock(0);
    lv_label_set_text_fmt(main_label, "Temp: %.1f\nHumi: %.1f\nLumi: %.1f", (float)temp / 10, (float)humi / 10, lumi);
    lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_LEFT, 0);
    bsp_display_unlock();
}

static void display_show_acce_data(void)
{
    ESP_LOGI(TAG, "acce_x:%.2f, acce_y:%.2f, acce_z:%.2f", acce.acce_x, acce.acce_y, acce.acce_z);

    bsp_display_lock(0);
    lv_label_set_text_fmt(main_label, "Acce_x: %.2f\nAcce_y: %.2f\nAcce_z: %.2f", acce.acce_x, acce.acce_y, acce.acce_z);
    lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_LEFT, 0);
    bsp_display_unlock();
}

static void display_show_gyro_data(void)
{
    ESP_LOGI(TAG, "gyro_x:%.2f, gyro_y:%.2f, gyro_z:%.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);

    bsp_display_lock(0);
    lv_label_set_text_fmt(main_label, "Gyro_x: %.2f\nGyro_y: %.2f\nGyro_z: %.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
    lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_LEFT, 0);
    bsp_display_unlock();
}

static void display_show_complimentary_angle(void)
{
    ESP_LOGI(TAG, "roll:%.2f, pitch:%.2f", complimentary_angle.roll, complimentary_angle.pitch);

    bsp_display_lock(0);
    lv_label_set_text_fmt(main_label, "Roll: %.2f\nPitch: %.2f", complimentary_angle.roll, complimentary_angle.pitch);
    lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_LEFT, 0);
    bsp_display_unlock();
}

static void display_show_barometer_data(void)
{
    int32_t real_p, real_t;
    float pressure, temperature;

    if (ESP_OK == fbm320_get_data(fbm320_dev, FBM320_MEAS_PRESS_OSR_1024, &real_t, &real_p)) {
        pressure = (float)real_p / 1000;
        temperature = (float)real_t / 100;
        ESP_LOGI(TAG, "pressure: %.1f, temperature: %.1f", pressure, temperature);

        bsp_display_lock(0);
        lv_label_set_text_fmt(main_label, "Press: %.1f\nTemp: %.1f", pressure, temperature);
        lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_LEFT, 0);
        bsp_display_unlock();
    }
}

static void display_show_magmeter_data(void)
{
    mag3110_result_t mag_induction;

    mag3110_get_magnetic_induction(mag3110_dev, &mag_induction);
    ESP_LOGI(TAG, "mag_x:%i, mag_y:%i, mag_z:%i", mag_induction.x, mag_induction.y, mag_induction.z);

    bsp_display_lock(0);
    lv_label_set_text_fmt(main_label, "Mag_x: %5i\nMag_y: %5i\nMag_z: %5i", mag_induction.x, mag_induction.y, mag_induction.z);
    lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_LEFT, 0);
    bsp_display_unlock();
}

static void display_show_task(void *pvParameters)
{
    uint8_t page_num = 0;
    while (1) {
        bsp_led_set(BSP_LED_AZURE, false);
        bsp_buzzer_set(false);

        if (xQueueReceive(q_page_num, &page_num, 1000 / portTICK_PERIOD_MS) == pdTRUE) {
            // Turn on LED and Buzzer when button is pressed
            bsp_led_set(BSP_LED_AZURE, true);
            bsp_buzzer_set(true);
        }

        switch (page_num) {
        case 0:
            display_show_env_data();
            break;
        case 1:
            display_show_acce_data();
            break;
        case 2:
            display_show_gyro_data();
            break;
        case 3:
            display_show_complimentary_angle();
            break;
        case 4:
            display_show_barometer_data();
            break;
        case 5:
            display_show_magmeter_data();
            break;
        default:
            break;
        }
    }
}

static void mpu6050_read(void *pvParameters)
{
    mpu6050_get_acce(mpu6050_dev, &acce);
    mpu6050_get_gyro(mpu6050_dev, &gyro);
    mpu6050_complimentory_filter(mpu6050_dev, &acce, &gyro, &complimentary_angle);
}

static void btn_handler(void *button_handle, void *usr_data)
{
    if (++g_page_num >= 6) {
        g_page_num = 0;
    }
    xQueueSend(q_page_num, &g_page_num, 0);
}

void app_main(void)
{
    // Init all board components
    bsp_i2c_init();
    disp = bsp_display_start();
    bh1750_init();
    app_hts221_init();
    mpu6050_init();
    app_fbm320_init();
    app_mag3110_init();
    bsp_leds_init();
    bsp_buzzer_init();

#if EXAMPLE_TEST_SD_CARD
    // Mount uSD card, light up WiFi LED on success
    if (ESP_OK == bsp_sdcard_mount()) {
        bsp_led_set(BSP_LED_WIFI, true); // Signal successful SD card access
        sdmmc_card_t *sdcard = bsp_sdcard_get_handle();
        sdmmc_card_print_info(stdout, sdcard);
        FILE *f = fopen(BSP_SD_MOUNT_POINT "/hello.txt", "w");
        fprintf(f, "Hello %s!\n", sdcard->cid.name);
        fclose(f);
        bsp_sdcard_unmount();
    }
#endif

    main_screen = lv_disp_get_scr_act(NULL);

    // Show icons
    display_show_signs();

    // Display text
    bsp_display_lock(0);
    main_label = lv_label_create(main_screen);
    lv_label_set_text_static(main_label, "Magnetometer\ncalibration");
    lv_obj_set_style_text_align(main_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_width(main_label, lv_display_get_physical_horizontal_resolution(disp));
    lv_obj_align(main_label, LV_ALIGN_TOP_MID, 0, 15);
    bsp_display_unlock();

    // Start magnetometer calibrating procedure
    mag3110_calibrate(mag3110_dev, 10000);
    bsp_display_lock(0);
    lv_label_set_text_static(main_label, "Magnetometer\ncalibration\nDone!");
    bsp_display_unlock();
    mag3110_start(mag3110_dev, MAG3110_DR_OS_10_128); // Magnetometer is stopped after calibration; it must be started here

    // Create FreeRTOS tasks and queues
    q_page_num = xQueueCreate(10, sizeof(uint8_t));
    xTaskCreate(display_show_task, "display_show_task", 2048 * 2, NULL, 5, NULL);

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

    /* Init buttons */
    button_handle_t btns[BSP_BUTTON_NUM];
    ESP_ERROR_CHECK(bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM));
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
#if BUTTON_VER_MAJOR >= 4
        ESP_ERROR_CHECK(iot_button_register_cb(btns[i], BUTTON_PRESS_DOWN, NULL, btn_handler, (void *) i));
#else
        ESP_ERROR_CHECK(iot_button_register_cb(btns[i], BUTTON_PRESS_DOWN, btn_handler, (void *) i));
#endif
    }
}
