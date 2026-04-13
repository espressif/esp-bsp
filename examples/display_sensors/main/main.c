/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief BSP Sensors Example
 * @details Acquire sensor data using the sensor hub component
 * @example https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_sensors
 */

#include <stdio.h>
#include "bsp/esp-bsp.h"
#include "esp_log.h"

#define IMU_SAMPLING_PERIOD             300
#define HUMITURE_SAMPLING_PERIOD        500

static const char *TAG = "example";

#if BSP_CAPS_IMU
static sensor_handle_t imu_sensor_handle = NULL;
#endif
#if BSP_CAPS_HUMITURE
static sensor_handle_t humiture_sensor_handle = NULL;
#endif

#if BSP_CAPS_DISPLAY
static lv_obj_t *acc_x_bar, *acc_y_bar, * acc_z_bar;
static lv_obj_t *gyr_x_bar, *gyr_y_bar, * gyr_z_bar;
static lv_style_t bar_style;

static lv_obj_t *chart = NULL;
static lv_chart_series_t *temp_series = NULL;
static lv_chart_series_t *humid_series = NULL;

static void chart_add_value(lv_chart_series_t *series, const int32_t value);
#endif

static void sensor_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
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
                 "humidity=%.2f",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->humidity);
#if BSP_CAPS_DISPLAY & BSP_CAPS_HUMITURE
        chart_add_value(humid_series, (int32_t)(sensor_data->humidity * 10.0f));
#endif
        break;
    case SENSOR_TEMP_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x TEMP_DATA_READY - "
                 "temperature=%.2f",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->temperature);
#if BSP_CAPS_DISPLAY & BSP_CAPS_HUMITURE
        chart_add_value(temp_series, (int32_t)(sensor_data->temperature * 10.0f));
#endif
        break;
    case SENSOR_ACCE_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x ACCE_DATA_READY - "
                 "acce_x=%.2f, acce_y=%.2f, acce_z=%.2f",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->acce.x, sensor_data->acce.y, sensor_data->acce.z);
#if BSP_CAPS_DISPLAY & BSP_CAPS_IMU
        lv_bar_set_value(acc_x_bar, (int32_t)(sensor_data->acce.x * 10.0f), LV_ANIM_OFF);
        lv_bar_set_value(acc_y_bar, (int32_t)(sensor_data->acce.y * 10.0f), LV_ANIM_OFF);
        lv_bar_set_value(acc_z_bar, (int32_t)(sensor_data->acce.z * 10.0f), LV_ANIM_OFF);
#endif
        break;
    case SENSOR_GYRO_DATA_READY:
        ESP_LOGI(TAG, "Timestamp = %llu - %s_0x%x GYRO_DATA_READY - "
                 "gyro_x=%.2f, gyro_y=%.2f, gyro_z=%.2f",
                 sensor_data->timestamp,
                 sensor_data->sensor_name,
                 sensor_data->sensor_addr,
                 sensor_data->gyro.x, sensor_data->gyro.y, sensor_data->gyro.z);
#if BSP_CAPS_DISPLAY & BSP_CAPS_IMU
        lv_bar_set_value(gyr_x_bar, (int32_t)(sensor_data->gyro.x * 10.0f), LV_ANIM_OFF);
        lv_bar_set_value(gyr_y_bar, (int32_t)(sensor_data->gyro.y * 10.0f), LV_ANIM_OFF);
        lv_bar_set_value(gyr_z_bar, (int32_t)(sensor_data->gyro.z * 10.0f), LV_ANIM_OFF);
#endif
        break;
    default:
        ESP_LOGI(TAG, "Timestamp = %" PRIi64 " - event id = %" PRIi32, sensor_data->timestamp, id);
        break;
    }
}

#if BSP_CAPS_DISPLAY & BSP_CAPS_HUMITURE
static void chart_add_value(lv_chart_series_t *series, const int32_t value)
{
    assert(chart != NULL);
    assert(series != NULL);
    lv_chart_set_next_value(chart, series, value);
    uint32_t p = lv_chart_get_point_count(chart);
    uint32_t s = lv_chart_get_x_start_point(chart, series);
    int32_t *a = lv_chart_get_series_y_array(chart, series);
    a[(s + 1) % p] = LV_CHART_POINT_NONE;
    a[(s + 2) % p] = LV_CHART_POINT_NONE;
    lv_chart_refresh(chart);
}

static void chart_init(lv_obj_t *scr)
{
    chart = lv_chart_create(scr);
    lv_chart_set_update_mode(chart, LV_CHART_UPDATE_MODE_CIRCULAR);
    lv_obj_set_style_size(chart, 0, 0, LV_PART_INDICATOR);
    lv_obj_set_size(chart, BSP_LCD_H_RES, (BSP_LCD_V_RES * 2) / 5);
    lv_obj_align(chart, LV_ALIGN_TOP_MID, 0, BSP_LCD_V_RES / 10);

    lv_chart_set_point_count(chart, 250);
    lv_chart_set_axis_range(chart, LV_CHART_AXIS_PRIMARY_Y, 100, 400);
    lv_chart_set_axis_range(chart, LV_CHART_AXIS_SECONDARY_Y, 0, 1000);
    lv_chart_set_div_line_count(chart, 5, 0);

    temp_series = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);
    humid_series = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_BLUE), LV_CHART_AXIS_SECONDARY_Y);

    lv_obj_t *chart_label = lv_label_create(scr);
    lv_label_set_recolor(chart_label, true);
    lv_label_set_text_static(chart_label, "#ff0000 Temperature#");
    lv_obj_align(chart_label, LV_ALIGN_TOP_LEFT, BSP_LCD_H_RES / 8, BSP_LCD_V_RES / 32);
    chart_label = lv_label_create(scr);
    lv_label_set_recolor(chart_label, true);
    lv_label_set_text_static(chart_label, "#0000ff Humidity#");
    lv_obj_align(chart_label, LV_ALIGN_TOP_RIGHT, -BSP_LCD_H_RES / 8, BSP_LCD_V_RES / 32);
}
#endif

#if BSP_CAPS_DISPLAY & BSP_CAPS_IMU
static lv_obj_t *imu_bars_new(lv_obj_t *scr, const lv_style_t *style, const uint32_t range)
{
    lv_obj_t *bar = lv_bar_create(scr);
    lv_bar_set_range(bar, -range, range);
    lv_bar_set_mode(bar, LV_BAR_MODE_SYMMETRICAL);
    lv_obj_add_style(bar, style, 0);
    return bar;
}

static void imu_create_axis_label(lv_obj_t *bar, const char *text)
{
    lv_obj_t *bar_label;
    bar_label = lv_label_create(lv_obj_get_screen(bar));
    lv_label_set_text_static(bar_label, text);
    lv_obj_align_to(bar_label, bar, LV_ALIGN_OUT_LEFT_MID, -BSP_LCD_H_RES / 32, 0);
}

static void imu_bars_init(lv_obj_t *scr)
{
    lv_obj_t *static_label;

    lv_style_init(&bar_style);
    lv_style_set_width(&bar_style, BSP_LCD_H_RES / 3);
    lv_style_set_height(&bar_style, BSP_LCD_V_RES / 16);

    acc_x_bar = imu_bars_new(scr, &bar_style, 15);
    acc_y_bar = imu_bars_new(scr, &bar_style, 15);
    acc_z_bar = imu_bars_new(scr, &bar_style, 15);
    lv_obj_align(acc_z_bar, LV_ALIGN_BOTTOM_LEFT, BSP_LCD_H_RES / 8, -BSP_LCD_V_RES / 16);
    lv_obj_align_to(acc_y_bar, acc_z_bar, LV_ALIGN_OUT_TOP_MID, 0, -BSP_LCD_V_RES / 16);
    lv_obj_align_to(acc_x_bar, acc_y_bar, LV_ALIGN_OUT_TOP_MID, 0, -BSP_LCD_V_RES / 16);

    static_label = lv_label_create(scr);
    lv_label_set_text_static(static_label, "Accelerometer");
    lv_obj_align_to(static_label, acc_x_bar, LV_ALIGN_OUT_TOP_MID, 0, -BSP_LCD_V_RES / 32);
    imu_create_axis_label(acc_x_bar, "X");
    imu_create_axis_label(acc_y_bar, "Y");
    imu_create_axis_label(acc_z_bar, "Z");

    gyr_x_bar = imu_bars_new(scr, &bar_style, 5000);
    gyr_y_bar = imu_bars_new(scr, &bar_style, 5000);
    gyr_z_bar = imu_bars_new(scr, &bar_style, 5000);
    lv_obj_align(gyr_z_bar, LV_ALIGN_BOTTOM_RIGHT, -BSP_LCD_H_RES / 16, -BSP_LCD_V_RES / 16);
    lv_obj_align_to(gyr_y_bar, gyr_z_bar, LV_ALIGN_OUT_TOP_MID, 0, -BSP_LCD_V_RES / 16);
    lv_obj_align_to(gyr_x_bar, gyr_y_bar, LV_ALIGN_OUT_TOP_MID, 0, -BSP_LCD_V_RES / 16);

    static_label = lv_label_create(scr);
    lv_label_set_text_static(static_label, "Gyroscope");
    lv_obj_align_to(static_label, gyr_x_bar, LV_ALIGN_OUT_TOP_MID, 0, -BSP_LCD_V_RES / 32);
    imu_create_axis_label(gyr_x_bar, "X");
    imu_create_axis_label(gyr_y_bar, "Y");
    imu_create_axis_label(gyr_z_bar, "Z");
}
#endif

void app_main(void)
{
#if BSP_CAPS_DISPLAY
    bsp_display_start();
    bsp_display_lock(0);
    lv_obj_t *main_scr = lv_screen_active();
#if BSP_CAPS_HUMITURE
    chart_init(main_scr);
#endif
#if BSP_CAPS_IMU
    imu_bars_init(main_scr);
#endif
    bsp_display_unlock();
    bsp_display_backlight_on();
#endif

#if BSP_CAPS_IMU
    bsp_sensor_config_t imu_config = {
        .type = IMU_ID,
        .mode = MODE_POLLING,
        .period = IMU_SAMPLING_PERIOD
    };
    ESP_ERROR_CHECK(bsp_sensor_init(&imu_config, &imu_sensor_handle));
    iot_sensor_handler_register(imu_sensor_handle, sensor_event_handler, NULL);
    iot_sensor_start(imu_sensor_handle);
#endif

#if BSP_CAPS_HUMITURE
    bsp_sensor_config_t humiture_config = {
        .type = HUMITURE_ID,
        .mode = MODE_POLLING,
        .period = HUMITURE_SAMPLING_PERIOD
    };

    ESP_ERROR_CHECK(bsp_sensor_init(&humiture_config, &humiture_sensor_handle));
    iot_sensor_handler_register(humiture_sensor_handle, sensor_event_handler, NULL);
    iot_sensor_start(humiture_sensor_handle);
#endif
}
