/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "bsp/esp-bsp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_camera.h"

static const char *TAG = "example";

void app_main(void)
{
    bsp_i2c_init();
    bsp_display_start();
    bsp_display_backlight_on(); // Set display brightness to 100%

    // Initialize the camera
    const camera_config_t camera_config = BSP_CAMERA_DEFAULT_CONFIG;
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }
    sensor_t *s = esp_camera_sensor_get();
    s->set_vflip(s, 1);
    ESP_LOGI(TAG, "Camera Init done");

    // Create LVGL canvas for camera image
    bsp_display_lock(0);
    lv_obj_t *camera_canvas = lv_canvas_create(lv_scr_act());
    assert(camera_canvas);
    lv_obj_center(camera_canvas);
    bsp_display_unlock();

    camera_fb_t *pic;
    while (1) {
        pic = esp_camera_fb_get();
        if (pic) {
            bsp_display_lock(0);
            lv_canvas_set_buffer(camera_canvas, pic->buf, pic->width, pic->height, LV_IMG_CF_TRUE_COLOR);
            bsp_display_unlock();
            esp_camera_fb_return(pic);
        } else {
            ESP_LOGE(TAG, "Get frame failed");
        }
    }
}
