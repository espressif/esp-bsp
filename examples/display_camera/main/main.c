/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief BSP Camera Example
 * @details Stream camera output to display (LVGL)
 * @example https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_camera-
 */

#include <stdio.h>
#include <string.h>
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
    s->set_vflip(s, BSP_CAMERA_VFLIP);
    s->set_hmirror(s, BSP_CAMERA_HMIRROR);
    ESP_LOGI(TAG, "Camera Init done");

    uint32_t cam_buff_size = BSP_LCD_H_RES * BSP_LCD_V_RES * 2;
    uint8_t *cam_buff = heap_caps_malloc(cam_buff_size, MALLOC_CAP_SPIRAM);
    assert(cam_buff);

    // Create LVGL canvas for camera image
    bsp_display_lock(0);
    lv_obj_t *camera_canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(camera_canvas, cam_buff, BSP_LCD_H_RES, BSP_LCD_V_RES, LV_COLOR_FORMAT_RGB565);
    assert(camera_canvas);
    lv_obj_center(camera_canvas);
    bsp_display_unlock();

    camera_fb_t *pic;
    while (1) {
        pic = esp_camera_fb_get();
        if (pic) {
            bsp_display_lock(0);
            memcpy(cam_buff, pic->buf, cam_buff_size);
            esp_camera_fb_return(pic);
            if (BSP_LCD_BIGENDIAN) {
                /* Swap bytes in RGB565 */
                lv_draw_sw_rgb565_swap(cam_buff, cam_buff_size);
            }
            lv_obj_invalidate(camera_canvas);
            bsp_display_unlock();
        } else {
            ESP_LOGE(TAG, "Get frame failed");
        }
        vTaskDelay(1);
    }
}
