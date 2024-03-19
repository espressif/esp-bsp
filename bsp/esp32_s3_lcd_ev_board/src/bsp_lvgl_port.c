/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_timer.h"
#include "esp_lcd_touch.h"

#include "bsp_err_check.h"
#include "bsp/display.h"
#include "bsp/esp32_s3_lcd_ev_board.h"

static const char *TAG = "bsp_lvgl_port";

lv_disp_t *bsp_display_lcd_init()
{
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;           // LCD panel handle

    bsp_display_config_t disp_config = { 0 };

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&disp_config, &panel_handle, &io_handle));

    // alloc draw buffers used by LVGL
    void *buf1 = NULL;
    void *buf2 = NULL;
    int buffer_size = 0;

    ESP_LOGD(TAG, "Malloc memory for LVGL buffer");
#ifndef CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
    // Normmaly, for RGB LCD, we just use one buffer for LVGL rendering
    buffer_size = BSP_LCD_H_RES * LVGL_BUFFER_HEIGHT;
    buf1 = heap_caps_malloc(buffer_size * sizeof(lv_color_t), LVGL_BUFFER_MALLOC);
    BSP_NULL_CHECK(buf1, NULL);
    ESP_LOGI(TAG, "LVGL buffer size: %dKB", buffer_size * sizeof(lv_color_t) / 1024);
#else
    // To avoid the tearing effect, we should use at least two frame buffers: one for LVGL rendering and another for RGB output
    buffer_size = BSP_LCD_H_RES * BSP_LCD_V_RES;
    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
#endif /* CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR */

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = buffer_size,
        .user_buf1 = buf1,
        .user_buf2 = buf2,

        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,

        .RGB = true,
        .flags = {
#if CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE
            .bb_mode = 1,
#endif
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
            .full_refresh = 1,
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
            .direct_mode = 1,
#endif
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}
