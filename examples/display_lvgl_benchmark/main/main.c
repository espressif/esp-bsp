/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "lv_demos.h"
#include "bsp/esp-bsp.h"

static char *TAG = "app_main";

#define LOG_MEM_INFO    (0)

void app_main(void)
{
    /* Initialize display and LVGL */
    bsp_display_start();

#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
    ESP_LOGI(TAG, "Avoid lcd tearing effect");
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
    ESP_LOGI(TAG, "LVGL full-refresh");
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
    ESP_LOGI(TAG, "LVGL direct-mode");
#endif
#endif

    /* Set display brightness to 100% */
    bsp_display_backlight_on();

    ESP_LOGI(TAG, "Display LVGL demo");
    bsp_display_lock(0);
    lv_demo_benchmark();    /* A demo to measure the performance of LVGL or to compare different settings. */
    bsp_display_unlock();
}
