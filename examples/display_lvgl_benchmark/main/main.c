/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief BSP LVGL Benchmark Example
 * @details Run LVGL benchmark tests
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
#if defined(BSP_BOARD_ESP32_S3_LCD_EV_BOARD)
    /* Only for esp32_s3_lcd_ev_board */
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
    };
    cfg.lvgl_port_cfg.task_stack = 10000;
    bsp_display_start_with_config(&cfg);
#elif defined(BSP_BOARD_ESP32_S3_EYE)
    /* Only for esp32_s3_eye */
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = {
            .task_priority = CONFIG_BSP_DISPLAY_LVGL_TASK_PRIORITY,
            .task_stack = 10000,
            .task_affinity = 1,
            .timer_period_ms = CONFIG_BSP_DISPLAY_LVGL_TICK,
            .task_max_sleep_ms = CONFIG_BSP_DISPLAY_LVGL_MAX_SLEEP,
        },
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
#if (CONFIG_BSP_DISPLAY_LVGL_BUFFER_IN_PSRAM && CONFIG_SPIRAM)
            .buff_dma = false,
            .buff_spiram = true,
#else
            .buff_dma = true,
            .buff_spiram = false,
#endif
        }
    };
    bsp_display_start_with_config(&cfg);
#else
    bsp_display_start();
#endif

    /* Set display brightness to 100% */
    bsp_display_backlight_on();

    ESP_LOGI(TAG, "Display LVGL demo");
    bsp_display_lock(0);
    lv_demo_benchmark();    /* A demo to measure the performance of LVGL or to compare different settings. */
    bsp_display_unlock();
}
