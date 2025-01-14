/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "hal/axi_dma_ll.h"
#include "hal/axi_icm_ll.h"
#include "soc/mipi_dsi_bridge_struct.h"

#include "lv_demos.h"
#include "bsp/esp-bsp.h"

static char *TAG = "app_main";

#define LOG_MEM_INFO    (0)

static void esp_dsi_avoid_blue_screen(void)
{
    axi_icm_ll_set_dw_gdma_qos_arbiter_prio(0, 9, 10);
    axi_icm_ll_set_dw_gdma_qos_arbiter_prio(1, 9, 10);
    axi_icm_ll_set_h264_dma_qos_arbiter_prio(1, 5, 5);
    axi_icm_ll_set_h264_dma_qos_arbiter_prio(0, 5, 5);
    axi_icm_ll_set_dma2d_qos_arbiter_prio(4, 4);
    axi_icm_ll_set_cache_qos_arbiter_prio(4, 4);

    int peak_level = 5; // 4: 800MB/s, 5: 400MB, 6: 200MB
    axi_icm_ll_set_qos_burstiness(AXI_ICM_MASTER_DMA2D, 256);
    axi_icm_ll_set_qos_peak_transaction_rate(AXI_ICM_MASTER_DMA2D, peak_level, peak_level + 1);
}

void app_main(void)
{
#if CONFIG_EXAMPLE_AVOID_BLUE_SCREEN && CONFIG_LV_COLOR_DEPTH_24
    esp_dsi_avoid_blue_screen();
#endif

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
    // lv_demo_widgets();      /* A widgets example */
    // lv_demo_music();        /* A modern, smartphone-like music player demo. */
    // lv_demo_stress();       /* A stress test for LVGL. */
    lv_demo_benchmark();    /* A demo to measure the performance of LVGL or to compare different settings. */
    bsp_display_unlock();

#if LOG_MEM_INFO
    static char buffer[128];    /* Make sure buffer is enough for `sprintf` */
    while (1) {
        sprintf(buffer, "   Biggest /     Free /    Total\n"
                "\t  SRAM : [%8d / %8d / %8d]\n"
                "\t PSRAM : [%8d / %8d / %8d]",
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_total_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
        ESP_LOGI("MEM", "%s", buffer);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
#endif
}
