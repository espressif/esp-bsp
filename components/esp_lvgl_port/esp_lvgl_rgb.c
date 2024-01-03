/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lvgl_port.h"

static const char *TAG = "LV_RGB";

#if CONFIG_IDF_TARGET_ESP32S3

typedef struct {
    esp_lcd_panel_handle_t rgb_handle;
    uint32_t rgb_ref_period;
} lv_manual_refresh;

static TaskHandle_t lcd_task_handle = NULL;

IRAM_ATTR static bool lvgl_port_rgb_on_vsync_callback(esp_lcd_panel_handle_t panel_handle, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    BaseType_t need_yield = pdFALSE;

    lv_disp_drv_t *disp_drv = (lv_disp_drv_t *)user_ctx;
    assert(disp_drv != NULL);
    lvgl_port_display_ctx_t *disp_ctx = disp_drv->user_data;

    if (disp_ctx->lcd_manual_mode) {
        xTaskNotifyFromISR(lcd_task_handle, ULONG_MAX, eNoAction, &need_yield);
    }

    if (disp_ctx->lcd_transdone_cb) {
        need_yield = disp_ctx->lcd_transdone_cb(panel_handle, user_ctx);
    }

    return (need_yield == pdTRUE);
}

esp_err_t lvgl_rgb_register_event_callbacks(lvgl_port_display_ctx_t *disp_ctx, const lvgl_port_display_cfg_t *disp_cfg)
{
    esp_err_t ret;
    /* Register done callback */
    const esp_lcd_rgb_panel_event_callbacks_t vsync_cbs = {
        .on_vsync = lvgl_port_rgb_on_vsync_callback,
    };

    const esp_lcd_rgb_panel_event_callbacks_t bb_cbs = {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 2) && CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE
        .on_bounce_frame_finish = lvgl_port_rgb_on_vsync_callback,
#endif
    };

    if (disp_cfg->trans_mode.bb_mode && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 2))) {
        esp_lcd_rgb_panel_register_event_callbacks(disp_ctx->panel_handle, &bb_cbs, &disp_ctx->disp_drv);
    } else {
        esp_lcd_rgb_panel_register_event_callbacks(disp_ctx->panel_handle, &vsync_cbs, &disp_ctx->disp_drv);
    }

    return ESP_OK;
}

static void lvgl_rgb_manual_task(void *arg)
{
    TickType_t tick;
    lv_manual_refresh *refresh = (lv_manual_refresh *)arg;

    ESP_LOGI(TAG, "Starting LCD refresh task");

    for (;;) {
        esp_lcd_rgb_panel_refresh(refresh->rgb_handle);
        tick = xTaskGetTickCount();
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        vTaskDelayUntil(&tick, pdMS_TO_TICKS(refresh->rgb_ref_period));
    }
}

esp_err_t lvgl_rgb_create_manual_task(const lvgl_port_display_cfg_t *disp_cfg)
{
    static lv_manual_refresh refresh;

    refresh.rgb_handle = disp_cfg->panel_handle;
    refresh.rgb_ref_period = disp_cfg->trans_mode.flags.ref_period;

    BaseType_t ret = xTaskCreate(lvgl_rgb_manual_task, "LCD", 2048, &refresh, disp_cfg->trans_mode.flags.ref_priority, &lcd_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create LCD task");
        return ESP_FAIL;
    }

    return ESP_OK;
}

#else

esp_err_t lvgl_rgb_register_event_callbacks(lvgl_port_display_ctx_t *disp_ctx, const lvgl_port_display_cfg_t *disp_cfg)
{
    ESP_LOGE(TAG, "RGB mode not supported!");
}

esp_err_t lvgl_rgb_create_manual_task(const lvgl_port_display_cfg_t *disp_cfg)
{
    ESP_LOGE(TAG, "RGB mode not supported!");
}

#endif
