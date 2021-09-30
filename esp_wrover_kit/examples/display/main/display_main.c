/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "esp_wrover_kit.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "esp_log.h"

LV_IMG_DECLARE(esp_logo)
LV_IMG_DECLARE(esp_text)

void app_main(void)
{
    xTaskCreate(bsp_display_task, "bsp_display", 4096 * 2, xTaskGetCurrentTaskHandle(), 7, NULL);
    ulTaskNotifyTake(pdFALSE, 1000);

    ESP_LOGI("example", "Running UI demo");
    lv_obj_t *screen = lv_disp_get_scr_act(NULL);
    lv_obj_t *img_logo = lv_img_create(screen);
    lv_obj_t *img_text = lv_img_create(screen);

    lv_img_set_src(img_logo, &esp_logo);
    lv_img_set_src(img_text, &esp_text);
    lv_obj_center(img_logo);
    lv_obj_align_to(img_text, img_logo, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
}
