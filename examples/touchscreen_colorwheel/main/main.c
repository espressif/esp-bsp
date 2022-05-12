/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_log.h"
#include "bsp/esp-box.h"
#include "lvgl.h"

#define TAG "ESP-BOX"
#define APP_DISP_DEFAULT_BRIGHTNESS 50

static lv_obj_t *main_screen = NULL;
static const lv_color_t default_color = {
    .full = 0xE7F9
};
/*******************************************************************************
* Private functions
*******************************************************************************/

static void cw_event_cb(lv_event_t *e)
{
    lv_obj_t *cw = lv_event_get_target(e);
    assert(cw != NULL);

    lv_color_t color = lv_colorwheel_get_rgb(cw);
    ESP_LOGI(TAG, "Selected color in RGB565: 0x%04x", color.full);

    if (main_screen) {
        /* Set background color */
        lv_obj_set_style_bg_color(main_screen, color, 0);
    }
}

void app_lvgl_display(void)
{
    bsp_display_lock(0);

    main_screen = lv_scr_act();
    lv_obj_set_style_bg_color(main_screen, default_color, 0);

    /* Block */
    lv_obj_t *cont = lv_obj_create(main_screen);
    lv_obj_set_style_bg_color(cont, lv_palette_darken(LV_PALETTE_GREY, 3), 0);
    lv_obj_set_size(cont, 180, 180);
    lv_obj_align(cont, LV_ALIGN_BOTTOM_MID, 0, -10);

    /* Color wheel */
    lv_obj_t *cw = lv_colorwheel_create(cont, true);
    lv_colorwheel_set_rgb(cw, default_color);
    lv_obj_add_event_cb(cw, cw_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_set_size(cw, 120, 120);
    lv_obj_center(cw);

    /* Label */
    lv_obj_t *lbl = lv_label_create(main_screen);
    lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl, BSP_LCD_H_RES);
    lv_label_set_text_static(lbl, "A long press will show the next color mode. Double click to reset.");
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lbl, LV_ALIGN_TOP_MID, 0, 0);

    bsp_display_unlock();
}

void app_main(void)
{
    /* Initialize I2C (for touch and audio) */
    bsp_i2c_init();

    /* Initialize display and LVGL */
    bsp_display_start();

    /* Set default display brightness */
    bsp_display_brightness_set(APP_DISP_DEFAULT_BRIGHTNESS);

    /* Add and show objects on display */
    app_lvgl_display();

    ESP_LOGI(TAG, "Example initialization done.");

}
