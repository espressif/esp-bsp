/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief BSP USB HID Example
 * @details USB HID demo (keyboard, mouse, or gamepad visualization using LVGL)
 */

#include "esp_log.h"
#include "bsp/esp-bsp.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"

static const char *TAG = "example";

static lv_disp_t *display;
static lv_indev_t *kb_indev = NULL;
static lv_obj_t *result = NULL;
static lv_obj_t *ta_name = NULL;
static lv_obj_t *ta_email = NULL;
static lv_group_t *kb_group = NULL;

/*******************************************************************************
* Private functions
*******************************************************************************/
static void app_lvgl_btn_back_cb(lv_event_t *e)
{
    if (result) {
        lv_obj_clean(result);
        lv_obj_del(result);
        result = NULL;
    }
    if (kb_indev) {
        lv_indev_set_group(kb_indev, kb_group);
    }
}

static void app_lvgl_btn_ok_cb(lv_event_t *e)
{
    lv_obj_t *scr = lv_scr_act();
    const char *name = lv_textarea_get_text(ta_name);
    const char *email = lv_textarea_get_text(ta_email);

    result = lv_obj_create(scr);
    lv_obj_set_size(result, BSP_LCD_V_RES, BSP_LCD_H_RES);
    lv_obj_align(result, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_flex_flow(result, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(result, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t *lbl = lv_label_create(result);
    lv_label_set_text_fmt(lbl, "Your name: %s", name);
    lbl = lv_label_create(result);
    lv_label_set_text_fmt(lbl, "Your email: %s", email);

    lv_obj_t *back_btn = lv_btn_create(result);
    lbl = lv_label_create(back_btn);
    lv_label_set_text_static(lbl, "BACK");
    /* Button event */
    lv_obj_add_event_cb(back_btn, app_lvgl_btn_back_cb, LV_EVENT_CLICKED, NULL);

    if (kb_indev) {
        lv_group_t *group = lv_group_create();
        lv_group_add_obj(group, back_btn);
        lv_indev_set_group(kb_indev, group);
    }
}

static void app_lvgl_display(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_t *lbl;
    bsp_display_lock(0);

    lv_obj_t *col = lv_obj_create(scr);
    lv_obj_set_size(col, BSP_LCD_V_RES, BSP_LCD_H_RES);
    lv_obj_align(col, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_flex_flow(col, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(col, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lbl = lv_label_create(col);
    lv_label_set_text(lbl, "USB HID Example");

    lv_obj_t *cont_row = lv_obj_create(col);
    lv_obj_set_size(cont_row, BSP_LCD_V_RES - 10, 50);
    lv_obj_align(cont_row, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_flex_flow(cont_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_top(cont_row, 5, 0);
    lv_obj_set_style_pad_bottom(cont_row, 5, 0);
    lv_obj_set_style_pad_left(cont_row, 5, 0);
    lv_obj_set_style_pad_right(cont_row, 5, 0);
    lv_obj_set_flex_align(cont_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lbl = lv_label_create(cont_row);
    lv_label_set_text(lbl, "Name:");
    ta_name = lv_textarea_create(cont_row);
    lv_obj_set_width(ta_name, BSP_LCD_V_RES - 100);
    lv_textarea_set_one_line(ta_name, true);

    cont_row = lv_obj_create(col);
    lv_obj_set_size(cont_row, BSP_LCD_V_RES - 10, 50);
    lv_obj_align(cont_row, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_flex_flow(cont_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_top(cont_row, 5, 0);
    lv_obj_set_style_pad_bottom(cont_row, 5, 0);
    lv_obj_set_style_pad_left(cont_row, 5, 0);
    lv_obj_set_style_pad_right(cont_row, 5, 0);
    lv_obj_set_flex_align(cont_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lbl = lv_label_create(cont_row);
    lv_label_set_text(lbl, "E-mail:");
    ta_email = lv_textarea_create(cont_row);
    lv_obj_set_width(ta_email, BSP_LCD_V_RES - 100);
    lv_textarea_set_one_line(ta_email, true);

    /* Button OK */
    lv_obj_t *btn_ok = lv_btn_create(col);
    lbl = lv_label_create(btn_ok);
    lv_label_set_text_static(lbl, "OK");
    /* Button event */
    lv_obj_add_event_cb(btn_ok, app_lvgl_btn_ok_cb, LV_EVENT_CLICKED, NULL);

    if (kb_indev) {
        kb_group = lv_group_create();
        lv_group_add_obj(kb_group, ta_name);
        lv_group_add_obj(kb_group, ta_email);
        lv_group_add_obj(kb_group, btn_ok);
        lv_indev_set_group(kb_indev, kb_group);
        ESP_LOGI(TAG, "Keyboard input device group was set.");
    }

    bsp_display_unlock();
}

void app_main(void)
{
    /* Initialize USB */
    bsp_usb_host_start(BSP_USB_HOST_POWER_MODE_USB_DEV, true);

    /* Initialize display and LVGL */
    display = bsp_display_start();

    /* Set display brightness to 100% */
    bsp_display_backlight_on();

    const lvgl_port_hid_mouse_cfg_t mouse_cfg = {
        .disp = display,
        .sensitivity = 1,
    };
    lvgl_port_add_usb_hid_mouse_input(&mouse_cfg);

    const lvgl_port_hid_keyboard_cfg_t kb_cfg = {
        .disp = display,
    };
    kb_indev = lvgl_port_add_usb_hid_keyboard_input(&kb_cfg);

    /* Add and show objects on display */
    app_lvgl_display();

    ESP_LOGI(TAG, "Example initialization done.");

}
