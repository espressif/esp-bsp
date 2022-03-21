/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "disp_example.h"
#include "lvgl.h"
#include "bsp/esp32_s2_kaluga_kit.h"

static lv_obj_t *recording_checkbox = NULL;
static lv_obj_t *playing_checkbox = NULL;
static lv_obj_t *volume_arc = NULL;

void disp_init(void)
{
    bsp_display_lock(0);

    /*Create a window*/
    lv_obj_t *win = lv_win_create(lv_scr_act(), 40);
    assert(win);
    lv_win_add_title(win, "ESP32-S2-Kaluga-Kit:\nBoard Support Package example");

    /* Volume arc */
    volume_arc = lv_arc_create(lv_scr_act());
    assert(volume_arc);
    lv_arc_set_end_angle(volume_arc, 200);
    lv_obj_set_size(volume_arc, 130, 130);
    lv_obj_center(volume_arc);
    lv_obj_t *volume_label = lv_label_create(volume_arc);
    assert(volume_label);
    lv_label_set_text_static(volume_label, "Volume");
    lv_obj_set_align(volume_label, LV_ALIGN_BOTTOM_MID);

    /* Checkboxes */
    recording_checkbox = lv_checkbox_create(win);
    assert(recording_checkbox);
    lv_checkbox_set_text_static(recording_checkbox, "Recording");
    playing_checkbox = lv_checkbox_create(win);
    assert(playing_checkbox);
    lv_checkbox_set_text_static(playing_checkbox, "Playing");

    bsp_display_unlock();
}

void disp_set_volume(int volume)
{
    assert(volume_arc);
    bsp_display_lock(0);
    lv_arc_set_value(volume_arc, volume);
    bsp_display_unlock();
}

void disp_set_playing(bool set)
{
    assert(playing_checkbox);
    bsp_display_lock(0);
    if (set) {
        lv_obj_add_state(playing_checkbox, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(playing_checkbox, LV_STATE_CHECKED);
    }
    bsp_display_unlock();
}

void disp_set_recording(bool set)
{
    assert(recording_checkbox);
    bsp_display_lock(0);
    if (set) {
        lv_obj_add_state(recording_checkbox, LV_STATE_CHECKED);
    } else {
        lv_obj_clear_state(recording_checkbox, LV_STATE_CHECKED);
    }
    bsp_display_unlock();
}
