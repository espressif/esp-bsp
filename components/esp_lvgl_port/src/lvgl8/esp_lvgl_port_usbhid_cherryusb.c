/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "platform/lvgl/usbh_hid_lvgl.h"

/*******************************************************************************
* Public API functions
*******************************************************************************/

lv_indev_t *lvgl_port_add_usb_hid_mouse_input(const lvgl_port_hid_mouse_cfg_t *mouse_cfg)
{
    lv_indev_t *indev;
    assert(mouse_cfg);
    assert(mouse_cfg->disp);

    lvgl_port_lock(0);
    /* Register a mouse input device */
    indev = usbh_hid_lvgl_add_mouse(mouse_cfg->sensitivity);

    /* Set image of cursor */
    lv_obj_t *cursor = mouse_cfg->cursor_img;
    if (cursor) {
        lv_indev_set_cursor(indev, cursor);
    }
    lvgl_port_unlock();

    return indev;
}

lv_indev_t *lvgl_port_add_usb_hid_keyboard_input(const lvgl_port_hid_keyboard_cfg_t *keyboard_cfg)
{
    lv_indev_t *indev = NULL;

    lvgl_port_lock(0);
    indev = usbh_hid_lvgl_add_keyboard();
    lvgl_port_unlock();

    return indev;
}

esp_err_t lvgl_port_remove_usb_hid_input(lv_indev_t *hid)
{
    assert(hid);

    lvgl_port_lock(0);
    /* Remove input device driver */
    lv_indev_delete(hid);
    lvgl_port_unlock();

    return ESP_OK;
}
