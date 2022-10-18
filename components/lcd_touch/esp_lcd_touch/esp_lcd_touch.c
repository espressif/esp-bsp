/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_touch.h"

/*******************************************************************************
* Function definitions
*******************************************************************************/

/*******************************************************************************
* Local variables
*******************************************************************************/

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t esp_lcd_touch_read_data(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);
    assert(tp->read_data != NULL);

    return tp->read_data(tp);
}

bool esp_lcd_touch_get_coordinates(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    bool touched = false;

    assert(tp != NULL);
    assert(x != NULL);
    assert(y != NULL);
    assert(tp->get_xy != NULL);

    touched = tp->get_xy(tp, x, y, strength, point_num, max_point_num);
    if (!touched) {
        return false;
    }

    /* Process coordinates by user */
    if (tp->config.process_coordinates != NULL) {
        tp->config.process_coordinates(tp, x, y, strength, point_num, max_point_num);
    }

    /* Software coordinates adjustment needed */
    bool sw_adj_needed = ((tp->config.flags.mirror_x && (tp->set_mirror_x == NULL)) ||
                          (tp->config.flags.mirror_y && (tp->set_mirror_y == NULL)) ||
                          (tp->config.flags.swap_xy && (tp->set_swap_xy == NULL)));

    /* Adjust all coordinates */
    for (int i = 0; (sw_adj_needed && i < *point_num); i++) {

        /*  Mirror X coordinates (if not supported by HW) */
        if (tp->config.flags.mirror_x && tp->set_mirror_x == NULL) {
            x[i] = tp->config.x_max - x[i];
        }

        /*  Mirror Y coordinates (if not supported by HW) */
        if (tp->config.flags.mirror_y && tp->set_mirror_y == NULL) {
            y[i] = tp->config.y_max - y[i];
        }

        /* Swap X and Y coordinates (if not supported by HW) */
        if (tp->config.flags.swap_xy && tp->set_swap_xy == NULL) {
            uint16_t tmp = x[i];
            x[i] = y[i];
            y[i] = tmp;
        }
    }

    return touched;
}

#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
esp_err_t esp_lcd_touch_get_button_state(esp_lcd_touch_handle_t tp, uint8_t n, uint8_t *state)
{
    assert(tp != NULL);
    assert(state != NULL);

    *state = 0;

    if (tp->get_button_state) {
        return tp->get_button_state(tp, n, state);
    } else {
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}
#endif

esp_err_t esp_lcd_touch_set_swap_xy(esp_lcd_touch_handle_t tp, bool swap)
{
    assert(tp != NULL);

    tp->config.flags.swap_xy = swap;

    /* Is swap supported by HW? */
    if (tp->set_swap_xy) {
        return tp->set_swap_xy(tp, swap);
    }

    return ESP_OK;
}

esp_err_t esp_lcd_touch_get_swap_xy(esp_lcd_touch_handle_t tp, bool *swap)
{
    assert(tp != NULL);
    assert(swap != NULL);

    /* Is swap supported by HW? */
    if (tp->get_swap_xy) {
        return tp->get_swap_xy(tp, swap);
    } else {
        *swap = tp->config.flags.swap_xy;
    }

    return ESP_OK;
}

esp_err_t esp_lcd_touch_set_mirror_x(esp_lcd_touch_handle_t tp, bool mirror)
{
    assert(tp != NULL);

    tp->config.flags.mirror_x = mirror;

    /* Is mirror supported by HW? */
    if (tp->set_mirror_x) {
        return tp->set_mirror_x(tp, mirror);
    }

    return ESP_OK;
}

esp_err_t esp_lcd_touch_get_mirror_x(esp_lcd_touch_handle_t tp, bool *mirror)
{
    assert(tp != NULL);
    assert(mirror != NULL);

    /* Is swap supported by HW? */
    if (tp->get_mirror_x) {
        return tp->get_mirror_x(tp, mirror);
    } else {
        *mirror = tp->config.flags.mirror_x;
    }

    return ESP_OK;
}

esp_err_t esp_lcd_touch_set_mirror_y(esp_lcd_touch_handle_t tp, bool mirror)
{
    assert(tp != NULL);

    tp->config.flags.mirror_y = mirror;

    /* Is mirror supported by HW? */
    if (tp->set_mirror_y) {
        return tp->set_mirror_y(tp, mirror);
    }

    return ESP_OK;
}

esp_err_t esp_lcd_touch_get_mirror_y(esp_lcd_touch_handle_t tp, bool *mirror)
{
    assert(tp != NULL);
    assert(mirror != NULL);

    /* Is swap supported by HW? */
    if (tp->get_mirror_y) {
        return tp->get_mirror_y(tp, mirror);
    } else {
        *mirror = tp->config.flags.mirror_y;
    }

    return ESP_OK;
}

esp_err_t esp_lcd_touch_del(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    if (tp->del != NULL) {
        return tp->del(tp);
    }

    return ESP_OK;
}
