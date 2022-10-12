/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_log.h"
#include "{__ESP_BOARD_INCLUDE__}"
#include "lvgl.h"
#include "ui/ui.h"


#define TAG "ESP-EXAMPLE"

/*******************************************************************************
* Private functions
*******************************************************************************/

// *INDENT-OFF*
void app_lvgl_display(void)
{{
    bsp_display_lock(0);

    ui_init();

    bsp_display_unlock();
}}

void app_main(void)
{{
    {__ESP_BOARD_I2C_INIT__}

    /* Initialize display and LVGL */
    bsp_display_start();

    /* Turn on display backlight */
    bsp_display_backlight_on();

    /* Add and show objects on display */
    app_lvgl_display();

    ESP_LOGI(TAG, "Example initialization done.");
}}
// *INDENT-ON*
