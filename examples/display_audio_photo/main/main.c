/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief BSP Display, Audio and Photo Example
 * @details Complex demo: browse files from filesystem and play/display JPEG, WAV, or TXT files (LVGL)
 * @example https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_audio_photo-
 */

#include "esp_log.h"
#include "bsp/esp-bsp.h"
#include "app_disp_fs.h"

static const char *TAG = "example";

/*******************************************************************************
* Private functions
*******************************************************************************/

void app_main(void)
{
    /* Initialize and mount SPIFFS */
    bsp_spiffs_mount();

    /* Initialize I2C (for touch and audio) */
    bsp_i2c_init();

    /* Initialize display and LVGL */
    bsp_display_start();

    /* Set default display brightness */
    bsp_display_brightness_set(APP_DISP_DEFAULT_BRIGHTNESS);

    /* Add and show LVGL objects on display */
    app_disp_lvgl_show();

    /* Initialize SPI flash file system and show list of files on display */
    app_disp_fs_init();

    /* Initialize audio */
    app_audio_init();

    ESP_LOGI(TAG, "Example initialization done.");

}
