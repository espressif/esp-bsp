/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief BSP Display SD card Example
 * @details Example of mounting an SD card using SD-MMC/SPI with display interaction. This example is also supported on boards without a display.
 * @example https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_sdcard
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp/esp-bsp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdmmc_cmd.h"

// Functionality which is only supported with display is hidden under BSP_CAPS_DISPLAY
#if(BSP_CAPS_DISPLAY)
#include "example_display.h"
#endif

#define EXAMPLE_READ_BUF_SIZE       64
// Choose with which interface the SD card should be mounted
#define EXAMPLE_SDMMC   1
#define EXAMPLE_SPI  0
#if EXAMPLE_SDMMC && EXAMPLE_SPI
// To change interfaces the SD card must be power cycled which is not supported by this example
#error "Only one interface is allowed!"
#endif

static const char *TAG = "example";

static bsp_sdcard_cfg_t cfg = {0};

#if EXAMPLE_SDMMC
static esp_err_t example_mount_sdmmc ()
{
    esp_err_t err;
    err = bsp_sdcard_sdmmc_mount(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount the SD card");
    }
    return err;
}
#endif

#if EXAMPLE_SPI
static esp_err_t example_mount_spi ()
{
    esp_err_t err;
    err = bsp_sdcard_sdspi_mount(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount the SD card");
    }
    return err;
}
#endif

static esp_err_t example_run_test()
{
    char write_buf[EXAMPLE_READ_BUF_SIZE] = {0};
    char read_buf[EXAMPLE_READ_BUF_SIZE] = {0};
    sdmmc_card_t *sdcard;

    sdcard = bsp_sdcard_get_handle();
    sdmmc_card_print_info(stdout, sdcard);

    // Write SD card specific info to the test.txt file
    FILE *fw = fopen(BSP_SD_MOUNT_POINT"/test.txt", "w");
    if (fw == NULL) {
        return ESP_FAIL;
    }
    snprintf(write_buf, EXAMPLE_READ_BUF_SIZE, "Card name %s and serial number %d\n", sdcard->cid.name, sdcard->cid.serial);
    fprintf(fw, "%s", write_buf);
    if (fclose(fw) == EOF) {
        return ESP_FAIL;
    }

    // Read SD card specific info from the test.txt file
    FILE *fr = fopen(BSP_SD_MOUNT_POINT"/test.txt", "r");
    if (fr == NULL) {
        return ESP_FAIL;
    }
    fgets(read_buf, EXAMPLE_READ_BUF_SIZE, fr);
    if (fclose(fr) == EOF) {
        return ESP_FAIL;
    }

    // Compare written and read strings
    if (!strncmp(write_buf, read_buf, EXAMPLE_READ_BUF_SIZE)) {
        ESP_LOGI(TAG, "Read data equals to written data.");
        ESP_LOGI(TAG, "Testing of SD card passed!");
    } else {
        ESP_LOGE(TAG, "Read data is empty or differs from written data");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err;

#if BSP_CAPS_DISPLAY
    // Initialize the display and show an initial screen
    ESP_ERROR_CHECK(example_display_init());
#endif

#if EXAMPLE_SDMMC
    err = example_mount_sdmmc();
#endif
#if EXAMPLE_SPI
    err = example_mount_spi();
#endif
    // Don't run the test when mounting fails
    if (err == ESP_OK) {
        err = example_run_test();
    }
    // End the example when either mounting or the test fails
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD card test failed with %s", esp_err_to_name(err));
#if(BSP_CAPS_DISPLAY)
        example_display_failed_test_screen(err);
#endif
        return;
    }

#if(BSP_CAPS_DISPLAY)
    example_display_main_screen(true);
    while (1) {
        example_display_poll();
    }
#endif
}
