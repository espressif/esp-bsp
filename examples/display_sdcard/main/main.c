/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief BSP Display SD card Example
 * @details Example of mounting an SD card using SD-MMC/SPI with display interaction. This example is also supported on boards without a display.
 * @example TODO: https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_camera-
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bsp/esp-bsp.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdmmc_cmd.h"

#if(BSP_CAPS_DISPLAY)
#include "display.h"
#endif

// Choose with which interface you want to mount the SD card
#define EXAMPLE_SDMMC   1
#define EXAMPLE_SPI  0
#if EXAMPLE_SDMMC & EXAMPLE_SPI
#error "Only one interface is allowed!"
#endif

#define EXAMPLE_READ_BUF_SIZE       64

static const char *TAG = "example";

static bsp_sdcard_cfg_t cfg = {0};

#if EXAMPLE_SDMMC
esp_err_t example_mount_sdmmc ()
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
esp_err_t example_mount_spi ()
{
    esp_err_t err;
    err = bsp_sdcard_sdspi_mount(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount the SD card");
    }
    return err;
}
#endif

esp_err_t example_run_test()
{
    esp_err_t err;
    char write_buf[EXAMPLE_READ_BUF_SIZE] = {0};
    char read_buf[EXAMPLE_READ_BUF_SIZE] = {0};
    sdmmc_card_t *sdcard;

    sdcard = bsp_sdcard_get_handle();
    sdmmc_card_print_info(stdout, sdcard);

    FILE *fw = fopen(BSP_SD_MOUNT_POINT"/test.txt", "w");
    if (fw == NULL) {
        return ESP_FAIL;
    }
    snprintf(write_buf, EXAMPLE_READ_BUF_SIZE, "Card name %s and serial number %d\n", sdcard->cid.name, sdcard->cid.serial);
    fprintf(fw, "%s", write_buf);
    err = fclose(fw);
    if (err != ESP_OK) {
        return err;
    }

    FILE *fr = fopen(BSP_SD_MOUNT_POINT"/test.txt", "r");
    if (fr == NULL) {
        return ESP_FAIL;
    }
    fgets(read_buf, EXAMPLE_READ_BUF_SIZE, fr);
    err = fclose(fr);
    if (err != ESP_OK) {
        return err;
    }

    if (!strncmp(write_buf, read_buf, EXAMPLE_READ_BUF_SIZE)) {
        ESP_LOGW(TAG, "Read data equals to written data.");
        ESP_LOGI(TAG, "Testing of SD card passed!");
#if(BSP_CAPS_DISPLAY)
        display_main();
#endif
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
    display_init();
#endif

#if EXAMPLE_SDMMC
    err = example_mount_sdmmc();
#endif
#if EXAMPLE_SPI
    err = example_mount_spi();
#endif
    if (err == ESP_OK) {
        err = example_run_test();
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SD card test failed with %s", esp_err_to_name(err));
#if(BSP_CAPS_DISPLAY)
        display_failed_test(err);
#endif
        return;
    }

#if(BSP_CAPS_DISPLAY)
    display_main();
    while (1) {
        // Delay to reset the watchdog
        vTaskDelay(pdMS_TO_TICKS(10));
        display_dispatch();
    }
#endif
}
