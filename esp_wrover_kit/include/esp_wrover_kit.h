/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 *
 * uSD card
 *
 * After mounting the uSD card, it can be accessed with stdio functions ie.:
 * \code{.c}
 * FILE* f = fopen(BSP_MOUNT_POINT"/hello.txt", "w");
 * fprintf(f, "Hello %s!\n", bsp_sdcard->cid.name);
 * fclose(f);
 * \endcode
 *
 * @attention IO2 is also routed to RGB LED and push button
 **************************************************************************************************/
#define BSP_MOUNT_POINT      CONFIG_BSP_uSD_MOUNT_POINT
extern sdmmc_card_t *bsp_sdcard;

/**
 * @brief Mount microSD card to virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if esp_vfs_fat_sdmmc_mount was already called
 *      - ESP_ERR_NO_MEM if memory can not be allocated
 *      - ESP_FAIL if partition can not be mounted
 *      - other error codes from SDMMC or SPI drivers, SDMMC protocol, or FATFS drivers
 */
esp_err_t bsp_sdcard_mount(void);

/**
 * @brief Unmount micorSD card from virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NOT_FOUND if the partition table does not contain FATFS partition with given label
 *      - ESP_ERR_INVALID_STATE if esp_vfs_fat_spiflash_mount was already called
 *      - ESP_ERR_NO_MEM if memory can not be allocated
 *      - ESP_FAIL if partition can not be mounted
 *      - other error codes from wear levelling library, SPI flash driver, or FATFS drivers
 */
esp_err_t bsp_sdcard_unmount(void);

/**************************************************************************************************
 *
 * LEDs
 *
 * @attention IO0 and IO2 are routed to switch button, so RGB LEDs and button CANNOT be used at the same time!
 * @attention IO2 is also routed to uSD card DATA0 signal
 *
 **************************************************************************************************/
typedef enum bsp_led_t {
    BSP_LED_RED = GPIO_NUM_0,
    BSP_LED_GREEN = GPIO_NUM_2,
    BSP_LED_BLUE = GPIO_NUM_4
} bsp_led_t;

/**
 * @brief Set LED's GPIOs as output push-pull
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_leds_init(void);

/**
 * @brief Turn LED on/off
 *
 * @param led_io LED io
 * @param on Switch LED on/off
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_led_set(const bsp_led_t led_io, const bool on);

/**************************************************************************************************
 *
 * Button
 *
 * @attention IO0 and IO2 are routed to RGB LEDs, so RGB LEDs and button CANNOT be used at the same time!
 *
 **************************************************************************************************/
#define BSP_BUTTON_IO    GPIO_NUM_0

/**
 * @brief Set button's GPIO as input
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_button_init(void);

/**
 * @brief Get button's state
 *
 * @return true  Button pressed
 * @return false Button released
 */
bool bsp_button_get(void);

/**************************************************************************************************
 *
 * LCD interface
 *
 * ESP32-WROVER-KIT is shipped with 3.2inch ST7789 or ILI9341 display controller. It features 16-bit colors and 320x240 resolution.
 * If your colours on the display are distorted, try changing the display type in menuconfig.
 **************************************************************************************************/

/**
 * @brief Display LVGL task
 *
 * LVGL is used to interface the display. Start display task and wait until it finishes LVGL initialization:
 * \code{.c}
 * xTaskCreate(bsp_display_task, "bsp_display", 4096*2, xTaskGetCurrentTaskHandle(), 7, NULL);
 * ulTaskNotifyTake(pdFALSE, 1000);
 * \endcode
 *
 * @param[in] pvParameter [Optional] Calling task handler. bsp_display_task will notify it, when LVGL initialization is completed.
 */
void bsp_display_task(void *pvParameter);

#ifdef __cplusplus
}
#endif
