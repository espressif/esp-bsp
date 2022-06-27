/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "sdkconfig.h"
#include "lvgl.h"

/**************************************************************************************************
 * ESP-WROVER-KIT pinout
 *
 * @attention IO0 and IO2 are routed to switch button and red/green LEDs, so RGB LEDs and button CANNOT be used at the same time!
 * @attention IO2 is routed to uSD card DATA0 signal and green LED
 **************************************************************************************************/

/* RGB LED */
typedef enum bsp_led_t {
    BSP_LED_RED = GPIO_NUM_0,
    BSP_LED_GREEN = GPIO_NUM_2,
    BSP_LED_BLUE = GPIO_NUM_4
} bsp_led_t;

/* Display */
#define BSP_LCD_SPI_MOSI      (GPIO_NUM_23)
#define BSP_LCD_SPI_MISO      (GPIO_NUM_25)
#define BSP_LCD_SPI_CLK       (GPIO_NUM_19)
#define BSP_LCD_SPI_CS        (GPIO_NUM_22)
#define BSP_LCD_DC            (GPIO_NUM_21)
#define BSP_LCD_RST           (GPIO_NUM_18)
#define BSP_LCD_BACKLIGHT     (GPIO_NUM_5)

/* Button */
typedef enum {
    BSP_BUTTON_BOOT = GPIO_NUM_0
} bsp_button_t;

/* uSD card */
#define BSP_SD_D0             (GPIO_NUM_2)
#define BSP_SD_D1             (GPIO_NUM_4)
#define BSP_SD_D2             (GPIO_NUM_12)
#define BSP_SD_D3             (GPIO_NUM_13)
#define BSP_SD_CMD            (GPIO_NUM_15)
#define BSP_SD_CLK            (GPIO_NUM_14)
#define BSP_SD_DET            (GPIO_NUM_21)

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
 **************************************************************************************************/

/**
 * @brief Set LED's GPIOs as output push-pull
 */
void bsp_leds_init(void);

/**
 * @brief Turn LED on/off
 *
 * @param led_io LED io
 * @param on Switch LED on/off
 */
void bsp_led_set(const bsp_led_t led_io, const bool on);

/**************************************************************************************************
 *
 * Button
 *
 **************************************************************************************************/

/**
 * @brief Set button's GPIO as input
 *
 * @param[in] btn Button to be initialized
 */
void bsp_button_init(const bsp_button_t btn);

/**
 * @brief Get button's state
 *
 * @param[in] btn Button to read
 * @return true  Button pressed
 * @return false Button released
 */
bool bsp_button_get(const bsp_button_t btn);

/**************************************************************************************************
 *
 * LCD interface
 *
 * ESP32-WROVER-KIT is shipped with 3.2inch ST7789 or ILI9341 display controller. It features 16-bit colors and 320x240 resolution.
 * If your colours on the display are distorted, try changing the display type in menuconfig.
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling and LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * Display's backlight must be enabled explicitly by calling bsp_display_backlight_on()
 **************************************************************************************************/
#define BSP_LCD_H_RES               (240)
#define BSP_LCD_V_RES               (320)
#define BSP_LCD_PIXEL_CLOCK_HZ      (40 * 1000 * 1000)
#define BSP_LCD_SPI_NUM             (SPI2_HOST)

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 *
 * @return Pointer to LVGL display
 */
lv_disp_t *bsp_display_start(void);

/**
 * @brief Take LVGL mutex
 *
 * @param timeout_ms Timeout in [ms]. 0 will block indefinitely.
 * @return true  Mutex was taken
 * @return false Mutex was NOT taken
 */
bool bsp_display_lock(uint32_t timeout_ms);

/**
 * @brief Give LVGL mutex
 *
 */
void bsp_display_unlock(void);

/**
 * @brief Set display's brightness
 *
 * Brightness is controlled with PWM signal to a pin controling backlight.
 *
 * @param[in] brightness_percent Brightness in [%]
 */
void bsp_display_brightness_set(int brightness_percent);

/**
 * @brief Turn on display backlight
 *
 * Display must be already initialized by calling bsp_display_start()
 */
void bsp_display_backlight_on(void);

/**
 * @brief Turn off display backlight
 *
 * Display must be already initialized by calling bsp_display_start()
 */
void bsp_display_backlight_off(void);

/**
 * @brief Rotate screen
 *
 * Display must be already initialized by calling bsp_display_start()
 *
 * @param[in] disp Pointer to LVGL display
 * @param[in] rotation Angle of the display rotation
 */
void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation);

#ifdef __cplusplus
}
#endif
