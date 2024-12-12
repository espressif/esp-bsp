/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP BSP: M5Stack Core
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "bsp/config.h"
#include "bsp/display.h"

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "lvgl.h"
#include "esp_lvgl_port.h"
#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

#include "iot_button.h"

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

#define BSP_CAPS_DISPLAY        1
#define BSP_CAPS_TOUCH          0
#define BSP_CAPS_BUTTONS        1
#define BSP_CAPS_AUDIO          0
#define BSP_CAPS_AUDIO_SPEAKER  1
#define BSP_CAPS_AUDIO_MIC      0
#define BSP_CAPS_SDCARD         1
#define BSP_CAPS_IMU            0

/**************************************************************************************************
 *  M5Stack-Core pinout
 **************************************************************************************************/
/* I2C */
#define BSP_I2C_SCL           (GPIO_NUM_22)
#define BSP_I2C_SDA           (GPIO_NUM_21)

/* Audio */
#define BSP_SPEAKER_IO        (GPIO_NUM_25)

/* Display */
#define BSP_LCD_MOSI          (GPIO_NUM_23)
#define BSP_LCD_MISO          (GPIO_NUM_NC)
#define BSP_LCD_PCLK          (GPIO_NUM_18)
#define BSP_LCD_CS            (GPIO_NUM_14)
#define BSP_LCD_DC            (GPIO_NUM_27)
#define BSP_LCD_RST           (GPIO_NUM_33)
#define BSP_LCD_BACKLIGHT     (GPIO_NUM_32)

/* SD card */
#define BSP_SD_MOSI           (GPIO_NUM_23)
#define BSP_SD_MISO           (GPIO_NUM_19)
#define BSP_SD_SCK            (GPIO_NUM_18)
#define BSP_SD_CS             (GPIO_NUM_4)

/* BUTTON */
#define BSP_BUTTON_LEFT       (GPIO_NUM_39)
#define BSP_BUTTON_MIDDLE     (GPIO_NUM_38)
#define BSP_BUTTON_RIGHT      (GPIO_NUM_37)

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 *
 * Speaker interface
 *
 * M5Stack-Core has a simple speaker that can be controlled via GPIO.
 * The speaker must be initialized before use by calling bsp_speaker_init()
 **************************************************************************************************/
/**
 * @brief Initialize speaker GPIO
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   GPIO parameter error
 */
esp_err_t bsp_speaker_init(void);

/**************************************************************************************************
 *
 * I2C interface
 *
 * There are multiple devices connected to I2C peripheral:
 *  - IP5306
 **************************************************************************************************/
#define BSP_I2C_NUM     CONFIG_BSP_I2C_NUM

/**
 * @brief Init I2C driver
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *      - ESP_FAIL              I2C driver installation error
 *
 */
esp_err_t bsp_i2c_init(void);

/**
 * @brief Deinit I2C driver and free its resources
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   I2C parameter error
 *
 */
esp_err_t bsp_i2c_deinit(void);

/**************************************************************************************************
 *
 * SPIFFS
 *
 * After mounting the SPIFFS, it can be accessed with stdio functions ie.:
 * \code{.c}
 * FILE* f = fopen(BSP_SPIFFS_MOUNT_POINT"/hello.txt", "w");
 * fprintf(f, "Hello World!\n");
 * fclose(f);
 * \endcode
 **************************************************************************************************/
#define BSP_SPIFFS_MOUNT_POINT      CONFIG_BSP_SPIFFS_MOUNT_POINT

/**
 * @brief Mount SPIFFS to virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if esp_vfs_spiffs_register was already called
 *      - ESP_ERR_NO_MEM if memory can not be allocated
 *      - ESP_FAIL if partition can not be mounted
 *      - other error codes
 */
esp_err_t bsp_spiffs_mount(void);

/**
 * @brief Unmount SPIFFS from virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NOT_FOUND if the partition table does not contain SPIFFS partition with given label
 *      - ESP_ERR_INVALID_STATE if esp_vfs_spiffs_unregister was already called
 *      - ESP_ERR_NO_MEM if memory can not be allocated
 *      - ESP_FAIL if partition can not be mounted
 *      - other error codes
 */
esp_err_t bsp_spiffs_unmount(void);

/**************************************************************************************************
 *
 * SD card
 *
 * After mounting the SD card, it can be accessed with stdio functions ie.:
 * \code{.c}
 * FILE* f = fopen(BSP_SD_MOUNT_POINT"/hello.txt", "w");
 * fprintf(f, "Hello %s!\n", bsp_sdcard->cid.name);
 * fclose(f);
 * \endcode
 *
 * @attention IO2 is also routed to RGB LED and push button
 **************************************************************************************************/
#define BSP_SD_MOUNT_POINT      CONFIG_BSP_SD_MOUNT_POINT
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
 * LCD interface
 *
 * M5Stack-Core is shipped with 2.0inch ILI9341C display controller.
 * It features 16-bit colors, 320x240 resolution.
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling and LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * Display's backlight must be enabled explicitly by calling bsp_display_backlight_on()
 **************************************************************************************************/
#define BSP_LCD_PIXEL_CLOCK_HZ     (40 * 1000 * 1000)
#define BSP_LCD_SPI_NUM            (SPI2_HOST)

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#define BSP_LCD_DRAW_BUFF_SIZE     (BSP_LCD_H_RES * 50)
#define BSP_LCD_DRAW_BUFF_DOUBLE   (1)

/**
 * @brief BSP display configuration structure
 */
typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg;  /*!< LVGL port configuration */
    uint32_t        buffer_size;    /*!< Size of the buffer for the screen in pixels */
    bool            double_buffer;  /*!< True, if should be allocated two buffers */
    struct {
        unsigned int buff_dma: 1;    /*!< Allocated LVGL buffer will be DMA capable */
        unsigned int buff_spiram: 1; /*!< Allocated LVGL buffer will be in PSRAM */
    } flags;
} bsp_display_cfg_t;
/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @return Pointer to LVGL display or NULL when error occured
 */
lv_display_t *bsp_display_start(void);

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @param cfg display configuration
 *
 * @return Pointer to LVGL display or NULL when error occured
 */
lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);

/**
 * @brief Get pointer to input device (touch, buttons, ...)
 *
 * @note The LVGL input device is initialized in bsp_display_start() function.
 *
 * @return Pointer to LVGL input device or NULL when not initialized
 */
lv_indev_t *bsp_display_get_input_dev(void);

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
 * @brief Rotate screen
 *
 * Display must be already initialized by calling bsp_display_start()
 *
 * @param[in] disp Pointer to LVGL display
 * @param[in] rotation Angle of the display rotation
 */
void bsp_display_rotate(lv_display_t *disp, lv_display_rotation_t rotation);
#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

/**************************************************************************************************
 *
 * BSP Features
 *
 * This module provides an interface to enable and disable various features on the M5Stack-Core-2 board.
 * Supported features include the LCD display, touch screen, SD card, and speaker.
 *
 * The bsp_feature_enable function is used to enable or disable a specified feature.
 * It communicates with the Power Management Unit (PMU) via the I2C interface to control the power of each feature.
 *
 * Supported PMU models include AXP2101 and AXP192.
 * Depending on the configuration, the function selects the appropriate PMU registers and values to enable or disable the feature.
 *
 * Parameters:
 * - feature: The feature to enable or disable, of type bsp_feature_t enum.
 * - enable: A boolean value, true to enable the feature, false to disable it.
 *
 * Return value:
 * - esp_err_t: Error code indicating the result of the operation.
 *
 **************************************************************************************************/
typedef enum {
    BSP_FEATURE_LCD,
} bsp_feature_t;

esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable);

/**************************************************************************************************
 *
 * Button interface
 *
 * M5Stack-Core has three buttons:
 * - Left button (GPIO39): Used for navigation (previous)
 * - Middle button (GPIO38): Used for selection (enter)
 * - Right button (GPIO37): Used for navigation (next)
 *
 * The buttons can be used with LVGL for UI navigation
 **************************************************************************************************/
/**
 * @brief Create button objects
 *
 * @param btn_array Output array of button handles
 * @param btn_cnt Output number of created buttons
 * @param btn_array_size Size of output array
 * @return
 *      - ESP_OK Success
 *      - ESP_ERR_INVALID_ARG Parameter error
 *      - ESP_FAIL Create button failed
 */
esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size);

#ifdef __cplusplus
}
#endif
