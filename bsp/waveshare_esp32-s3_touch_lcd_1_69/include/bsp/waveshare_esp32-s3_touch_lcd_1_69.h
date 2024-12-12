/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP BSP: Generic
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/sdmmc_host.h"

#if CONFIG_BSP_DISPLAY_ENABLED
#include "bsp/display.h"
#include "esp_lvgl_port.h"
#endif //CONFIG_BSP_DISPLAY_ENABLED

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

#if CONFIG_BSP_DISPLAY_ENABLED
#define BSP_CAPS_DISPLAY    1
#endif
#if CONFIG_BSP_TOUCH_ENABLED
#define BSP_CAPS_TOUCH      1
#endif

#define BSP_CAPS_AUDIO          0
#define BSP_CAPS_AUDIO_SPEAKER  0
#define BSP_CAPS_AUDIO_MIC      0
#define BSP_CAPS_SDCARD         0
#define BSP_CAPS_IMU            0

/**************************************************************************************************
 *  Pinout
 **************************************************************************************************/
/* I2C */
// #define BSP_I2C_SCL         (CONFIG_BSP_I2C_GPIO_SCL)
// #define BSP_I2C_SDA         (CONFIG_BSP_I2C_GPIO_SDA)

/* SD card */
// #define BSP_SD_CMD          (CONFIG_BSP_SD_CMD)
// #define BSP_SD_CLK          (CONFIG_BSP_SD_CLK)
// #define BSP_SD_D0           (CONFIG_BSP_SD_D0)
// #define BSP_SD_D1           (CONFIG_BSP_SD_D1)
// #define BSP_SD_D2           (CONFIG_BSP_SD_D2)
// #define BSP_SD_D3           (CONFIG_BSP_SD_D3)

/* Display */
// #define BSP_LCD_DATA0       (CONFIG_BSP_DISPLAY_MOSI_GPIO)
// #define BSP_LCD_PCLK        (CONFIG_BSP_DISPLAY_SCLK_GPIO)
// #define BSP_LCD_CS          (CONFIG_BSP_DISPLAY_CS_GPIO)
// #define BSP_LCD_DC          (CONFIG_BSP_DISPLAY_DC_GPIO)
// #define BSP_LCD_RST         (CONFIG_BSP_DISPLAY_RST_GPIO)
// #define BSP_LCD_BACKLIGHT   (CONFIG_BSP_DISPLAY_BACKLIGHT_GPIO)
// #define BSP_LCD_TOUCH_RST   (CONFIG_BSP_TOUCH_RST_GPIO)
// #define BSP_LCD_TOUCH_INT   (CONFIG_BSP_TOUCH_INT_GPIO)

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_BSP_DISPLAY_ENABLED
/**
 * @brief BSP display configuration structure
 *
 */
typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg;
} bsp_display_cfg_t;
#endif //CONFIG_BSP_DISPLAY_ENABLED

/**************************************************************************************************
 *
 * I2C interface
 *
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

/**
 * @brief Unmount microSD card from virtual file system
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


#if CONFIG_BSP_DISPLAY_ENABLED
/**************************************************************************************************
 *
 * LCD interface
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling and LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * Display's backlight must be enabled explicitly by calling bsp_display_backlight_on()
 **************************************************************************************************/
#define BSP_LCD_PIXEL_CLOCK_HZ     (CONFIG_BSP_DISPLAY_PIXEL_CLOCK * 1000 * 1000)
#define BSP_LCD_SPI_NUM            (SPI2_HOST)

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @return Pointer to LVGL display or NULL when error occurred
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
 * @return Pointer to LVGL display or NULL when error occurred
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
void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation);
#endif //CONFIG_BSP_DISPLAY_ENABLED

/**
 * @brief Init the Buzzer Line, and leave off.
 *
 * @return
 *      - ESP_OK                On success
 */
esp_err_t bsp_board_init(void);

/**
 * @brief Control Buzzer Line
 *
 * @param[in] on Line State
*/
void bsp_buzzer_control( bool on );

#ifdef __cplusplus
}
#endif
