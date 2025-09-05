/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP BSP: DevKit BSP
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/sdmmc_host.h"
#include "iot_button.h"
#include "led_indicator.h"

/**************************************************************************************************
 *  BSP Board Name
 **************************************************************************************************/

/** @defgroup boardname Board Name
 *  @brief BSP Board Name
 *  @{
 */
#define BSP_BOARD_DEVKIT
/** @} */ // end of boardname

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

/** @defgroup capabilities Capabilities
 *  @brief BSP Capabilities
 *  @{
 */
#if CONFIG_BSP_BUTTONS_NUM > 0
#define BSP_CAPS_BUTTONS    1
#endif
#if CONFIG_BSP_LEDS_NUM > 0
#define BSP_CAPS_LED       1
#endif
#define BSP_CAPS_AUDIO          0
#define BSP_CAPS_AUDIO_SPEAKER  0
#define BSP_CAPS_AUDIO_MIC      0
#define BSP_CAPS_SDCARD         0
#define BSP_CAPS_IMU            0
/** @} */ // end of capabilities

/**************************************************************************************************
 *  Pinout
 **************************************************************************************************/

/** @defgroup g01_i2c I2C
 *  @brief I2C BSP API
 *  @{
 */
#define BSP_I2C_SCL         (CONFIG_BSP_I2C_GPIO_SCL)
#define BSP_I2C_SDA         (CONFIG_BSP_I2C_GPIO_SDA)
/** @} */ // end of audio

/** @defgroup g02_storage SD Card and SPIFFS
 *  @brief SPIFFS and SD card BSP API
 *  @{
 */
/* SD card */
#define BSP_SD_CMD          (CONFIG_BSP_SD_CMD)
#define BSP_SD_CLK          (CONFIG_BSP_SD_CLK)
#define BSP_SD_D0           (CONFIG_BSP_SD_D0)
#define BSP_SD_D1           (CONFIG_BSP_SD_D1)
#define BSP_SD_D2           (CONFIG_BSP_SD_D2)
#define BSP_SD_D3           (CONFIG_BSP_SD_D3)
/** @} */ // end of storage

/** @defgroup g05_buttons Buttons
 *  @brief Buttons BSP API
 *  @{
 */
#define BSP_BUTTON_1_IO     (CONFIG_BSP_BUTTON_1_GPIO)
#define BSP_BUTTON_2_IO     (CONFIG_BSP_BUTTON_2_GPIO)
#define BSP_BUTTON_3_IO     (CONFIG_BSP_BUTTON_3_GPIO)
#define BSP_BUTTON_4_IO     (CONFIG_BSP_BUTTON_4_GPIO)
#define BSP_BUTTON_5_IO     (CONFIG_BSP_BUTTON_5_GPIO)
/** @} */ // end of buttons

/** @defgroup g06_led Leds
 *  @brief Leds BSP API
 *  @{
 */
#define BSP_LED_1_IO        (CONFIG_BSP_LED_1_GPIO)
#define BSP_LED_2_IO        (CONFIG_BSP_LED_2_GPIO)
#define BSP_LED_3_IO        (CONFIG_BSP_LED_3_GPIO)
#define BSP_LED_4_IO        (CONFIG_BSP_LED_4_GPIO)
#define BSP_LED_5_IO        (CONFIG_BSP_LED_5_GPIO)
/** @} */ // end of leds

/** \addtogroup g05_buttons
 *  @brief BSP Buttons
 *  @{
 */
typedef enum {
#if CONFIG_BSP_BUTTONS_NUM > 0
    BSP_BUTTON_1,
#endif
#if CONFIG_BSP_BUTTONS_NUM > 1
    BSP_BUTTON_2,
#endif
#if CONFIG_BSP_BUTTONS_NUM > 2
    BSP_BUTTON_3,
#endif
#if CONFIG_BSP_BUTTONS_NUM > 3
    BSP_BUTTON_4,
#endif
#if CONFIG_BSP_BUTTONS_NUM > 4
    BSP_BUTTON_5,
#endif
    BSP_BUTTON_NUM
} bsp_button_t;
/** @} */ // end of buttons

/** \addtogroup g06_led
 *  @brief BSP Leds
 *  @{
 */
typedef enum {
#if CONFIG_BSP_LEDS_NUM > 0
    BSP_LED_1,
#endif
#if CONFIG_BSP_LEDS_NUM > 1
    BSP_LED_2,
#endif
#if CONFIG_BSP_LEDS_NUM > 2
    BSP_LED_3,
#endif
#if CONFIG_BSP_LEDS_NUM > 3
    BSP_LED_4,
#endif
#if CONFIG_BSP_LEDS_NUM > 4
    BSP_LED_5,
#endif
    BSP_LED_NUM
} bsp_led_t;

/* Default LED effects */
enum {
    BSP_LED_ON,
    BSP_LED_OFF,
    BSP_LED_BLINK_FAST,
    BSP_LED_BLINK_SLOW,
    BSP_LED_BREATHE_FAST,
    BSP_LED_BREATHE_SLOW,
    BSP_LED_MAX,
};
/** @} */ // end of leds

#ifdef __cplusplus
extern "C" {
#endif

/** \addtogroup g01_i2c
 *  @{
 */

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

/**
 * @brief Get I2C driver handle
 *
 * @return
 *      - I2C handle
 */
i2c_master_bus_handle_t bsp_i2c_get_handle(void);

/** @} */ // end of i2c

/** \addtogroup g02_storage
 *  @{
 */

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
 * uSD card
 *
 * After mounting the uSD card, it can be accessed with stdio functions ie.:
 * \code{.c}
 * FILE* f = fopen(BSP_MOUNT_POINT"/hello.txt", "w");
 * fprintf(f, "Hello %s!\n", bsp_sdcard->cid.name);
 * fclose(f);
 * \endcode
 **************************************************************************************************/
#define BSP_SD_MOUNT_POINT      CONFIG_BSP_SD_MOUNT_POINT

#if SOC_SDMMC_HOST_SUPPORTED
extern sdmmc_card_t *bsp_sdcard;
#endif

/**
 * @brief Mount microSD card to virtual file system
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if esp_vfs_fat_sdmmc_mount was already called
 *      - ESP_ERR_NO_MEM if memory cannot be allocated
 *      - ESP_FAIL if partition cannot be mounted
 *      - other error codes from SDMMC or SPI drivers, SDMMC protocol, or FATFS drivers
 */
esp_err_t bsp_sdcard_mount(void);

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

/** @} */ // end of storage

/** \addtogroup g05_buttons
 *  @{
 */

/**************************************************************************************************
 *
 * Button
 *
 **************************************************************************************************/

/**
 * @brief Initialize all buttons
 *
 * Returned button handlers must be used with espressif/button component API
 *
 * @param[out] btn_array      Output button array
 * @param[out] btn_cnt        Number of button handlers saved to btn_array, can be NULL
 * @param[in]  btn_array_size Size of output button array. Must be at least BSP_BUTTON_NUM
 * @return
 *     - ESP_OK               All buttons initialized
 *     - ESP_ERR_INVALID_ARG  btn_array is too small or NULL
 *     - ESP_FAIL             Underlaying iot_button_create failed
 */
esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size);

/** @} */ // end of buttons

/** \addtogroup g06_led
 *  @{
 */

/**************************************************************************************************
 *
 * LEDs
 *
 **************************************************************************************************/

/**
 * @brief Initialize all LEDs
 *
 * @param[out] led_array      Output LED array
 * @param[out] led_cnt        Number of LED handlers saved to led_array, can be NULL
 * @param[in]  led_array_size Size of output LED array. Must be at least BSP_LED_NUM
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_led_indicator_create(led_indicator_handle_t led_array[], int *led_cnt, int led_array_size);

/**
 * @brief Turn LED on/off
 *
 * @param handle led handle
 * @param on Switch LED on/off
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_led_set(led_indicator_handle_t handle, const bool on);

/**
 * @brief Set LED temperature
 *
 * @param handle led handle
 * @param temperature Color temperature of LED
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_led_set_temperature(led_indicator_handle_t handle, const uint16_t temperature);

/** @} */ // end of leds

#ifdef __cplusplus
}
#endif
