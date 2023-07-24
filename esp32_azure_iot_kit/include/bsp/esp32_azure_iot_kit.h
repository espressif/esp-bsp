/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP BSP: Azure IoT kit
 */

#pragma once

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 *
 * I2C interface
 *
 * After initialization of I2C, use BSP_I2C_NUM macro when creating I2C devices drivers ie.:
 * \code{.c}
 * hts221_dev = hts221_create(BSP_I2C_NUM);
 * \endcode
 **************************************************************************************************/
#define BSP_I2C_SCL_IO  GPIO_NUM_26
#define BSP_I2C_SDA_IO  GPIO_NUM_25
#define BSP_I2C_NUM     CONFIG_BSP_I2C_NUM

typedef enum {
    I2C_CLK_100KHZ = 0,
    I2C_CLK_400KHZ = 1,
    I2C_CLK_600KHZ = 2
} bsp_i2c_clk_speed_t;

/**
 * @brief Init I2C driver
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Driver install error
 */
esp_err_t bsp_i2c_init(void);

/**
 * @brief Deinit I2C driver and free its resources
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_i2c_deinit(void);

/**
 * @brief Set I2C clk after initialization of the I2C driver
 *
 * @param i2c_clk I2C clk frequency
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_i2c_set_clk_speed(bsp_i2c_clk_speed_t i2c_clk);

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
typedef enum bsp_led_t {
    BSP_LED_WIFI = GPIO_NUM_32,
    BSP_LED_AZURE = GPIO_NUM_33
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
 * @param led_io Azure or Wifi LED
 * @param on Switch LED on/off
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_led_set(const bsp_led_t led_io, const bool on);

/**************************************************************************************************
 *
 * Buzzer
 *
 **************************************************************************************************/
#define BSP_BUZZER_IO    GPIO_NUM_27

/**
 * @brief Init buzzer
 *
 * This function configures LEDC peripheral for PWM generation that is fed to buzzer.
 *
 * @return esp_err_t
 */
esp_err_t bsp_buzzer_init(void);

/**
 * @brief Enable/disable buzzer
 *
 * @param[in] on Switch buzzer on/off
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_buzzer_set(const bool on);

/**************************************************************************************************
 *
 * Button
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

#ifdef __cplusplus
}
#endif
