/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/sdmmc_host.h"
#include "soc/usb_pins.h"
#include "lvgl.h"

/**************************************************************************************************
 * ESP32-S3-USB-OTG pinout
 **************************************************************************************************/

/* LEDs */
// There is also red LED, that indicates battery charging
typedef enum bsp_led_t {
    BSP_LED_GREEN = GPIO_NUM_15,
    BSP_LED_YELLOW = GPIO_NUM_16
} bsp_led_t;

/* Display */
#define BSP_LCD_SPI_MOSI      (GPIO_NUM_7)
#define BSP_LCD_SPI_CLK       (GPIO_NUM_6)
#define BSP_LCD_SPI_CS        (GPIO_NUM_5)
#define BSP_LCD_DC            (GPIO_NUM_4)
#define BSP_LCD_RST           (GPIO_NUM_8)
#define BSP_LCD_BACKLIGHT     (GPIO_NUM_9)

/* uSD card */
#define BSP_SD_D0             (GPIO_NUM_37)
#define BSP_SD_D1             (GPIO_NUM_38)
#define BSP_SD_D2             (GPIO_NUM_33)
#define BSP_SD_D3             (GPIO_NUM_34)
#define BSP_SD_CMD            (GPIO_NUM_35)
#define BSP_SD_CLK            (GPIO_NUM_36)

/* Button */
// All signal are active low
typedef enum {
    BSP_BUTTON_OK = GPIO_NUM_0, // Also serves as BOOT select
    BSP_BUTTON_DW = GPIO_NUM_11,
    BSP_BUTTON_UP = GPIO_NUM_10,
    BSP_BUTTON_MENU = GPIO_NUM_14,
    BSP_USB_OVERCURRENT = GPIO_NUM_21 // This is not a real button, but the button API can be reused here
} bsp_button_t;

/* USB */
#define BSP_USB_POS           USBPHY_DP_NUM
#define BSP_USB_NEG           USBPHY_DM_NUM
#define BSP_USB_MODE_SEL      (GPIO_NUM_18) // Select Host (high level) or Device (low level, default) mode
#define BSP_USB_HOST_VOLTAGE  (GPIO_NUM_1)  // Voltage at this pin = (V_BUS / 3.7), ADC1 channel 0
#define BSP_USB_HOST_VOLTAGE_DIV (3.7f)
#define BSP_USB_LIMIT_EN      (GPIO_NUM_17) // Active high (pulled low)
#define BSP_USB_DEV_VBUS_EN   (GPIO_NUM_12) // Active high (pulled low)

/* Battery */
#define BSP_BATTERY_VOLTAGE   (GPIO_NUM_2)  // Voltage at this pin = (V_BAT / 2), ADC1 channel 1
#define BSP_BATTERY_VOLTAGE_DIV (2)
#define BSP_BATTERY_BOOST_EN  (GPIO_NUM_13) // 3.3->5V for USB device power from battery. Active high (pulled low)

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
 **************************************************************************************************/
#define BSP_MOUNT_POINT      CONFIG_BSP_uSD_MOUNT_POINT
extern sdmmc_card_t *bsp_sdcard;

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

/**************************************************************************************************
 *
 * LCD interface
 *
 * ESP32-S3-USB-OTG is shipped with 1.3inch ST7789 display controller.
 * It features 16-bit colors and 240x240 resolution.
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling and LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 **************************************************************************************************/
#define BSP_LCD_H_RES              (240)
#define BSP_LCD_V_RES              (240)
#define BSP_LCD_PIXEL_CLOCK_HZ     (40 * 1000 * 1000)
#define BSP_LCD_SPI_NUM            (SPI3_HOST)
#define BSP_LCD_BUFF_SIZE          (BSP_LCD_H_RES * 30) // Frame buffer size in pixels

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
 * Brightness is controlled with PWM signal to a pin controlling backlight.
 * Display must be already initialized by calling bsp_display_start()
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
 */
void bsp_button_init(void);

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
 * USB
 *
 * ESP32-S3-USB-OTG board comes with 3 USB ports:
 * 1. USB-UART0: Only for debugging purposes.
 * 2. USB DEV: USB A Male connector. ESP32-S3 acts as USB device.
 * 3. USB HOST: USB A Female connector. ESP32-S3 acts as USB host.
 *
 * In ESP32-S3 there is only one USB phy, so USB DEV and USB HOST connectors can't be used at the same time.
 * To activate one of the two connectors, use functions bsp_usb_mode_select_device() or bsp_usb_mode_select_host().
 **************************************************************************************************/

/**
 * @brief Switch ESP32-S3-USB-OTG to USB device mode
 *
 */
void bsp_usb_mode_select_device(void);

/**
 * @brief Switch ESP32-S3-USB-OTG to USB host mode
 *
 * For easy setup of USB host mode use bsp_usb_host_start() function.
 *
 * Use this in custom USB Host lib configurations.
 */
void bsp_usb_mode_select_host(void);

/**
 * @brief Power modes of USB Host connector
 *
 * For easy setup of USB host mode use bsp_usb_host_start() function.
 *
 * Use this function only if you want to change power mode on already initialized board,
 * or in custom USB Host lib configurations.
 *
 * @note USB Host connector can't be powered from debugging USB port (USB-UART0)
 * @note If selecting battery mode, the battery slide switch must be switched on
 */
typedef enum bsp_usb_host_power_mode_t {
    BSP_USB_HOST_POWER_MODE_OFF,     //!< No power
    BSP_USB_HOST_POWER_MODE_BATTERY, //!< Power from battery via 3.3->5V boost
    BSP_USB_HOST_POWER_MODE_USB_DEV, //!< Power from USB DEV port
} bsp_usb_host_power_mode_t;

/**
 * @brief Select power source of USB Host connector
 *
 * @param[in] mode        USB Host connector power mode
 * @param[in] limit_500mA Limit output current to 500mA
 */
void bsp_usb_host_power_mode(bsp_usb_host_power_mode_t mode, bool limit_500mA);

/**
 * @brief Start USB host
 *
 * This is a one-stop-shop function that will configure the board for USB Host mode
 * and start USB Host library
 *
 * @param[in] mode        USB Host connector power mode
 * @param[in] limit_500mA Limit output current to 500mA
 */
void bsp_usb_host_start(bsp_usb_host_power_mode_t mode, bool limit_500mA);

/**
 * @brief Stop USB host
 *
 * USB Host lib will be uninstalled and power from connector removed.
 */
void bsp_usb_host_stop(void);

/**************************************************************************************************
 *
 * Voltage measurements
 *
 * There are 2 voltages measured on ESP32-S2-USB-OTG board:
 * 1. Battery voltage
 * 2. Voltage on USB device connector
 **************************************************************************************************/

/**
 * @brief Init voltage measurements
 *
 * ADC configuration and calibration
 *
 * @note If the calibration fails, voltage can't be measured
 * @return true  Calibration OK
 * @return false Calibration failed
 */
bool bsp_voltage_init(void);

/**
 * @brief Get battery voltage
 *
 * @note bsp_voltage_init() must be called first
 * @return Resulting voltage in [mV]
 */
int bsp_voltage_battery_get(void);

/**
 * @brief Get USB device connector voltage
 *
 * @note bsp_voltage_init() must be called first
 * @return Resulting voltage in [mV]
 */
int bsp_voltage_usb_get(void);

#ifdef __cplusplus
}
#endif
