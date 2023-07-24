/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "driver/sdmmc_host.h"
#include "soc/usb_pins.h"
#include "iot_button.h"
#include "esp_io_expander.h"
#include "esp_codec_dev.h"
#include "lvgl.h"

/**************************************************************************************************
 *  Board pinout
 **************************************************************************************************/
/* I2C */
#define BSP_I2C_SCL           (GPIO_NUM_18)
#define BSP_I2C_SDA           (GPIO_NUM_17)

/* Audio */
#define BSP_I2S_SCLK          (GPIO_NUM_9)
#define BSP_I2S_MCLK          (GPIO_NUM_16)
#define BSP_I2S_LCLK          (GPIO_NUM_45)
#define BSP_I2S_DOUT          (GPIO_NUM_8)   // To Codec ES7210
#define BSP_I2S_DSIN          (GPIO_NUM_10)  // From ADC ES8311
#define BSP_POWER_AMP_IO      (GPIO_NUM_48)

/* Display */
#define BSP_LCD_DATA0         (GPIO_NUM_0)
#define BSP_LCD_PCLK          (GPIO_NUM_1)
#define BSP_LCD_DC            (GPIO_NUM_2)
#define BSP_LCD_CS            (GPIO_NUM_NC) // Connected via IO expander P3
#define BSP_LCD_RST           (GPIO_NUM_NC) // Connected via IO expander P2
#define BSP_LCD_BACKLIGHT     (GPIO_NUM_NC) // Connected via IO expander P1
#define BSP_LCD_TOUCH_INT     (GPIO_NUM_NC) // Connected via IO expander P4
#define BSP_LCD_IO_CS         (IO_EXPANDER_PIN_NUM_3)
#define BSP_LCD_IO_RST        (IO_EXPANDER_PIN_NUM_2)
#define BSP_LCD_IO_BACKLIGHT  (IO_EXPANDER_PIN_NUM_1)

/* Camera */
#define BSP_CAMERA_XCLK      (GPIO_NUM_40)
#define BSP_CAMERA_PCLK      (GPIO_NUM_11)
#define BSP_CAMERA_VSYNC     (GPIO_NUM_21)
#define BSP_CAMERA_HSYNC     (GPIO_NUM_38)
#define BSP_CAMERA_D0        (GPIO_NUM_13)
#define BSP_CAMERA_D1        (GPIO_NUM_47)
#define BSP_CAMERA_D2        (GPIO_NUM_14)
#define BSP_CAMERA_D3        (GPIO_NUM_3)
#define BSP_CAMERA_D4        (GPIO_NUM_12)
#define BSP_CAMERA_D5        (GPIO_NUM_42)
#define BSP_CAMERA_D6        (GPIO_NUM_41)
#define BSP_CAMERA_D7        (GPIO_NUM_39)

/* uSD card */
#define BSP_SD_D0             (GPIO_NUM_4)
#define BSP_SD_CMD            (GPIO_NUM_7)
#define BSP_SD_CLK            (GPIO_NUM_15)

/* Battery */
#define BSP_BATTERY_VOLTAGE   (GPIO_NUM_6)  // Voltage at this pin = (V_BAT / 4), ADC1 channel 5
#define BSP_BATTERY_VOLTAGE_DIV (4)

/* Buttons */
#define BSP_BUTTONS_IO       (GPIO_NUM_5) // All 5 buttons mapped to this GPIO

/* Leds */
typedef enum bsp_led_t {
    BSP_LED_BLUE = IO_EXPANDER_PIN_NUM_6,
    BSP_LED_RED = IO_EXPANDER_PIN_NUM_7,
} bsp_led_t;

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 *
 * Buttons interface
 *
 * Example configuration:
 * \code{.c}
 * button_handle_t btns[BSP_BUTTON_NUM];
 * bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM);
 * iot_button_register_cb(btns[0], ...
 * \endcode
 **************************************************************************************************/
typedef enum {
    BSP_BUTTON_REC = 0,
    BSP_BUTTON_MUTE,
    BSP_BUTTON_PLAY,
    BSP_BUTTON_SET,
    BSP_BUTTON_VOLDOWN,
    BSP_BUTTON_VOLUP,
    BSP_BUTTON_MAIN,
    BSP_BUTTON_NUM,
} bsp_button_t;

/**
 * @brief Set button's GPIO as input
 *
 * @param[in] btn Button to be initialized
 * @return
 *     - ESP_OK Success
 */
esp_err_t bsp_button_init(const bsp_button_t btn)
__attribute__((deprecated("use bsp_iot_button_create instead")));

/**
 * @brief Get button's state
 *
 * Note: For LCD panel button which is defined as BSP_BUTTON_MAIN, bsp_display_start should
 *       be called before call this function.
 *
 * @param[in] btn Button to read
 * @return true  Button pressed
 * @return false Button released
 */
bool bsp_button_get(const bsp_button_t btn)
__attribute__((deprecated("use espressif/button API instead")));

/**
 * @brief Initialize all buttons
 *
 * Returned button handlers must be used with espressif/button component API
 *
 * @note For LCD panel button which is defined as BSP_BUTTON_MAIN, bsp_display_start should
 *       be called before call this function.
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

/**************************************************************************************************
 *
 * I2S audio interface
 *
 * There are two devices connected to the I2S peripheral:
 *  - Codec ES8311 for output (playback) path
 *  - ADC ES7210 for input (recording) path
 *
 * For speaker initialization use bsp_audio_codec_speaker_init() which is inside initialize I2S with bsp_audio_init().
 * For microphone initialization use bsp_audio_codec_microphone_init() which is inside initialize I2S with bsp_audio_init().
 * After speaker or microphone initialization, use functions from esp_codec_dev for play/record audio.
 * Example audio play:
 * \code{.c}
 * esp_codec_dev_set_out_vol(spk_codec_dev, DEFAULT_VOLUME);
 * esp_codec_dev_open(spk_codec_dev, &fs);
 * esp_codec_dev_write(spk_codec_dev, wav_bytes, wav_bytes_len);
 * esp_codec_dev_close(spk_codec_dev);
 * \endcode
 **************************************************************************************************/

/**
 * @brief Initialize speaker codec device
 *
 * @return Pointer to codec device handle or NULL when error occurred
 */
esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void);

/**
 * @brief Initialize microphone codec device
 *
 * @return Pointer to codec device handle or NULL when error occurred
 */
esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void);

/**************************************************************************************************
 *
 * I2C interface
 *
 * There are multiple devices connected to I2C peripheral:
 *  - Codec ES8311 (configuration only)
 *  - ADC ES7210 (configuration only)
 *  - LCD Touch controller
 *  - IO Expander TCA9554
 *  - Camera
 *
 * After initialization of I2C, use BSP_I2C_NUM macro when creating I2C device drivers
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
 * IO Expander Interface
 *
 **************************************************************************************************/
#define BSP_IO_EXPANDER_I2C_ADDRESS     (ESP_IO_EXPANDER_I2C_TCA9554A_ADDRESS_000)

/**
 * @brief Init IO expander chip TCA9554
 *
 * @note I2C must be already initialized by bsp_i2c_init()
 * @note If the device was already initialized, users can also call it to get handle
 * @note This function will be called in `bsp_display_start()`
 *
 * @return Pointer to device handle or NULL when error occurred
 */
esp_io_expander_handle_t bsp_io_expander_init(void);


/**************************************************************************************************
 *
 * Camera interface
 *
 * ESP32-S3-Korvo-2 has connector for camera.
 * As a camera driver, esp32-camera component is used.
 *
 * Example configuration:
 * \code{.c}
 * const camera_config_t camera_config = BSP_CAMERA_DEFAULT_CONFIG;
 * esp_err_t err = esp_camera_init(&camera_config);
 * \endcode
 **************************************************************************************************/
/**
 * @brief ESP32-S3-Korvo-2 camera default configuration
 *
 * In this configuration we select RGB565 color format and 240x240 image size - matching the display.
 * We use double-buffering for the best performance.
 * Since we don't want to waste internal SRAM, we allocate the framebuffers in external PSRAM.
 * By setting XCLK to 16MHz, we configure the esp32-camera driver to use EDMA when accessing the PSRAM.
 *
 * @attention I2C must be enabled by bsp_i2c_init(), before camera is initialized
 */
#define BSP_CAMERA_DEFAULT_CONFIG         \
    {                                     \
        .pin_pwdn = GPIO_NUM_NC,          \
        .pin_reset = GPIO_NUM_NC,         \
        .pin_xclk = BSP_CAMERA_XCLK,      \
        .pin_sccb_sda = GPIO_NUM_NC,      \
        .pin_sccb_scl = GPIO_NUM_NC,      \
        .pin_d7 = BSP_CAMERA_D7,          \
        .pin_d6 = BSP_CAMERA_D6,          \
        .pin_d5 = BSP_CAMERA_D5,          \
        .pin_d4 = BSP_CAMERA_D4,          \
        .pin_d3 = BSP_CAMERA_D3,          \
        .pin_d2 = BSP_CAMERA_D2,          \
        .pin_d1 = BSP_CAMERA_D1,          \
        .pin_d0 = BSP_CAMERA_D0,          \
        .pin_vsync = BSP_CAMERA_VSYNC,    \
        .pin_href = BSP_CAMERA_HSYNC,     \
        .pin_pclk = BSP_CAMERA_PCLK,      \
        .xclk_freq_hz = 16000000,         \
        .ledc_timer = LEDC_TIMER_0,       \
        .ledc_channel = LEDC_CHANNEL_0,   \
        .pixel_format = PIXFORMAT_RGB565, \
        .frame_size = FRAMESIZE_240X240,  \
        .jpeg_quality = 12,               \
        .fb_count = 2,                    \
        .fb_location = CAMERA_FB_IN_PSRAM,\
        .sccb_i2c_port = BSP_I2C_NUM,     \
    }

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
#define BSP_MOUNT_POINT      CONFIG_BSP_SD_MOUNT_POINT
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
 * Board is shipped with 2.4inch ILI9341 display controller.
 * It features 16-bit colors, 320x240 resolution and capacitive touch controller.
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling and LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * Display's backlight must be enabled explicitly by calling bsp_display_backlight_on()
 **************************************************************************************************/
#define BSP_LCD_H_RES              (320)
#define BSP_LCD_V_RES              (240)
#define BSP_LCD_PIXEL_CLOCK_HZ     (40 * 1000 * 1000)
#define BSP_LCD_SPI_NUM            (SPI3_HOST)

#define BSP_LCD_DRAW_BUFF_SIZE     (BSP_LCD_H_RES * 50)
#define BSP_LCD_DRAW_BUFF_DOUBLE   (1)

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @return Pointer to LVGL display or NULL when error occured
 */
lv_disp_t *bsp_display_start(void);

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
 * @brief Set display's brightness
 *
 * @attention ESP32-S3 Korvo v2 board has backlight control connected through IO expander, so brightness control is not supported.
 *
 * @param[in] brightness_percent Brightness in [%]
 * @return
 *      - ESP_ERR_NOT_SUPPORTED Always
 */
esp_err_t bsp_display_brightness_set(int brightness_percent);

/**
 * @brief Turn on display backlight
 *
 * @note I2C must be already initialized by bsp_i2c_init()
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_STATE Could not init IO expander
 */
esp_err_t bsp_display_backlight_on(void);

/**
 * @brief Turn off display backlight
 *
 * @note I2C must be already initialized by bsp_i2c_init()
 *
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_STATE Could not init IO expander
 */
esp_err_t bsp_display_backlight_off(void);

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
 *
 * @note I2C must be already initialized by bsp_i2c_init()
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_STATE Could not init IO expander
 *     - ESP_ERR_INVALID_ARG   Parameter error
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
 * Voltage measurements
 *
 **************************************************************************************************/

/**
 * @brief Init voltage measurements
 *
 * ADC configuration and calibration
 *
 * @note If the calibration fails, voltage can't be measured
 * @return true  Calibration OK
 * @return false Calibration failed
 * @return
 *     - ESP_OK                 On success
 *     - ESP_ERR_INVALID_ARG    Invalid arguments
 *     - ESP_ERR_NO_MEM         No memory
 *     - ESP_ERR_NOT_FOUND      ADC peripheral to be claimed is already in use
 *     - ESP_ERR_NOT_SUPPORTED  ADC scheme required eFuse bits not burnt
 */
esp_err_t bsp_voltage_init(void);

/**
 * @brief Get battery voltage
 *
 * @note bsp_voltage_init() must be called first
 * @return Resulting voltage in [mV] or -1 on error
 */
int bsp_voltage_battery_get(void);

#ifdef __cplusplus
}
#endif
