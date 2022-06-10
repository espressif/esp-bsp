/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
#include "soc/usb_pins.h"
#include "lvgl.h"

/**************************************************************************************************
 *  ESP-BOX pinout
 **************************************************************************************************/
/* I2C */
#define BSP_I2C_SCL           (GPIO_NUM_18)
#define BSP_I2C_SDA           (GPIO_NUM_8)

/* Audio */
#define BSP_I2S_SCLK          (GPIO_NUM_17)
#define BSP_I2S_MCLK          (GPIO_NUM_2)
#define BSP_I2S_LCLK          (GPIO_NUM_47)
#define BSP_I2S_DOUT          (GPIO_NUM_15) // To Codec ES8311
#define BSP_I2S_DSIN          (GPIO_NUM_16) // From ADC ES7210
#define BSP_POWER_AMP_IO      (GPIO_NUM_46)
#define BSP_MUTE_STATUS       (GPIO_NUM_1)

/* Display */
#define BSP_LCD_DATA0         (GPIO_NUM_6)
#define BSP_LCD_PCLK          (GPIO_NUM_7)
#define BSP_LCD_CS            (GPIO_NUM_5)
#define BSP_LCD_DC            (GPIO_NUM_4)
#define BSP_LCD_RST           (GPIO_NUM_48)
#define BSP_LCD_BACKLIGHT     (GPIO_NUM_45)
#define BSP_LCD_TOUCH_INT     (GPIO_NUM_3)

/* USB */
#define BSP_USB_POS           USBPHY_DP_NUM
#define BSP_USB_NEG           USBPHY_DM_NUM

/* PMOD */
/*
 * PMOD interface (peripheral module interface) is an open standard defined by Digilent Inc.
 * for peripherals used with FPGA or microcontroller development boards.
 *
 * ESP-BOX contains two double PMOD connectors, protected with ESD protection diodes.
 * Power pins are on 3.3V.
 *
 * Double PMOD Connectors on ESP-BOX are labeled as follows:
 *      ┌────────────┐
 *      | IO1    IO5 │
 *      | IO2    IO6 │
 *      | IO3    IO7 │
 *      | IO4    IO8 │
 *      ├────────────┤
 *      | GND    GND │
 *      | 3V3    3V3 │
 *      └────────────┘
 */
#define BSP_PMOD1_IO1        GPIO_NUM42
#define BSP_PMOD1_IO2        GPIO_NUM21
#define BSP_PMOD1_IO3        BSP_USB_NEG
#define BSP_PMOD1_IO4        BSP_USB_POS
#define BSP_PMOD1_IO5        GPIO_NUM38
#define BSP_PMOD1_IO6        GPIO_NUM39
#define BSP_PMOD1_IO7        GPIO_NUM40  // Intended for I2C SCL (pull-up NOT populated)
#define BSP_PMOD1_IO8        GPIO_NUM41  // Intended for I2C SDA (pull-up NOT populated)

#define BSP_PMOD2_IO1        GPIO_NUM10  // Intended for SPI2 CS
#define BSP_PMOD2_IO2        GPIO_NUM11  // Intended for SPI2 D (MOSI)
#define BSP_PMOD2_IO3        GPIO_NUM13  // Intended for SPI2 Q (MISO)
#define BSP_PMOD2_IO4        GPIO_NUM12  // Intended for SPI2 CLK
#define BSP_PMOD2_IO5        GPIO_NUM9   // Intended for SPI2 HD (Hold)
#define BSP_PMOD2_IO6        GPIO_NUM43  // UART0 TX by default
#define BSP_PMOD2_IO7        GPIO_NUM44  // UART0 RX by default
#define BSP_PMOD2_IO8        GPIO_NUM14  // Intended for SPI2 WP (Write-protect)

/* Buttons */
typedef enum {
    BSP_BUTTON_CONFIG = GPIO_NUM_0,
    BSP_BUTTON_MUTE   = GPIO_NUM_1
} bsp_button_t;

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 *
 * I2S audio interface
 *
 * There are two devices connected to the I2S peripheral:
 *  - Codec ES8311 for output (playback) path
 *  - ADC ES7210 for input (recording) path
 *
 * After initialization of I2S, use BSP_I2S_NUM macro when reading/writing to I2S stream:
 * \code{.c}
 * i2s_write(BSP_I2S_NUM, wav_bytes, wav_bytes_len, &i2s_bytes_written, pdMS_TO_TICKS(500));
 * \endcode
 **************************************************************************************************/
#define BSP_I2S_NUM           CONFIG_BSP_I2S_NUM

/**
 * @brief Mono Duplex I2S init structure
 *
 */
#define BSP_I2S_DUPLEX_MONO_CONFIG(_sample_rate)                      \
    {                                                                 \
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,          \
        .sample_rate = _sample_rate,                                  \
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,                 \
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,                  \
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,            \
        .dma_buf_count = 3,                                           \
        .dma_buf_len = 1024,                                          \
        .use_apll = true,                                             \
        .tx_desc_auto_clear = true,                                   \
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM \
    }

/**
 * @brief ES8311 init structure
 *
 */
#define BSP_ES8311_SCLK_CONFIG(_sample_rate) \
    {                                        \
        .mclk_from_mclk_pin = false,         \
        .mclk_inverted = false,              \
        .sclk_inverted = false,              \
        .sample_frequency = _sample_rate     \
    }

/**
 * @brief Init audio
 *
 * @param[in] i2s_config I2S configuration
 */
void bsp_audio_init(const i2s_config_t *i2s_config);

/**
 * @brief Deinit audio
 *
 * Frees resources used for I2S driver.
 */
void bsp_audio_deinit(void);

/**
 * @brief Enable/disable audio power amplifier
 *
 * @param[in] enable Enable/disable audio power amplifier
 */
void bsp_audio_poweramp_enable(const bool enable);

/**************************************************************************************************
 *
 * I2C interface
 *
 * There are multiple devices connected to I2C peripheral:
 *  - Codec ES8311 (configuration only)
 *  - ADC ES7210 (configuration only)
 *  - Encryption chip ATECC608A (NOT populated on most boards)
 *  - LCD Touch controller
 *  - Inertial Measurement Unit ICM-42607-P
 *
 * After initialization of I2C, use BSP_I2C_NUM macro when creating I2C devices drivers ie.:
 * \code{.c}
 * es8311_handle_t es8311_dev = es8311_create(BSP_I2C_NUM, ES8311_ADDRRES_0);
 * \endcode
 **************************************************************************************************/
#define BSP_I2C_NUM     CONFIG_BSP_I2C_NUM

/**
 * @brief Init I2C driver
 *
 */
void bsp_i2c_init(void);

/**
 * @brief Deinit I2C driver and free its resources
 *
 */
void bsp_i2c_deinit(void);

/**************************************************************************************************
 *
 * LCD interface
 *
 * ESP-BOX is shipped with 2.4inch ST7789 display controller.
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

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
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
/**************************************************************************************************
 *
 * Button
 *
 * There are three buttons on ESP-BOX:
 *  - Reset:  Not programable
 *  - Config: Controls boot mode during reset. Can be programmed after application starts
 *  - Mute:   This button is wired to Logic Gates and its result is mapped to GPIO_NUM_1
 **************************************************************************************************/

/**
 * @brief Set button's GPIO as input
 *
 * @param[in] btn Button to be initialized
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_button_init(const bsp_button_t btn);

/**
 * @brief Get button's state
 *
 * @param[in] btn Button to read
 * @return true  Button pressed
 * @return false Button released
 */
bool bsp_button_get(const bsp_button_t btn);

#ifdef __cplusplus
}
#endif
