/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
#include "driver/touch_pad.h"
#include "iot_button.h"
#include "lvgl.h"

/**************************************************************************************************
 * ESP32-S2 Kaluga Kit pinout
 **************************************************************************************************/
/* I2C */
#define BSP_I2C_SCL           (GPIO_NUM_7)
#define BSP_I2C_SDA           (GPIO_NUM_8)

/* Audio */
#define BSP_I2S_SCLK          (GPIO_NUM_18)
#define BSP_I2S_MCLK          (GPIO_NUM_35)
#define BSP_I2S_LCLK          (GPIO_NUM_17)
#define BSP_I2S_DOUT          (GPIO_NUM_12)
#define BSP_I2S_DSIN          (GPIO_NUM_34)
#define BSP_POWER_AMP_IO      (GPIO_NUM_10)

/* Display */
#define BSP_LCD_SPI_MOSI      (GPIO_NUM_9)
#define BSP_LCD_SPI_CLK       (GPIO_NUM_15)
#define BSP_LCD_SPI_CS        (GPIO_NUM_11)
#define BSP_LCD_DC            (GPIO_NUM_13)
#define BSP_LCD_RST           (GPIO_NUM_16)

/* Touch pad */
/* Make sure that microswitches at the bottom of the Kaluga board are in ON position */
#define BSP_TOUCH_BUTTON_PLAY     (GPIO_NUM_2)
#define BSP_TOUCH_BUTTON_PHOTO    (GPIO_NUM_6)
#define BSP_TOUCH_BUTTON_NETWORK  (GPIO_NUM_11)
#define BSP_TOUCH_BUTTON_RECORD   (GPIO_NUM_5)
#define BSP_TOUCH_BUTTON_VOLUP    (GPIO_NUM_1)
#define BSP_TOUCH_BUTTON_VOLDOWN  (GPIO_NUM_3)
#define BSP_TOUCH_BUTTON_GUARD    (GPIO_NUM_4)
#define BSP_TOUCH_SHELED_ELECT    (GPIO_NUM_14)

/* Camera */
#define BSP_CAMERA_XCLK      (GPIO_NUM_1)
#define BSP_CAMERA_PCLK      (GPIO_NUM_33)
#define BSP_CAMERA_VSYNC     (GPIO_NUM_2)
#define BSP_CAMERA_HSYNC     (GPIO_NUM_3)
#define BSP_CAMERA_D0        (GPIO_NUM_36) /*!< Labeled as: D2 */
#define BSP_CAMERA_D1        (GPIO_NUM_37) /*!< Labeled as: D3 */
#define BSP_CAMERA_D2        (GPIO_NUM_41) /*!< Labeled as: D4 */
#define BSP_CAMERA_D3        (GPIO_NUM_42) /*!< Labeled as: D5 */
#define BSP_CAMERA_D4        (GPIO_NUM_39) /*!< Labeled as: D6 */
#define BSP_CAMERA_D5        (GPIO_NUM_40) /*!< Labeled as: D7 */
#define BSP_CAMERA_D6        (GPIO_NUM_21) /*!< Labeled as: D8 */
#define BSP_CAMERA_D7        (GPIO_NUM_38) /*!< Labeled as: D9 */

/* Others */
#define BSP_LEDSTRIP_IO      (GPIO_NUM_45)
#define BSP_BUTTONS_IO       (GPIO_NUM_6) // Push buttons on audio board, all mapped to this GPIO

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 *
 * I2S audio interface
 *
 * There is one device connected to the I2S peripheral:
 *  - Codec ES8311 (plyback and recording)
 *
 * After initialization of I2S, use BSP_I2S_NUM macro when reading/writing to I2S stream:
 * \code{.c}
 * i2s_write(BSP_I2S_NUM, wav_bytes, wav_bytes_len, &i2s_bytes_written, pdMS_TO_TICKS(500));
 * \endcode
 **************************************************************************************************/
#define BSP_I2S_NUM           (I2S_NUM_0) // ESP32-S2 has only 1 I2S peripheral

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
void bsp_audio_poweramp_enable(bool enable);

/**************************************************************************************************
 *
 * I2C interface
 *
 * There are two devices connected to I2C peripheral:
 *  - Codec ES8311 (configuration only)
 *  - External camera module
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
 * Camera interface
 *
 * ESP32-S2-Kaluga-Kit is shipped with OV2640 camera module.
 * As a camera driver, esp32-camera component is used.
 *
 * Example configuration:
 * \code{.c}
 * const camera_config_t camera_config = BSP_CAMERA_DEFAULT_CONFIG;
 * esp_err_t err = esp_camera_init(&camera_config);
 * \endcode
 **************************************************************************************************/
/**
 * @brief Kaluga camera default configuration
 *
 * In this configuration we select RGB565 color format and 320x240 image size,
 * we place the frame buffer to internal DRAM so we can sent it to LCD display without any changes.
 */
#define BSP_CAMERA_DEFAULT_CONFIG         \
    {                                     \
        .pin_pwdn = GPIO_NUM_NC,          \
        .pin_reset = GPIO_NUM_NC,         \
        .pin_xclk = BSP_CAMERA_XCLK,      \
        .pin_sscb_sda = GPIO_NUM_NC,      \
        .pin_sscb_scl = GPIO_NUM_NC,      \
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
        .xclk_freq_hz = 20000000,         \
        .ledc_timer = LEDC_TIMER_0,       \
        .ledc_channel = LEDC_CHANNEL_0,   \
        .pixel_format = PIXFORMAT_RGB565, \
        .frame_size = FRAMESIZE_QVGA,     \
        .jpeg_quality = 12,               \
        .fb_count = 1,                    \
        .fb_location = CAMERA_FB_IN_PSRAM \
    }

/**************************************************************************************************
 *
 * LCD interface
 *
 * ESP32-S2-Kaluga-Kit is shipped with 3.2inch ST7789 display controller.
 * It features 16-bit colors and 320x240 resolution.
 * Backlight control signal is disconnected on LCD board.
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling and LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 **************************************************************************************************/
#define BSP_LCD_H_RES              (320)
#define BSP_LCD_V_RES              (240)
#define BSP_LCD_PIXEL_CLOCK_HZ     (80 * 1000 * 1000)
#define BSP_LCD_SPI_NUM            (SPI3_HOST)

/**
 * @brief Initialize display and graphics library
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

/* Backlight functions are not implemented - Kaluga board doesn't provide backlight control
   These functions are here to provide consistent API with other Board Support Packages */
void bsp_display_backlight_on(void);
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
 * Audio buttons interface
 *
 * Example configuration:
 * \code{.c}
 * button_handle_t audio_button[BSP_BUTTON_NUM];
 * for (int i = 0; i < BSP_BUTTON_NUM; i++) {
 *     audio_button[i] = button_create(&bsp_button_config[i]);
 * }
 * \endcode
 *
 * @attention 'Photo' button on touch pad has a conflict with buttons on Audio board, they share the same IO no. 6.
 *            Thus photo button and buttons on audio board cannot be used at the same time.
 *            Make sure that microswitch T6 at the bottom of the Kaluga board is in ON position
 **************************************************************************************************/
typedef enum {
    BSP_BUTTON_REC = 0,
    BSP_BUTTON_MODE,
    BSP_BUTTON_PLAY,
    BSP_BUTTON_SET,
    BSP_BUTTON_VOLDOWN,
    BSP_BUTTON_VOLUP,
    BSP_BUTTON_NUM
} bsp_button_t;

extern const button_config_t bsp_button_config[BSP_BUTTON_NUM];

/**************************************************************************************************
 *
 * TouchPad interface
 *
 * @attention 'Photo' button on touch pad has a conflict with buttons on Audio board, they share the same IO no. 6.
 *            Thus Photo button and buttons on audio board cannot be used at the same time.
 * @attention 'Network' button on touch pad has a conflict with LCD chip select pin, they share the same IO no. 11.
 *            Thus Network button and LCD cannot be used at the same time.
 * @attention 'Play', 'Volume up' and 'Volume down' have a conflict with external camera interface.
 *            Thus, these touchpads can't be used at the same time with camera.
 * @note      It is useful to grant touchpad exclusive access to its pins.
 *            In order to achieve this, turn off switches T1-6, T11 and T14 on the bottom side of Kaluga board.
 **************************************************************************************************/
typedef enum {
    TOUCH_BUTTON_PLAY     = TOUCH_PAD_NUM2,
    TOUCH_BUTTON_PHOTO    = TOUCH_PAD_NUM6,
    TOUCH_BUTTON_NETWORK  = TOUCH_PAD_NUM11,
    TOUCH_BUTTON_RECORD   = TOUCH_PAD_NUM5,
    TOUCH_BUTTON_VOLUP    = TOUCH_PAD_NUM1,
    TOUCH_BUTTON_VOLDOWN  = TOUCH_PAD_NUM3,
    TOUCH_BUTTON_GUARD    = TOUCH_PAD_NUM4,
    TOUCH_SHELED_ELECT    = TOUCH_PAD_NUM14,
    TOUCH_BUTTON_NUM      = 7
} bsp_touchpad_button_t;

/**
 * @brief Init buttons on Touchpad board
 *
 * @attention This function initializes all touchpad buttons, including 'Photo' and 'Network' buttons,
 *            which have conflicts with Audio board buttons and LCD respectively.
 * @param[in] fn  Interrupt callback for touchpad peripheral.
 */
void bsp_touchpad_init(intr_handler_t fn);

/**
 * @brief Deinit buttons on Touchpad board
 *
 */
void bsp_touchpad_deinit(void);

/**
 * @brief Calibrate touch threshold
 *
 * @param[in] tch_pad       Touch pad from bsp_touchpad_button_t enum.
 * @param[in] tch_threshold Interrupt threshold ratio. Min.: 0, max.: 1.
 */
void bsp_touchpad_calibrate(bsp_touchpad_button_t tch_pad, float tch_threshold);

#ifdef __cplusplus
}
#endif
