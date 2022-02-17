// Copyright 2015-2021 Espressif Systems (Shanghai) CO LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s.h"
#include "driver/touch_pad.h"
#include "button.h"

/* ESP32-S2 Kaluga Kit pinout */
#define BSP_I2C_SCL           (GPIO_NUM_7)
#define BSP_I2C_SDA           (GPIO_NUM_8)

#define BSP_I2S_SCLK          (GPIO_NUM_18)
#define BSP_I2S_MCLK          (GPIO_NUM_35)
#define BSP_I2S_LCLK          (GPIO_NUM_17)
#define BSP_I2S_DOUT          (GPIO_NUM_12)
#define BSP_I2S_DSIN          (GPIO_NUM_34)
#define BSP_POWER_AMP_IO      (GPIO_NUM_10)
#define BSP_LEDSTRIP_IO       (GPIO_NUM_45)

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 *
 * I2S audio interface
 *
 * After initialization of I2S, use BSP_I2S_NUM macro when reading/writing to I2S stream:
 * \code{.c}
 * i2s_write(BSP_I2S_NUM, wav_bytes, wav_bytes_len, &i2s_bytes_written, 500 / portTICK_PERIOD_MS);
 * \endcode
 **************************************************************************************************/
#define BSP_I2S_NUM           (I2S_NUM_0) // ESP32S2 has only 1 I2S peripheral

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
 * ESP32-S2-Kaluga-Kit is shipped with 3.2inch ST7789 display controller. It features 16-bit colors and 320x240 resolution.
 *
 * Default LVGL settings and LCD pinout are defined in sdkconfig.defaults file. Backlight control signal is disconnected on LCD board.
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
enum {
    BSP_BUTTON_REC = 0,
    BSP_BUTTON_MODE,
    BSP_BUTTON_PLAY,
    BSP_BUTTON_SET,
    BSP_BUTTON_VOLDOWN,
    BSP_BUTTON_VOLUP,
    BSP_BUTTON_NUM
} bsp_button_e;

extern const button_config_t bsp_button_config[BSP_BUTTON_NUM];

/**************************************************************************************************
 *
 * TouchPad interface
 *
 * @attention 'Photo' button on touch pad has a conflict with buttons on Audio board, they share the same IO no. 6.
 *            Thus Photo button and buttons on audio board cannot be used at the same time.
 * @attention 'Network' button on touch pad has a conflict with LCD chip select pin, they share the same IO no. 11.
 *            Thus Network button and LCD cannot be used at the same time.
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
