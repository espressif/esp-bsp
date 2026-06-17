/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP BSP: ESP-SensairShuttle
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "bsp/config.h"
#include "driver/i2c_master.h"
#include "esp_vfs_fat.h"
#include "driver/i2s_std.h"
#include "driver/i2s_pdm.h"
#include "esp_codec_dev.h"
#include "iot_button.h"
#include "bsp/display.h"
#include "led_indicator.h"
#include "iot_sensor_hub.h"


#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "lvgl.h"
#include "esp_lvgl_port.h"
#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0
/**************************************************************************************************
 *  BSP Board Name
 **************************************************************************************************/

/** @defgroup boardname Board Name
 *  @brief BSP Board Name
 *  @{
 */
#define BSP_BOARD_ESP_SENSAIRSHUTTLE
/** @} */ // end of boardname

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

/** @defgroup capabilities Capabilities
 *  @brief BSP Capabilities
 *  @{
 */
#define BSP_CAPS_DISPLAY          1
#define BSP_CAPS_TOUCH            1
#define BSP_CAPS_BUTTONS          1
#define BSP_CAPS_KNOB             0
#define BSP_CAPS_AUDIO            1
#define BSP_CAPS_AUDIO_SPEAKER    1
#define BSP_CAPS_AUDIO_MIC        1
#define BSP_CAPS_SDCARD           0
#define BSP_CAPS_LED              1
#define BSP_CAPS_CAMERA           0
#define BSP_CAPS_BAT              0
#define BSP_CAPS_IMU              1
#define BSP_CAPS_HUMITURE         0
/** @} */ // end of capabilities

/**************************************************************************************************
 *  Board pinout
 **************************************************************************************************/

/** @defgroup g01_i2c I2C
 *  @brief I2C BSP API
 *  @{
 */
#define BSP_I2C_SCL     (GPIO_NUM_3)
#define BSP_I2C_SDA     (GPIO_NUM_2)
/** @} */ // end of i2c

/** @defgroup g03_audio Audio
 *  @brief Audio BSP API
 *  @{
 */
#define BSP_I2S_CLK          (GPIO_NUM_NC)
#define BSP_I2S_DOUT         (GPIO_NUM_7)
#define BSP_I2S_DSIN         (GPIO_NUM_NC)
#define BSP_POWER_AMP_IO     (GPIO_NUM_NC)
#define BSP_SPEAKER_EN       (GPIO_NUM_1)
/** @} */ // end of audio

/** @defgroup g04_display Display and Touch
 *  @brief Display BSP API
 *  @{
 */
#define BSP_LCD_PCLK         (GPIO_NUM_24)
#define BSP_LCD_DATA0        (GPIO_NUM_23)
#define BSP_LCD_DC           (GPIO_NUM_26)
#define BSP_LCD_CS           (GPIO_NUM_25)
#define BSP_LCD_RST          (GPIO_NUM_NC)
#define BSP_LCD_BACKLIGHT    (GPIO_NUM_NC)
#define BSP_LCD_EN           (GPIO_NUM_5)
#define BSP_LCD_TOUCH_INT    (GPIO_NUM_NC)
/** @} */ // end of display

/** @defgroup g05_buttons Buttons
 *  @brief Buttons BSP API
 *  @{
 */
#define BSP_BUTTON_BOOT_IO   (GPIO_NUM_9)
/** @} */ // end of buttons

/** @defgroup g06_led Leds
 *  @brief Leds BSP API
 *  @{
 */
#define BSP_LED_RGB_IO       (GPIO_NUM_27)
/** @} */ // end of led

/** \addtogroup g05_buttons
 *  @{
 */
/* Button definitions */
typedef enum {
    BSP_BUTTON_BOOT,
    BSP_BUTTON_NUM
} bsp_button_t;
/** @} */ // end of buttons

/** \addtogroup g06_led
 *  @{
 */
/* LED definitions */
typedef enum {
    BSP_LED_1,
    BSP_LED_NUM
} bsp_led_t;

/* Default LED effects */
typedef enum {
    BSP_LED_ON,
    BSP_LED_OFF,
    BSP_LED_BLINK_FAST,
    BSP_LED_BLINK_SLOW,
    BSP_LED_BREATHE_FAST,
    BSP_LED_BREATHE_SLOW,
    BSP_LED_MAX,
} bsp_led_effect_t;
/** @} */ // end of leds

#ifdef __cplusplus
extern "C" {
#endif

/** \addtogroup g03_audio
 *  @{
 */

/**************************************************************************************************
 *
 * I2S audio interface
 *
 * For speaker initialization use bsp_audio_codec_speaker_init() which is inside initialize I2S with bsp_audio_init().
 * For microphone initialization use bsp_audio_codec_microphone_init() which is inside initialize I2S with bsp_audio_init().
 * After speaker or microphone initialization, use functions from esp_codec_dev for play/record audio.
 * Example audio play:
 * \code{.c}
 * esp_codec_dev_set_out_vol(spk_codec_dev, DEFAULT_VOLUME);
 * esp_codec_dev_open(spk_codec_dev, &fs);
 * esp_codec_dev_write(spk_codec_dev, wav_bytes, bytes_read_from_spiffs);
 * esp_codec_dev_close(spk_codec_dev);
 * \endcode
 **************************************************************************************************/

/**
 * @brief Init audio
 *
 * @note There is no deinit audio function. Users can free audio resources by calling i2s_del_channel()
 * @param[in]  i2s_config I2S configuration. Pass NULL to use default values
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_NOT_SUPPORTED The communication mode is not supported on the current chip
 *      - ESP_ERR_INVALID_ARG   NULL pointer or invalid configuration
 *      - ESP_ERR_NOT_FOUND     No available I2S channel found
 *      - ESP_ERR_NO_MEM        No memory for storing the channel information
 *      - ESP_ERR_INVALID_STATE This channel has not initialized or already started
 */
esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config);

/**
 * @brief Get codec I2S interface (initialized in bsp_audio_init)
 *
 * @return
 *      - Pointer to codec I2S interface handle or NULL when error occurred
 */
const audio_codec_data_if_t *bsp_audio_get_codec_itf(void);

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

/** @} */ // end of audio

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

/** @defgroup g02_storage SD Card and SPIFFS
 *  @brief SPIFFS and SD card BSP API
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

/** @} */ // end of storage

/** \addtogroup g04_display
 *  @{
 */

/**************************************************************************************************
 *
 * LCD interface
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling any LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * Display's backlight must be enabled explicitly by calling bsp_display_backlight_on()
 **************************************************************************************************/
#define BSP_LCD_PIXEL_CLOCK_HZ     (17 * 1000 * 1000)
#define BSP_LCD_SPI_NUM            (SPI2_HOST)

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

/**
 * @brief BSP display (LVGL) configuration structure
 */
typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg;  /*!< LVGL port configuration */
    uint32_t        buffer_size;    /*!< Size of the buffer for the screen in pixels */
    bool            double_buffer;  /*!< True, if should be allocated two buffers */
    struct {
        unsigned int buff_dma: 1;    /*!< Allocated LVGL buffer will be DMA capable */
        unsigned int buff_spiram: 1; /*!< Allocated LVGL buffer will be in PSRAM */
        unsigned int sw_rotate: 1;   /*!< Use software rotation (slower), The feature is unavailable under avoid-tear mode */
    } flags;
} bsp_display_cfg_t;

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
 * @brief Set display enter sleep mode
 *
 * All the display (LCD, backlight, touch) will enter sleep mode.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NOT_SUPPORTED if this function is not supported by the panel
 */
esp_err_t bsp_display_enter_sleep(void);

/**
 * @brief Set display exit sleep mode
 *
 * All the display (LCD, backlight, touch) will exit sleep mode.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_NOT_SUPPORTED if this function is not supported by the panel
 */
esp_err_t bsp_display_exit_sleep(void);

/**
 * @brief Rotate screen
 *
 * Display must be already initialized by calling bsp_display_start()
 *
 * @param[in] disp Pointer to LVGL display
 * @param[in] rotation Angle of the display rotation
 */
void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation);
#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

/** @} */ // end of display

/** @defgroup g99_others Others
 *  @brief Other BSP API
 *  @{
 */

/**************************************************************************************************
 *
 * BSP Features
 *
 * This module provides an interface to enable and disable various features on this board.
 *
 * The bsp_feature_enable function is used to enable or disable a specified feature.
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
    BSP_FEATURE_SPEAKER
} bsp_feature_t;

/**
 * @brief Enable selected feature
 *
 * @param feature  Feature for enablement
 * @param enable   Enable/disable selected feature
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 */
esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable);

/** @} */ // end of others

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

/** @} */ // end of leds

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
 *     - ESP_FAIL             Underlying iot_button_create failed
 */
esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size);

/** @} */ // end of buttons

/** @defgroup g10_sensors Sensors
 *  @brief BSP API for sensors
 *  @{
 */

/**************************************************************************************************
*
* Sensors API
*
* Available sensors:
*  - IMU: bmi270
*
* More information in display_sensors example
*
**************************************************************************************************/

/**
 * @brief BSP sensor configuration structure
 */
typedef struct {
    sensor_type_t type;
    sensor_mode_t mode;
    uint16_t period;
} bsp_sensor_config_t;

/**
 * @brief Initialize a sensor
 *
 * @param[in]  cfg              Pointer to the sensor configuration
 * @param[out] sensor_handle    Pointer to the outgoing sensor handle
 * @return
 *     - ESP_OK on success, otherwise returns ESP_ERR_xxx
 */
esp_err_t bsp_sensor_init(const bsp_sensor_config_t *cfg, sensor_handle_t *sensor_handle);

/** @} */ // end of sensors

#ifdef __cplusplus
}
#endif
