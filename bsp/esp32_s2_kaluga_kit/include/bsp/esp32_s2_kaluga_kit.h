/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP BSP: ESP32-S2-Kaluga Kit
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/touch_sensor.h"
#include "driver/i2s_std.h"
#include "iot_button.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "esp_codec_dev.h"
#include "bsp/display.h"
#include "esp_adc/adc_oneshot.h"

/**************************************************************************************************
 *  BSP Board Name
 **************************************************************************************************/

/** @defgroup boardname Board Name
 *  @brief BSP Board Name
 *  @{
 */
#define BSP_BOARD_ESP32_S2_KALUGA_KIT
/** @} */ // end of boardname

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

/** @defgroup capabilities Capabilities
 *  @brief BSP Capabilities
 *  @{
 */
#define BSP_CAPS_DISPLAY        1
#define BSP_CAPS_TOUCH          0
#define BSP_CAPS_BUTTONS        1
#define BSP_CAPS_AUDIO          1
#define BSP_CAPS_AUDIO_SPEAKER  1
#define BSP_CAPS_AUDIO_MIC      1
#define BSP_CAPS_LED            1
#define BSP_CAPS_SDCARD         0
#define BSP_CAPS_IMU            0
#define BSP_CAPS_CAMERA         1
/** @} */ // end of capabilities

/**************************************************************************************************
 * ESP32-S2 Kaluga Kit pinout
 **************************************************************************************************/

/** @defgroup g01_i2c I2C
 *  @brief I2C BSP API
 *  @{
 */
#define BSP_I2C_SCL           (GPIO_NUM_7)
#define BSP_I2C_SDA           (GPIO_NUM_8)
/** @} */ // end of i2c

/** @defgroup g03_audio Audio
 *  @brief Audio BSP API
 *  @{
 */
#define BSP_I2S_SCLK          (GPIO_NUM_18)
#define BSP_I2S_MCLK          (GPIO_NUM_35)
#define BSP_I2S_LCLK          (GPIO_NUM_17)
#define BSP_I2S_DOUT          (GPIO_NUM_12)
#define BSP_I2S_DSIN          (GPIO_NUM_34)
#define BSP_POWER_AMP_IO      (GPIO_NUM_10)
/** @} */ // end of audio

/** @defgroup g04_display Display and Touch
 *  @brief Display BSP API
 *  @{
 */
#define BSP_LCD_SPI_MOSI      (GPIO_NUM_9)
#define BSP_LCD_SPI_CLK       (GPIO_NUM_15)
#define BSP_LCD_SPI_CS        (GPIO_NUM_11)
#define BSP_LCD_DC            (GPIO_NUM_13)
#define BSP_LCD_RST           (GPIO_NUM_16)
/** @} */ // end of display

/** @defgroup g08_camera Camera
 *  @brief Camera BSP API
 *  @{
 */
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
/** @} */ // end of camera

/** @defgroup g06_led Leds
 *  @brief Leds BSP API
 *  @{
 */
#define BSP_LEDSTRIP_IO      (GPIO_NUM_45)
/** @} */ // end of leds

/** @defgroup g05_buttons Buttons
 *  @brief Buttons BSP API
 *  @{
 */
#define BSP_BUTTONS_IO       (GPIO_NUM_6) // Push buttons on audio board, all mapped to this GPIO

/* Touch pad */
/* Make sure that microswitches T1-6, T11 and T14 at the bottom of the Kaluga board are in ON position */
#define BSP_TOUCH_BUTTON_PLAY     (GPIO_NUM_2)
#define BSP_TOUCH_BUTTON_PHOTO    (GPIO_NUM_6)
#define BSP_TOUCH_BUTTON_NETWORK  (GPIO_NUM_11)
#define BSP_TOUCH_BUTTON_RECORD   (GPIO_NUM_5)
#define BSP_TOUCH_BUTTON_VOLUP    (GPIO_NUM_1)
#define BSP_TOUCH_BUTTON_VOLDOWN  (GPIO_NUM_3)
#define BSP_TOUCH_BUTTON_GUARD    (GPIO_NUM_4)
#define BSP_TOUCH_SHELED_ELECT    (GPIO_NUM_14)
/** @} */ // end of buttons

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
 * There is one device connected to the I2S peripheral:
 *  - Codec ES8311 (playback and recording)
 *
 * For speaker initialization use bsp_audio_codec_speaker_init() which is inside initialize I2S with bsp_audio_init().
 * For microphone initialization use bsp_audio_codec_microphone_init() which is inside initialize I2S with bsp_audio_init().
 * After speaker or microphone initialization, use functions from esp_codec_dev for play/record audio.
 * Example audio play:
 * \code{.c}
 * esp_codec_dev_handle_t spk_codec_dev = bsp_audio_codec_speaker_init();
 * esp_codec_dev_set_out_vol(spk_codec_dev, DEFAULT_VOLUME);
 * esp_codec_dev_open(spk_codec_dev, &fs);
 * esp_codec_dev_write(spk_codec_dev, wav_bytes, wav_bytes_len);
 * esp_codec_dev_close(spk_codec_dev);
 * \endcode
 **************************************************************************************************/

/**
 * @brief Init audio
 *
 * @note There is no deinit audio function. Users can free audio resources by calling i2s_del_channel()
 * @warning The type of i2s_config param is depending on IDF version.
 * @param[in]  i2s_config I2S configuration. Pass NULL to use default values (Mono, duplex, 16bit, 22050 Hz)
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
 * There are two devices connected to I2C peripheral:
 *  - Codec ES8311 (configuration only)
 *  - External camera module
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

/** \addtogroup g08_camera
 *  @{
 */

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
 * In this configuration we select RGB565 color format and 320x240 image size - matching the display.
 * We use double-buffering for the best performance.
 * Since ESP32-S2 has only 320kB of internal SRAM, we allocate the framebuffers in external PSRAM.
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
        .frame_size = FRAMESIZE_QVGA,     \
        .jpeg_quality = 12,               \
        .fb_count = 2,                    \
        .fb_location = CAMERA_FB_IN_PSRAM,\
        .sccb_i2c_port = BSP_I2C_NUM,     \
    }

#define BSP_CAMERA_VFLIP        1
#define BSP_CAMERA_HMIRROR      0

/** @} */ // end of camera

/** \addtogroup g04_display
 *  @{
 */

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
 *
 * @note Default SPI clock is set to 40MHz. On some displays shipped with Kaluga kit, 80MHz can be achieved
 **************************************************************************************************/
#define BSP_LCD_PIXEL_CLOCK_HZ     (40 * 1000 * 1000)
#define BSP_LCD_SPI_NUM            (SPI3_HOST)

#define BSP_LCD_DRAW_BUFF_SIZE     (BSP_LCD_H_RES * 20)
#define BSP_LCD_DRAW_BUFF_DOUBLE   (0)

/**
 * @brief BSP display configuration structure
 *
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
 * @brief Initialize display and graphics library
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 *
 * @return Pointer to LVGL display or NULL when error occurred
 */
lv_display_t *bsp_display_start(void);

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 *
 * @param[in] cfg Display configuration
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

/** @} */ // end of display

/** @defgroup g01_adc ADC
 *  @brief ADC BSP API
 *  @{
 */

/**************************************************************************************************
 *
 * ADC interface
 *
 * There are multiple devices connected to ADC peripheral:
 *  - Buttons
 *  - Battery voltage measurement
 *
 * After initialization of ADC, use adc_handle when using ADC driver.
 **************************************************************************************************/

#define BSP_ADC_UNIT     ADC_UNIT_1

/**
 * @brief Initialize ADC
 *
 * The ADC can be initialized inside BSP, when needed.
 */
esp_err_t bsp_adc_initialize(void);

/**
 * @brief Get ADC handle
 *
 * @note This function is available only in IDF5 and higher
 *
 * @return ADC handle
 */
adc_oneshot_unit_handle_t bsp_adc_get_handle(void);

/** @} */ // end of adc

/** \addtogroup g05_buttons
 *  @{
 */

/**************************************************************************************************
 *
 * Audio buttons interface
 *
 * Example configuration:
 * \code{.c}
 * button_handle_t btns[BSP_BUTTON_NUM];
 * bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM);
 * iot_button_register_cb(btns[0], ...
 * \endcode
 *
 * @attention 'Photo' button on touch pad has a conflict with buttons on Audio board, they share the same IO no. 6.
 *            Thus photo button and buttons on audio board cannot be used at the same time.
 *            Make sure that microswitch T6 at the bottom of the Kaluga board is in ON position
 **************************************************************************************************/
//@todo add touch pads here
typedef enum {
    BSP_BUTTON_REC = 0,
    BSP_BUTTON_MODE,
    BSP_BUTTON_PLAY,
    BSP_BUTTON_SET,
    BSP_BUTTON_VOLDOWN,
    BSP_BUTTON_VOLUP,
    BSP_BUTTON_NUM
} bsp_button_t;

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
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   NULL pointer or invalid configuration
 *      - ESP_FAIL              Touch pad not initialized
 *      - ESP_ERR_NO_MEM        No memory
 */
esp_err_t bsp_touchpad_init(intr_handler_t fn);

/**
 * @brief Deinit buttons on Touchpad board
 *
 */
esp_err_t bsp_touchpad_deinit(void);

/**
 * @brief Calibrate touch threshold
 *
 * @param[in] tch_pad       Touch pad from bsp_touchpad_button_t enum.
 * @param[in] tch_threshold Interrupt threshold ratio. Min.: 0, max.: 1.
 * @return
 *      - ESP_OK                On success
 *      - ESP_ERR_INVALID_ARG   NULL pointer or invalid configuration
 */
esp_err_t bsp_touchpad_calibrate(bsp_touchpad_button_t tch_pad, float tch_threshold);

/** @} */ // end of buttons

#ifdef __cplusplus
}
#endif
