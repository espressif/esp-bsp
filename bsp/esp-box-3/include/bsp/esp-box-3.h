/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP BSP: ESP-BOX-3
 */

#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "esp_vfs_fat.h"
#include "esp_codec_dev.h"
#include "iot_button.h"
#include "bsp/config.h"
#include "bsp/display.h"

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "lvgl.h"
#include "esp_lvgl_port.h"
#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

#define BSP_CAPS_DISPLAY        1
#define BSP_CAPS_TOUCH          1
#define BSP_CAPS_BUTTONS        1
#define BSP_CAPS_AUDIO          1
#define BSP_CAPS_AUDIO_SPEAKER  1
#define BSP_CAPS_AUDIO_MIC      1
#define BSP_CAPS_SDCARD         1
#define BSP_CAPS_IMU            1

/**************************************************************************************************
 *  ESP-BOX pinout
 **************************************************************************************************/
/* I2C */
#define BSP_I2C_SCL           (GPIO_NUM_18)
#define BSP_I2C_SDA           (GPIO_NUM_8)

/* Audio */
#define BSP_I2S_SCLK          (GPIO_NUM_17)
#define BSP_I2S_MCLK          (GPIO_NUM_2)
#define BSP_I2S_LCLK          (GPIO_NUM_45)
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

#define BSP_LCD_BACKLIGHT     (GPIO_NUM_47)
#define BSP_LCD_TOUCH_INT     (GPIO_NUM_3)

/* USB */
#define BSP_USB_POS           (GPIO_NUM_20)
#define BSP_USB_NEG           (GPIO_NUM_19)

/* Buttons */
#define BSP_BUTTON_CONFIG_IO  (GPIO_NUM_0)
#define BSP_BUTTON_MUTE_IO    (GPIO_NUM_1)

/* uSD card MMC */
#define BSP_SD_D0             (GPIO_NUM_9)
#define BSP_SD_D1             (GPIO_NUM_13)
#define BSP_SD_D2             (GPIO_NUM_42)
#define BSP_SD_D3             (GPIO_NUM_12)
#define BSP_SD_CMD            (GPIO_NUM_14)
#define BSP_SD_CLK            (GPIO_NUM_11)
#define BSP_SD_DET            (GPIO_NUM_NC)
#define BSP_SD_POWER          (GPIO_NUM_43)

/* uSD card SPI */
#define BSP_SD_SPI_MISO       (GPIO_NUM_9)
#define BSP_SD_SPI_CS         (GPIO_NUM_12)
#define BSP_SD_SPI_MOSI       (GPIO_NUM_14)
#define BSP_SD_SPI_CLK        (GPIO_NUM_11)

/* PMOD */
/*
 * PMOD interface (peripheral module interface) is an open standard defined by Digilent Inc.
 * for peripherals used with FPGA or microcontroller development boards.
 *
 * ESP-BOX contains two double PMOD connectors, protected with ESD protection diodes.
 * Power pins are on 3.3V.
 *
 * Double PMOD Connectors on ESP-BOX-3 dock are labeled as follows:
 *      |------------|
 *      | IO1    IO5 |
 *      | IO2    IO6 |
 *      | IO3    IO7 |
 *      | IO4    IO8 |
 *      |------------|
 *      | GND    GND |
 *      | 3V3    3V3 |
 *      |------------|
 */
#define BSP_PMOD1_IO1        GPIO_NUM_42
#define BSP_PMOD1_IO2        BSP_USB_POS
#define BSP_PMOD1_IO3        GPIO_NUM_39
#define BSP_PMOD1_IO4        GPIO_NUM_40 // Intended for I2C SCL (pull-up NOT populated)
#define BSP_PMOD1_IO5        GPIO_NUM_21
#define BSP_PMOD1_IO6        BSP_USB_NEG
#define BSP_PMOD1_IO7        GPIO_NUM_38
#define BSP_PMOD1_IO8        GPIO_NUM_41 // Intended for I2C SDA (pull-up NOT populated)

#define BSP_PMOD2_IO1        GPIO_NUM_13 // Intended for SPI2 Q (MISO)
#define BSP_PMOD2_IO2        GPIO_NUM_9  // Intended for SPI2 HD (Hold)
#define BSP_PMOD2_IO3        GPIO_NUM_12 // Intended for SPI2 CLK
#define BSP_PMOD2_IO4        GPIO_NUM_44 // UART0 RX by default
#define BSP_PMOD2_IO5        GPIO_NUM_10 // Intended for SPI2 CS
#define BSP_PMOD2_IO6        GPIO_NUM_14 // Intended for SPI2 WP (Write-protect)
#define BSP_PMOD2_IO7        GPIO_NUM_11 // Intended for SPI2 D (MOSI)
#define BSP_PMOD2_IO8        GPIO_NUM_43 // UART0 TX by defaultf

/* Buttons */
typedef enum {
    BSP_BUTTON_CONFIG = 0,
    BSP_BUTTON_MUTE,
    BSP_BUTTON_MAIN,
    BSP_BUTTON_NUM
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
 * SD card
 *
 * After mounting the SD card, it can be accessed with stdio functions ie.:
 * \code{.c}
 * FILE* f = fopen(BSP_SD_MOUNT_POINT"/hello.txt", "w");
 * fprintf(f, "Hello %s!\n", bsp_sdcard->cid.name);
 * fclose(f);
 * \endcode
 *
 * @attention IO2 is also routed to RGB LED and push button
 **************************************************************************************************/
#define BSP_SD_MOUNT_POINT      CONFIG_BSP_SD_MOUNT_POINT
#define BSP_SDSPI_HOST          (SPI2_HOST)

typedef struct {
    const esp_vfs_fat_sdmmc_mount_config_t *mount;
    sdmmc_host_t *host;
    union {
        const sdmmc_slot_config_t   *sdmmc;
        const sdspi_device_config_t *sdspi;
    } slot;
} bsp_sdcard_cfg_t;

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

/**
 * @brief Get SD card handle
 *
 * @return SD card handle
 */
sdmmc_card_t *bsp_sdcard_get_handle(void);

/**
 * @brief Get SD card MMC host config
 *
 * @param slot SD card slot
 * @param config Structure which will be filled
 */
void bsp_sdcard_get_sdmmc_host(const int slot, sdmmc_host_t *config);

/**
 * @brief Get SD card SPI host config
 *
 * @param slot SD card slot
 * @param config Structure which will be filled
 */
void bsp_sdcard_get_sdspi_host(const int slot, sdmmc_host_t *config);

/**
 * @brief Get SD card MMC slot config
 *
 * @param slot SD card slot
 * @param config Structure which will be filled
 */
void bsp_sdcard_sdmmc_get_slot(const int slot, sdmmc_slot_config_t *config);

/**
 * @brief Get SD card SPI slot config
 *
 * @param spi_host SPI host ID
 * @param config Structure which will be filled
 */
void bsp_sdcard_sdspi_get_slot(const spi_host_device_t spi_host, sdspi_device_config_t *config);

/**
 * @brief Mount microSD card to virtual file system (MMC mode)
 *
 * @param cfg SD card configuration
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if esp_vfs_fat_sdmmc_mount was already called
 *      - ESP_ERR_NO_MEM if memory cannot be allocated
 *      - ESP_FAIL if partition cannot be mounted
 *      - other error codes from SDMMC or SPI drivers, SDMMC protocol, or FATFS drivers
 */
esp_err_t bsp_sdcard_sdmmc_mount(bsp_sdcard_cfg_t *cfg);

/**
 * @brief Mount microSD card to virtual file system (SPI mode)
 *
 * @param cfg SD card configuration
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if esp_vfs_fat_sdmmc_mount was already called
 *      - ESP_ERR_NO_MEM if memory cannot be allocated
 *      - ESP_FAIL if partition cannot be mounted
 *      - other error codes from SDMMC or SPI drivers, SDMMC protocol, or FATFS drivers
 */
esp_err_t bsp_sdcard_sdspi_mount(bsp_sdcard_cfg_t *cfg);

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
#define BSP_LCD_PIXEL_CLOCK_HZ     (40 * 1000 * 1000)
#define BSP_LCD_SPI_NUM            (SPI3_HOST)

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

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

#ifdef __cplusplus
}
#endif
