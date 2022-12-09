/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

/**
 * @brief I2C address of the ES7210
 *
 * The 8-bit address format is as follows:
 *
 *                (Slave Address)
 *     ┌─────────────────┷─────────────────┐
 *  ┌─────┐─────┐─────┐─────┐─────┐─────┐─────┐─────┐
 *  |  1  |  0  |  0  |  0  |  0  | AD1 | AD0 | R/W |
 *  └─────┘─────┘─────┘─────┘─────┘─────┘─────┘─────┘
 *     └────────┯────────┘           └───┯───┘
 *           (Fixed)          (Hardware Selectable)
 *
 * And the 7-bit slave address is the most important data for users.
 * For example, if the chip's AD0,AD1 are connected to GND, its 7-bit slave address is 1000000b(0x40).
 * Then users can use `ES7210_ADDRRES_00` to init it.
 */
#define ES7210_ADDRRES_00 (0x40)
#define ES7210_ADDRESS_01 (0x41)
#define ES7210_ADDRESS_10 (0x42)
#define ES7210_ADDRESS_11 (0x43)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Select I2S interface format for ES7210
 */
typedef enum {
    ES7210_I2S_FMT_I2S   = 0x00,    /*!< normal I2S format */
    ES7210_I2S_FMT_LJ    = 0x01,    /*!< left justify format */
    ES7210_I2S_FMT_DSP_A = 0x03,    /*!< DSP-A format, MSB is available on 2nd SCLK rising edge after LRCK rising edge */
    ES7210_I2S_FMT_DSP_B = 0x13     /*!< DSP-B format, MSB is available on 1st SCLK rising edge after LRCK rising edge */
} es7210_i2s_fmt_t;

/**
 * @brief Select I2S bit width for ES7210
 *
 */
typedef enum {
    ES7210_I2S_BITS_16B = 16,       /*!< 16-bit I2S mode */
    ES7210_I2S_BITS_18B = 18,       /*!< 18-bit I2S mode */
    ES7210_I2S_BITS_20B = 20,       /*!< 20-bit I2S mode */
    ES7210_I2S_BITS_24B = 24,       /*!< 24-bit I2S mode */
    ES7210_I2S_BITS_32B = 32        /*!< 32-bit I2S mode */
} es7210_i2s_bits_t;

/**
 * @brief Select MIC gain for ES7210
 *
 */
typedef enum {
    ES7210_MIC_GAIN_0DB  = 0,      /*!< 0dB MIC gain */
    ES7210_MIC_GAIN_3DB  = 1,      /*!< 3dB MIC gain */
    ES7210_MIC_GAIN_6DB  = 2,      /*!< 6dB MIC gain */
    ES7210_MIC_GAIN_9DB  = 3,      /*!< 9dB MIC gain */
    ES7210_MIC_GAIN_12DB = 4,      /*!< 12dB MIC gain */
    ES7210_MIC_GAIN_15DB = 5,      /*!< 15dB MIC gain */
    ES7210_MIC_GAIN_18DB = 6,      /*!< 18dB MIC gain */
    ES7210_MIC_GAIN_21DB = 7,      /*!< 21dB MIC gain */
    ES7210_MIC_GAIN_24DB = 8,      /*!< 24dB MIC gain */
    ES7210_MIC_GAIN_27DB = 9,      /*!< 27dB MIC gain */
    ES7210_MIC_GAIN_30DB = 10,     /*!< 30dB MIC gain */
    ES7210_MIC_GAIN_33DB = 11,     /*!< 33dB MIC gain */
    ES7210_MIC_GAIN_34_5DB = 12,   /*!< 34.5dB MIC gain */
    ES7210_MIC_GAIN_36DB = 13,     /*!< 36dB MIC gain */
    ES7210_MIC_GAIN_37_5DB = 14    /*!< 37.5dB MIC gain */
} es7210_mic_gain_t;

/**
 * @brief Select MIC bias for ES7210
 *
 */
typedef enum {
    ES7210_MIC_BIAS_2V18 = 0x00,   /*!< 2.18V MIC bias */
    ES7210_MIC_BIAS_2V26 = 0x10,   /*!< 2.26V MIC bias */
    ES7210_MIC_BIAS_2V36 = 0x20,   /*!< 2.36V MIC bias */
    ES7210_MIC_BIAS_2V45 = 0x30,   /*!< 2.45V MIC bias */
    ES7210_MIC_BIAS_2V55 = 0x40,   /*!< 2.55V MIC bias */
    ES7210_MIC_BIAS_2V66 = 0x50,   /*!< 2.66V MIC bias */
    ES7210_MIC_BIAS_2V78 = 0x60,   /*!< 2.78V MIC bias */
    ES7210_MIC_BIAS_2V87 = 0x70    /*!< 2.87V MIC bias */
} es7210_mic_bias_t;

/**
 * @brief Type of es7210 device handle
 *
 */
typedef struct es7210_dev_t *es7210_dev_handle_t;

/**
 * @brief ES7210 I2C config struct
 *
 */
typedef struct {
    i2c_port_t  i2c_port;           /*!< I2C port used to connecte ES7210 device */
    uint8_t     i2c_addr;           /*!< I2C address of ES7210 device, can be 0x40 0x41 0x42 or 0x43 according to A0 and A1 pin */
} es7210_i2c_config_t;

/**
 * @brief ES7210 codec config struct
 *
 */
typedef struct {
    uint32_t sample_rate_hz;        /*!< Sample rate in Hz, common values are supported */
    uint32_t mclk_ratio;            /*!< MCLK-to-Sample-rate clock ratio, typically 256 */
    es7210_i2s_fmt_t i2s_format;    /*!< I2S format of ES7210's output, can be any value in es7210_i2s_fmt_t */
    es7210_i2s_bits_t bit_width;    /*!< I2S bit width of ES7210's output, can be any value in es7210_i2s_bits_t */
    es7210_mic_bias_t mic_bias;     /*!< Bias volatge of analog MIC, please refer to your MIC's datasheet */
    es7210_mic_gain_t mic_gain;     /*!< Gain of analog MIC, please adjust according to your MIC's sensitivity */
    struct {
        uint32_t tdm_enable: 1;     /*!< Choose whether to enable TDM mode */
    } flags;
} es7210_codec_config_t;

/**
 * @brief Create new ES7210 device handle.
 *
 * @param[in]  i2c_conf Config for I2C used by ES7210
 * @param[out] handle_out New ES7210 device handle
 * @return
 *          - ESP_OK                  Device handle creation success.
 *          - ESP_ERR_INVALID_ARG     Invalid device handle or argument.
 *          - ESP_ERR_NO_MEM          Memory allocation failed.
 *
 */
esp_err_t es7210_new_codec(const es7210_i2c_config_t *i2c_conf, es7210_dev_handle_t *handle_out);

/**
 * @brief Delete ES7210 device handle.
 *
 * @param[in] handle ES7210 device handle
 * @return
 *          - ESP_OK                  Device handle deletion success.
 *          - ESP_ERR_INVALID_ARG     Invalid device handle or argument.
 *
 */
esp_err_t es7210_del_codec(es7210_dev_handle_t handle);

/**
 * @brief Configure codec-related parameters of ES7210.
 *
 * @param[in] handle ES7210 device handle
 * @param[in] codec_conf codec-related parameters of ES7210
 * @return
 *          - ESP_OK                  Codec config success.
 *          - ESP_ERR_INVALID_ARG     Invalid device handle or argument.
 *          - ESP_ERR_NO_MEM          Memory allocation failed.
 *          - ESP_FAIL                Sending command error, slave hasn't ACK the transfer.
 *          - ESP_ERR_INVALID_STATE   I2C driver not installed or not in master mode.
 *          - ESP_ERR_TIMEOUT         Operation timeout because the bus is busy.
 *
 */
esp_err_t es7210_config_codec(es7210_dev_handle_t handle, const es7210_codec_config_t *codec_conf);

/**
 * @brief Configure volume of ES7210.
 *
 * @param[in] handle ES7210 device handle
 * @param[in] volume_db Volume to be set, in dB, with a range from -95dB to +32dB.
 * @return
 *          - ESP_OK                  Volume config success.
 *          - ESP_ERR_INVALID_ARG     Invalid device handle or argument.
 *          - ESP_ERR_NO_MEM          Memory allocation failed.
 *          - ESP_FAIL                Sending command error, slave hasn't ACK the transfer.
 *          - ESP_ERR_INVALID_STATE   I2C driver not installed or not in master mode.
 *          - ESP_ERR_TIMEOUT         Operation timeout because the bus is busy.
 *
 */
esp_err_t es7210_config_volume(es7210_dev_handle_t handle, int8_t volume_db);

#ifdef __cplusplus
}
#endif
