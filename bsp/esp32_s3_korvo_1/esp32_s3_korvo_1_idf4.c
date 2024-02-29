/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "bsp/esp32_s3_korvo_1.h"
#include "bsp_err_check.h"
#include "driver/adc.h"
#include "hal/i2s_types.h"
#include "esp_adc_cal.h"
#include "esp_codec_dev_defaults.h"

static const char *TAG = "S3-Korvo-1";

#define BSP_I2S0_SIMPLEX_MONO_CFG(_sample_rate)                      \
    {                                                                 \
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,                        \
        .sample_rate = _sample_rate,                                  \
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,                 \
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,                  \
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,            \
        .dma_buf_count = 3,                                           \
        .dma_buf_len = 1024,                                          \
        .tx_desc_auto_clear = true,                                   \
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM \
    }

/* This board has 3 microphones, therefore using I2S TDM mode, it allows using more than 2 channels */
#define BSP_I2S1_CFG(_sample_rate)                      \
    {                                                                 \
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,                        \
        .sample_rate = _sample_rate,                                  \
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,                 \
        .channel_format = I2S_CHANNEL_FMT_MULTIPLE,                   \
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,            \
        .dma_buf_count = 6,                                           \
        .dma_buf_len = 1024,                                          \
        .chan_mask = I2S_TDM_ACTIVE_CH0 | I2S_TDM_ACTIVE_CH1,         \
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM \
    }

static const audio_codec_data_if_t *i2s_data_if_spk = NULL;  /* Codec data interface */
static const audio_codec_data_if_t *i2s_data_if_mic = NULL;  /* Codec data interface */
static esp_adc_cal_characteristics_t bsp_adc_chars;

esp_err_t bsp_audio_init(const i2s_config_t *i2s_config)
{
    esp_err_t ret = ESP_FAIL;

    if (i2s_data_if_spk != NULL) {
        /* Audio was initialized before */
        return ESP_OK;
    }

    /* Setup I2S peripheral */
    const i2s_pin_config_t i2s_pin_config_spk = {
        .mck_io_num = BSP_I2S0_MCLK,
        .bck_io_num = BSP_I2S0_SCLK,
        .ws_io_num = BSP_I2S0_LCLK,
        .data_out_num = BSP_I2S0_DOUT,
        .data_in_num = BSP_I2S0_DSIN
    };

    const i2s_pin_config_t i2s_pin_config_mic = {
        .mck_io_num = BSP_I2S1_MCLK,
        .bck_io_num = BSP_I2S1_SCLK,
        .ws_io_num = BSP_I2S1_LCLK,
        .data_out_num = BSP_I2S1_DOUT,
        .data_in_num = BSP_I2S1_DSIN
    };

    /* Setup I2S channels */

    /* I2S_NUM_0 */
    const i2s_config_t std_cfg_default = BSP_I2S0_SIMPLEX_MONO_CFG(16000);
    const i2s_config_t *p_i2s_cfg = &std_cfg_default;
    if (i2s_config != NULL) {
        p_i2s_cfg = i2s_config;
    }

    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, p_i2s_cfg, 0, NULL));
    ESP_GOTO_ON_ERROR(i2s_set_pin(I2S_NUM_0, &i2s_pin_config_spk), err, TAG, "I2S set pin failed");

    audio_codec_i2s_cfg_t i2s_cfg_spk = {
        .port = I2S_NUM_0,
    };
    i2s_data_if_spk = audio_codec_new_i2s_data(&i2s_cfg_spk);
    BSP_NULL_CHECK_GOTO(i2s_data_if_spk, err);

    /* I2S_NUM_1 */
    const i2s_config_t tdm_cfg_default = BSP_I2S1_CFG(16000);
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_1, &tdm_cfg_default, 0, NULL));
    ESP_GOTO_ON_ERROR(i2s_set_pin(I2S_NUM_1, &i2s_pin_config_mic), err, TAG, "I2S set pin failed");

    audio_codec_i2s_cfg_t i2s_cfg_mic = {
        .port = I2S_NUM_1,
    };
    i2s_data_if_mic = audio_codec_new_i2s_data(&i2s_cfg_mic);
    BSP_NULL_CHECK_GOTO(i2s_data_if_mic, err);

    return ESP_OK;

err:
    i2s_driver_uninstall(I2S_NUM_0);
    i2s_driver_uninstall(I2S_NUM_1);
    return ret;
}

const audio_codec_data_if_t *bsp_audio_get_codec_itf_spk(void)
{
    return i2s_data_if_spk;
}

const audio_codec_data_if_t *bsp_audio_get_codec_itf_mic(void)
{
    return i2s_data_if_mic;
}

esp_err_t bsp_adc_initialize(void)
{
    esp_err_t ret = ESP_OK;
    BSP_ERROR_CHECK_RETURN_ERR(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP_FIT));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &bsp_adc_chars);

    /* ADC1 config */
    BSP_ERROR_CHECK_RETURN_ERR(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    return ret;
}
