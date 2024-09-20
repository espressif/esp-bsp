/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "bsp/esp32_s3_korvo_1.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"

static const char *TAG = "S3-Korvo-1";

static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;
static const audio_codec_data_if_t *i2s_data_if_spk = NULL;  /* Codec data interface */
static const audio_codec_data_if_t *i2s_data_if_mic = NULL;  /* Codec data interface */
static adc_oneshot_unit_handle_t bsp_adc_handle = NULL;

/* Can be used for i2s_std_gpio_config_t and/or i2s_std_config_t initialization */
#define BSP_I2S0_GPIO_CFG       \
    {                          \
        .mclk = BSP_I2S0_MCLK,  \
        .bclk = BSP_I2S0_SCLK,  \
        .ws = BSP_I2S0_LCLK,    \
        .dout = BSP_I2S0_DOUT,  \
        .din = BSP_I2S0_DSIN,   \
        .invert_flags = {      \
            .mclk_inv = false, \
            .bclk_inv = false, \
            .ws_inv = false,   \
        },                     \
    }

#define BSP_I2S1_GPIO_CFG       \
    {                          \
        .mclk = BSP_I2S1_MCLK,  \
        .bclk = BSP_I2S1_SCLK,  \
        .ws = BSP_I2S1_LCLK,    \
        .dout = BSP_I2S1_DOUT,  \
        .din = BSP_I2S1_DSIN,   \
        .invert_flags = {      \
            .mclk_inv = false, \
            .bclk_inv = false, \
            .ws_inv = false,   \
        },                     \
    }

/* This configuration is used by default in bsp_audio_init() */
#define BSP_I2S0_SIMPLEX_MONO_CFG(_sample_rate)                                                         \
    {                                                                                                 \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                          \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S0_GPIO_CFG,                                                                 \
    }

/* This board has 3 microphones, therefore using I2S TDM mode, it allows using more than 2 channels */
#define BSP_I2S1_CFG(_sample_rate)                                                                      \
    {                                                                                                 \
        .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(_sample_rate),                                          \
        .slot_cfg = I2S_TDM_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_TDM_SLOT0 | I2S_TDM_SLOT1),\
        .gpio_cfg = BSP_I2S1_GPIO_CFG,                                                                 \
    }

static esp_err_t bsp_i2s_chan_init(int i2s_num, i2s_chan_handle_t *tx_chan, i2s_chan_handle_t *rx_chan)
{
    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_num, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    return i2s_new_channel(&chan_cfg, tx_chan, rx_chan);
}

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
{
    esp_err_t ret = ESP_FAIL;
    if (i2s_tx_chan && i2s_rx_chan) {
        /* Audio was initialized before */
        return ESP_OK;
    }

    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2s_chan_init(I2S_NUM_0, &i2s_tx_chan, NULL));
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2s_chan_init(I2S_NUM_1, NULL, &i2s_rx_chan));

    /* Setup I2S channels */
    const i2s_std_config_t std_cfg_default = BSP_I2S0_SIMPLEX_MONO_CFG(22050);
    const i2s_std_config_t *p_i2s_cfg = &std_cfg_default;
    if (i2s_config != NULL) {
        p_i2s_cfg = i2s_config;
    }

    const i2s_tdm_config_t tdm_cfg_default = BSP_I2S1_CFG(16000);

    if (i2s_tx_chan != NULL) {
        ESP_GOTO_ON_ERROR(i2s_channel_init_std_mode(i2s_tx_chan, p_i2s_cfg), err, TAG, "I2S channel initialization failed");
        ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_tx_chan), err, TAG, "I2S enabling failed");
    }
    if (i2s_rx_chan != NULL) {
        ESP_GOTO_ON_ERROR(i2s_channel_init_tdm_mode(i2s_rx_chan, &tdm_cfg_default), err, TAG, "I2S channel initialization failed");
        ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_rx_chan), err, TAG, "I2S enabling failed");
    }

    audio_codec_i2s_cfg_t i2s_cfg_spk = {
        .port = I2S_NUM_0,
        .rx_handle = NULL,
        .tx_handle = i2s_tx_chan,
    };
    i2s_data_if_spk = audio_codec_new_i2s_data(&i2s_cfg_spk);
    BSP_NULL_CHECK_GOTO(i2s_data_if_spk, err);

    audio_codec_i2s_cfg_t i2s_cfg_mic = {
        .port = I2S_NUM_1,
        .rx_handle = i2s_rx_chan,
        .tx_handle = NULL,
    };
    i2s_data_if_mic = audio_codec_new_i2s_data(&i2s_cfg_mic);
    BSP_NULL_CHECK_GOTO(i2s_data_if_mic, err);

    return ESP_OK;

err:
    if (i2s_tx_chan) {
        i2s_del_channel(i2s_tx_chan);
    }
    if (i2s_rx_chan) {
        i2s_del_channel(i2s_rx_chan);
    }

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
    /* ADC was initialized before */
    if (bsp_adc_handle != NULL) {
        return ESP_OK;
    }

    /* Initialize ADC */
    const adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = BSP_ADC_UNIT,
    };
    BSP_ERROR_CHECK_RETURN_ERR(adc_oneshot_new_unit(&init_config1, &bsp_adc_handle));

    return ESP_OK;
}

adc_oneshot_unit_handle_t bsp_adc_get_handle(void)
{
    return bsp_adc_handle;
}
