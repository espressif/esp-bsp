/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_check.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "hal/gpio_hal.h"
#include "dummy_codec.h"
#include "esp_codec_adc.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"
#include "bsp/esp_sensairshuttle.h"

#include "driver/gpio.h"

static const char *TAG = "ESP-SensairShuttle";

#define SPEAKER_N_IO    GPIO_NUM_8

/* Can be used for i2s_pdm_tx_gpio_config_t and/or i2s_pdm_tx_config_t initialization */
#define BSP_I2S_PDM_TX_GPIO_CFG     \
    {                            \
        .clk = BSP_I2S_CLK,      \
        .dout = BSP_I2S_DOUT,    \
        .invert_flags = {        \
            .clk_inv = false,    \
        },                       \
    }

/* This configuration is used by default in bsp_audio_init() */
#define BSP_I2S_PDM_TX_DUPLEX_CFG(_sample_rate)                                         \
    {                                                                                    \
        .clk_cfg = I2S_PDM_TX_CLK_DEFAULT_CONFIG(_sample_rate),                          \
        .slot_cfg = I2S_PDM_TX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,             \
                                                   I2S_SLOT_MODE_MONO),                  \
        .gpio_cfg = BSP_I2S_PDM_TX_GPIO_CFG,                                                \
    }
static i2s_chan_handle_t i2s_tx_chan = NULL;
static const audio_codec_data_if_t *i2s_data_if = NULL;  /* Codec data interface */

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
{
    esp_err_t ret = ESP_FAIL;
    if (i2s_tx_chan) {
        /* Audio was initialized before */
        return ESP_OK;
    }

    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    BSP_ERROR_CHECK_RETURN_ERR(i2s_new_channel(&chan_cfg, &i2s_tx_chan, NULL));

    /* Setup TX I2S channels */
    const i2s_pdm_tx_config_t pdm_tx_cfg_default = BSP_I2S_PDM_TX_DUPLEX_CFG(44100);

    if (i2s_tx_chan != NULL) {
        ESP_GOTO_ON_ERROR(i2s_channel_init_pdm_tx_mode(i2s_tx_chan, &pdm_tx_cfg_default), err, TAG,
                          "I2S channel initialization failed");
        ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_tx_chan), err, TAG, "I2S enabling failed");
    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = CONFIG_BSP_I2S_NUM,
        .rx_handle = NULL,
        .tx_handle = i2s_tx_chan,
    };
    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    BSP_NULL_CHECK_GOTO(i2s_data_if, err);

    return ESP_OK;

err:
    if (i2s_tx_chan) {
        i2s_del_channel(i2s_tx_chan);
        i2s_tx_chan = NULL;
    }

    return ret;
}

const audio_codec_data_if_t *bsp_audio_get_codec_itf(void)
{
    return i2s_data_if;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    const audio_codec_data_if_t *i2s_data_if = bsp_audio_get_codec_itf();
    if (i2s_data_if == NULL) {

        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL));
        i2s_data_if = bsp_audio_get_codec_itf();
    }
    assert(i2s_data_if);
    /* Enable Feature */
    BSP_ERROR_CHECK_RETURN_NULL(bsp_feature_enable(BSP_FEATURE_SPEAKER, true));

    PIN_FUNC_SELECT(IO_MUX_GPIO8_REG, PIN_FUNC_GPIO);
    gpio_set_direction(SPEAKER_N_IO, GPIO_MODE_OUTPUT);
    esp_rom_gpio_connect_out_signal(SPEAKER_N_IO, I2SO_SD_OUT_IDX, 1, 0);
    gpio_set_drive_capability(SPEAKER_N_IO, GPIO_DRIVE_CAP_0);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = NULL,
        .data_if = i2s_data_if,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    audio_codec_adc_cfg_t adc_cfg = {
        .handle = NULL,
        .continuous_cfg = {
            .max_store_buf_size = 1024,
            .conv_frame_size    = 256,
            .sample_freq_hz     = 16000,
            .format             = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
            .pattern_num        = 1,
            .cfg_mode           = AUDIO_CODEC_ADC_CFG_MODE_SINGLE_UNIT,
            .cfg.single_unit = {
                .unit_id      = ADC_UNIT_1,
                .atten        = ADC_ATTEN_DB_12,
                .bit_width    = ADC_BITWIDTH_12,
                .channel_id   = { ADC_CHANNEL_5 },
            },
        },
    };

    const audio_codec_data_if_t *adc_data_if = audio_codec_new_adc_data(&adc_cfg);
    assert(adc_data_if);

    dummy_codec_cfg_t codec_cfg = {
        .gpio_if     = NULL,
        .pa_pin      = -1,
        .pa_reverted = false,
    };
    const audio_codec_if_t *dummy_dev = dummy_codec_new(&codec_cfg);
    if (dummy_dev == NULL) {
        ESP_LOGE(TAG, "Failed to create dummy codec");
        audio_codec_delete_data_if(adc_data_if);
        return NULL;
    }

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type  = ESP_CODEC_DEV_TYPE_IN,
        .codec_if  = dummy_dev,
        .data_if   = adc_data_if,
    };
    esp_codec_dev_handle_t handle = esp_codec_dev_new(&codec_dev_cfg);
    if (handle == NULL) {
        ESP_LOGE(TAG, "Failed to create codec device");
        audio_codec_delete_codec_if(dummy_dev);
        audio_codec_delete_data_if(adc_data_if);
    }
    return handle;
}
