/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/i2s_std.h"
#include "bsp/esp32_s2_kaluga_kit.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"

static const audio_codec_data_if_t *i2s_data_if = NULL;  /* Codec data interface */
static adc_oneshot_unit_handle_t bsp_adc_handle = NULL;

/**
 * @brief Kaluga-kit I2S pinout
 *
 * Can be used for i2s_std_gpio_config_t and/or i2s_std_config_t initialization
 */
#define BSP_I2S_GPIO_CFG       \
    {                          \
        .mclk = BSP_I2S_MCLK,  \
        .bclk = BSP_I2S_SCLK,  \
        .ws = BSP_I2S_LCLK,    \
        .dout = BSP_I2S_DOUT,  \
        .din = BSP_I2S_DSIN,   \
        .invert_flags = {      \
            .mclk_inv = false, \
            .bclk_inv = false, \
            .ws_inv = false,   \
        },                     \
    }

/**
 * @brief Mono Duplex I2S configuration structure
 *
 * This configuration is used by default in bsp_audio_init()
 */
#define BSP_I2S_DUPLEX_MONO_CFG(_sample_rate)                                                         \
    {                                                                                                 \
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(_sample_rate),                                          \
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO), \
        .gpio_cfg = BSP_I2S_GPIO_CFG,                                                                 \
    }

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
{
    if (i2s_data_if) {
        /* Audio was initialized before */
        return ESP_OK;
    }

    i2s_chan_handle_t tx_channel, rx_channel;
    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    BSP_ERROR_CHECK_RETURN_ERR(i2s_new_channel(&chan_cfg, &tx_channel, &rx_channel));

    /* Setup I2S channels */
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(22050);
    const i2s_std_config_t *p_i2s_cfg = &std_cfg_default;
    if (i2s_config != NULL) {
        p_i2s_cfg = i2s_config;
    }

    if (tx_channel != NULL) {
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_init_std_mode(tx_channel, p_i2s_cfg));
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_enable(tx_channel));
    }
    if (rx_channel != NULL) {
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_init_std_mode(rx_channel, p_i2s_cfg));
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_enable(rx_channel));
    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = rx_channel,
        .tx_handle = tx_channel,
    };

    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    BSP_NULL_CHECK(i2s_data_if, ESP_FAIL);
    return ESP_OK;
}

const audio_codec_data_if_t *bsp_audio_get_codec_itf(void)
{
    return i2s_data_if;
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
