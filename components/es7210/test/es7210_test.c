/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "es7210.h"
#include "esp_system.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 18      /*!< gpio number for I2C master clock on S3-Korvo */
#define I2C_MASTER_SDA_IO 17      /*!< gpio number for I2C master data  on S3-Korvo */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */

#define ES7210_I2C_ADDR             (0x40)
#define ES7210_SAMPLE_RATE          (48000)
#define ES7210_I2S_FORMAT           ES7210_I2S_FMT_I2S
#define ES7210_MCLK_MULTIPLE        (256)
#define ES7210_BIT_WIDTH            ES7210_I2S_BITS_16B
#define ES7210_MIC_BIAS             ES7210_MIC_BIAS_2V87
#define ES7210_MIC_GAIN             ES7210_MIC_GAIN_30DB
#define ES7210_ADC_VOLUME           (0)

static const char *TAG = "es7210 test";
static es7210_dev_handle_t es7210_handle = NULL;

#define TEST_ERROR_CHECK(res, format)      TEST_ASSERT_EQUAL_MESSAGE(res, ESP_OK, format)

static void test_i2c_init(void)
{
    i2c_config_t i2c_conf = {
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .mode = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    TEST_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &i2c_conf), "Failed to config I2C params");
    TEST_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0), "Failed to install I2C driver");
}

static void test_es7210_init(bool is_tdm)
{
    /* Create ES7210 device handle */
    es7210_i2c_config_t es7210_i2c_conf = {
        .i2c_port = I2C_MASTER_NUM,
        .i2c_addr = ES7210_I2C_ADDR
    };
    TEST_ERROR_CHECK(es7210_new_codec(&es7210_i2c_conf, &es7210_handle), "Failed to allocate new ES7210 codec");

    ESP_LOGI(TAG, "Configure ES7210 codec parameters");
    es7210_codec_config_t codec_conf = {
        .i2s_format = ES7210_I2S_FORMAT,
        .mclk_ratio = ES7210_MCLK_MULTIPLE,
        .sample_rate_hz = ES7210_SAMPLE_RATE,
        .bit_width = ES7210_BIT_WIDTH,
        .mic_bias = ES7210_MIC_BIAS,
        .mic_gain = ES7210_MIC_GAIN,
        .flags.tdm_enable = is_tdm
    };
    TEST_ERROR_CHECK(es7210_config_codec(es7210_handle, &codec_conf), "Failed to config ES7210");
    TEST_ERROR_CHECK(es7210_config_volume(es7210_handle, ES7210_ADC_VOLUME), "Failed  to config the volume of ES7210");
}

TEST_CASE("ADC Codec ES7210 basic functionality test", "[es7210][iot][device]")
{
    test_i2c_init();
    test_es7210_init(false);
    TEST_ERROR_CHECK(es7210_del_codec(es7210_handle), "Failed to delete ES7210 handle");
    test_es7210_init(true);
    TEST_ERROR_CHECK(es7210_del_codec(es7210_handle), "Failed to delete ES7210 handle");
    TEST_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM), "Failed to delete I2C driver");
}
