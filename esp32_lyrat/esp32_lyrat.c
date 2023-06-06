/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "bsp/esp32_lyrat.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"

static const char *TAG = "LyraT";

sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler

static esp_err_t  bsp_touchpad_custom_init(void *param);
static esp_err_t  bsp_touchpad_custom_deinit(void *param);
static uint8_t bsp_touchpad_custom_get_key_value(void *param);
static bool i2c_initialized = false;

static const button_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config.gpio_num = BSP_BUTTON_REC_IO,
        .gpio_button_config.active_level = 0,
    },
    {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config.gpio_num = BSP_BUTTON_MODE_IO,
        .gpio_button_config.active_level = 0,
    },
    {
        .type = BUTTON_TYPE_CUSTOM,
        .custom_button_config.button_custom_init = bsp_touchpad_custom_init,
        .custom_button_config.button_custom_get_key_value = bsp_touchpad_custom_get_key_value,
        .custom_button_config.button_custom_deinit = bsp_touchpad_custom_deinit,
        .custom_button_config.priv = (void *)BSP_BUTTON_PLAY_TOUCH,
    },
    {
        .type = BUTTON_TYPE_CUSTOM,
        .custom_button_config.button_custom_init = bsp_touchpad_custom_init,
        .custom_button_config.button_custom_get_key_value = bsp_touchpad_custom_get_key_value,
        .custom_button_config.button_custom_deinit = bsp_touchpad_custom_deinit,
        .custom_button_config.priv = (void *)BSP_BUTTON_SET_TOUCH,
    },
    {
        .type = BUTTON_TYPE_CUSTOM,
        .custom_button_config.button_custom_init = bsp_touchpad_custom_init,
        .custom_button_config.button_custom_get_key_value = bsp_touchpad_custom_get_key_value,
        .custom_button_config.button_custom_deinit = bsp_touchpad_custom_deinit,
        .custom_button_config.priv = (void *)BSP_BUTTON_VOLUP_TOUCH,
    },
    {
        .type = BUTTON_TYPE_CUSTOM,
        .custom_button_config.button_custom_init = bsp_touchpad_custom_init,
        .custom_button_config.button_custom_get_key_value = bsp_touchpad_custom_get_key_value,
        .custom_button_config.button_custom_deinit = bsp_touchpad_custom_deinit,
        .custom_button_config.priv = (void *)BSP_BUTTON_VOLDOWN_TOUCH,
    }
};

esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = BSP_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = CONFIG_BSP_I2C_CLK_SPEED_HZ
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_param_config(BSP_I2C_NUM, &i2c_conf));
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_install(BSP_I2C_NUM, i2c_conf.mode, 0, 0, 0));

    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_delete(BSP_I2C_NUM));
    i2c_initialized = false;
    return ESP_OK;
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    BSP_ERROR_CHECK_RETURN_ERR(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

esp_err_t bsp_sdcard_mount(void)
{
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    const sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    const sdmmc_slot_config_t slot_config = {
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP,
        .width = 1,
        .flags = 0,
    };

    return esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
}

static esp_codec_dev_handle_t bsp_audio_codec_init(void)
{
    static esp_codec_dev_handle_t codec = NULL;
    if (codec) {
        return codec;
    }

    const audio_codec_data_if_t *i2s_data_if = bsp_audio_get_codec_itf();
    if (i2s_data_if == NULL) {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_audio_init(NULL));
        i2s_data_if = bsp_audio_get_codec_itf();
    }
    assert(i2s_data_if);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8388_CODEC_DEFAULT_ADDR,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };

    es8388_codec_cfg_t es8388_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .hw_gain = gain,
    };
    const audio_codec_if_t *es8388_dev = es8388_codec_new(&es8388_cfg);
    BSP_NULL_CHECK(es8388_dev, NULL);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = es8388_dev,
        .data_if = i2s_data_if,
    };
    codec = esp_codec_dev_new(&codec_dev_cfg);
    BSP_NULL_CHECK(codec, NULL);

    return codec;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    return bsp_audio_codec_init();
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    return bsp_audio_codec_init();
}

#define TOUCHPAD_FILTER_TOUCH_PERIOD (50)
#define TOUCHPAD_THRESH              (400)

static esp_err_t bsp_touchpad_custom_init(void *param)
{
    static bool touch_pad_initialized = false;
    touch_pad_t btn = (touch_pad_t)param;

    if (!touch_pad_initialized) {
        /*!< Initialize touch pad peripheral, it will start a timer to run a filter */
        BSP_ERROR_CHECK_RETURN_ERR(touch_pad_init());

        // Set reference voltage for charging/discharging
        // In this case, the high reference valtage will be 2.7V - 1V = 1.7V
        // The low reference voltage will be 0.5
        // The larger the range, the larger the pulse count value.
        BSP_ERROR_CHECK_RETURN_ERR(touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V));
        BSP_ERROR_CHECK_RETURN_ERR(touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD));

        touch_pad_initialized = true;
    }
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_config(btn, 0));

    ESP_LOGI(TAG, "Initialized touch button %d", btn);

    return ESP_OK;
}

static esp_err_t  bsp_touchpad_custom_deinit(void *param)
{
    //touch_pad_t btn = (touch_pad_t)param;

    return ESP_OK;
}

static uint8_t bsp_touchpad_custom_get_key_value(void *param)
{
    touch_pad_t btn = (touch_pad_t)param;
    uint16_t touch_value;

    touch_pad_read_raw_data(btn, &touch_value);

    return (touch_value > 0 && touch_value < TOUCHPAD_THRESH ? 0 : 1);
}

esp_err_t bsp_leds_init(void)
{
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BIT64(BSP_LED_GREEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    BSP_ERROR_CHECK_RETURN_ERR(gpio_config(&led_io_config));
    return ESP_OK;
}

esp_err_t bsp_led_set(const bsp_led_t led_io, const bool on)
{
    BSP_ERROR_CHECK_RETURN_ERR(gpio_set_level((gpio_num_t) led_io, (uint32_t) on));
    return ESP_OK;
}

esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size)
{
    esp_err_t ret = ESP_OK;
    if ((btn_array_size < BSP_BUTTON_NUM) ||
            (btn_array == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (btn_cnt) {
        *btn_cnt = 0;
    }
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        btn_array[i] = iot_button_create(&bsp_button_config[i]);
        if (btn_array[i] == NULL) {
            ret = ESP_FAIL;
            break;
        }
        if (btn_cnt) {
            (*btn_cnt)++;
        }
    }
    return ret;
}
