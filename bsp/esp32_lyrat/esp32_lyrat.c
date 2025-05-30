/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "bsp/esp32_lyrat.h"
#include "esp_vfs_fat.h"
#include "esp_spiffs.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"
#include "button_gpio.h"

static const char *TAG = "LyraT";

static sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler
static bool spi_sd_initialized = false;

static esp_err_t  bsp_touchpad_custom_deinit(button_driver_t *button_driver);
static uint8_t bsp_touchpad_custom_get_key_value(button_driver_t *button_driver);
static bool i2c_initialized = false;

typedef enum {
    BSP_BUTTON_TYPE_GPIO,
    BSP_BUTTON_TYPE_CUSTOM
} bsp_button_type_t;

typedef struct {
    button_driver_t base;
    touch_pad_t     key;
} bsp_button_type_custom_t;

typedef struct {
    bsp_button_type_t type;
    union {
        button_gpio_config_t     gpio;
        bsp_button_type_custom_t custom;
    } cfg;
} bsp_button_config_t;

static const bsp_button_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .type = BSP_BUTTON_TYPE_GPIO,
        .cfg.gpio = {
            .gpio_num = BSP_BUTTON_REC_IO,
            .active_level = 0,
        }
    },
    {
        .type = BSP_BUTTON_TYPE_GPIO,
        .cfg.gpio = {
            .gpio_num = BSP_BUTTON_MODE_IO,
            .active_level = 0,
        }
    },
    {
        .type = BSP_BUTTON_TYPE_CUSTOM,
        .cfg.custom = {
            .base = {
                .get_key_level = bsp_touchpad_custom_get_key_value,
                .del = bsp_touchpad_custom_deinit,
            },
            .key = BSP_BUTTON_PLAY_TOUCH
        }
    },
    {
        .type = BSP_BUTTON_TYPE_CUSTOM,
        .cfg.custom = {
            .base = {
                .get_key_level = bsp_touchpad_custom_get_key_value,
                .del = bsp_touchpad_custom_deinit,
            },
            .key = BSP_BUTTON_SET_TOUCH
        }
    },
    {
        .type = BSP_BUTTON_TYPE_CUSTOM,
        .cfg.custom = {
            .base = {
                .get_key_level = bsp_touchpad_custom_get_key_value,
                .del = bsp_touchpad_custom_deinit,
            },
            .key = BSP_BUTTON_VOLUP_TOUCH
        }
    },
    {
        .type = BSP_BUTTON_TYPE_CUSTOM,
        .cfg.custom = {
            .base = {
                .get_key_level = bsp_touchpad_custom_get_key_value,
                .del = bsp_touchpad_custom_deinit,
            },
            .key = BSP_BUTTON_VOLDOWN_TOUCH
        }
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

sdmmc_card_t *bsp_sdcard_get_handle(void)
{
    return bsp_sdcard;
}

void bsp_sdcard_get_sdmmc_host(const int slot, sdmmc_host_t *config)
{
    assert(config);

    sdmmc_host_t host_config = SDMMC_HOST_DEFAULT();

    memcpy(config, &host_config, sizeof(sdmmc_host_t));
}

void bsp_sdcard_get_sdspi_host(const int slot, sdmmc_host_t *config)
{
    assert(config);

    sdmmc_host_t host_config = SDSPI_HOST_DEFAULT();
    host_config.slot = slot;

    memcpy(config, &host_config, sizeof(sdmmc_host_t));
}

void bsp_sdcard_sdmmc_get_slot(const int slot, sdmmc_slot_config_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdmmc_slot_config_t));

    /* SD card is connected to Slot 0 pins. Slot 0 uses IO MUX, so not specifying the pins here */
    config->cd = SDMMC_SLOT_NO_CD;
    config->wp = SDMMC_SLOT_NO_WP;
    config->width = 1;
    config->flags = SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
}

void bsp_sdcard_sdspi_get_slot(const spi_host_device_t spi_host, sdspi_device_config_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdspi_device_config_t));

    config->gpio_cs   = BSP_SD_SPI_CS;
    config->gpio_cd   = SDSPI_SLOT_NO_CD;
    config->gpio_wp   = SDSPI_SLOT_NO_WP;
    config->gpio_int  = GPIO_NUM_NC;
    config->host_id = spi_host;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
    config->gpio_wp_polarity = SDSPI_IO_ACTIVE_LOW;
#endif
}

esp_err_t bsp_sdcard_sdmmc_mount(bsp_sdcard_cfg_t *cfg)
{
    sdmmc_host_t sdhost = {0};
    sdmmc_slot_config_t sdslot = {0};
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    assert(cfg);

    if (!cfg->mount) {
        cfg->mount = &mount_config;
    }

    if (!cfg->host) {
        bsp_sdcard_get_sdmmc_host(SDMMC_HOST_SLOT_0, &sdhost);
        cfg->host = &sdhost;
    }

    if (!cfg->slot.sdmmc) {
        bsp_sdcard_sdmmc_get_slot(SDMMC_HOST_SLOT_0, &sdslot);
        cfg->slot.sdmmc = &sdslot;
    }

#if !CONFIG_FATFS_LONG_FILENAMES
    ESP_LOGW(TAG, "Warning: Long filenames on SD card are disabled in menuconfig!");
#endif

    return esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, cfg->host, cfg->slot.sdmmc, cfg->mount, &bsp_sdcard);
}

esp_err_t bsp_sdcard_sdspi_mount(bsp_sdcard_cfg_t *cfg)
{
    sdmmc_host_t sdhost = {0};
    sdspi_device_config_t sdslot = {0};
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    assert(cfg);

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num     = BSP_SD_SPI_CLK,
        .mosi_io_num     = BSP_SD_SPI_MOSI,
        .miso_io_num     = BSP_SD_SPI_MISO,
        .quadwp_io_num   = GPIO_NUM_NC,
        .quadhd_io_num   = GPIO_NUM_NC,
        .max_transfer_sz = 4000,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_SDSPI_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");
    spi_sd_initialized = true;

    if (!cfg->mount) {
        cfg->mount = &mount_config;
    }

    if (!cfg->host) {
        bsp_sdcard_get_sdspi_host(SDMMC_HOST_SLOT_0, &sdhost);
        cfg->host = &sdhost;
    }

    if (!cfg->slot.sdspi) {
        bsp_sdcard_sdspi_get_slot(BSP_SDSPI_HOST, &sdslot);
        cfg->slot.sdspi = &sdslot;
    }

#if !CONFIG_FATFS_LONG_FILENAMES
    ESP_LOGW(TAG, "Warning: Long filenames on SD card are disabled in menuconfig!");
#endif

    ESP_RETURN_ON_ERROR(esp_vfs_fat_sdspi_mount(BSP_SD_MOUNT_POINT, cfg->host, cfg->slot.sdspi, cfg->mount, &bsp_sdcard), TAG, "SD card SPI mount failed, please check JP8. Pin 2 must be switched to ON.");
    return ESP_OK;
}

esp_err_t bsp_sdcard_mount(void)
{
    bsp_sdcard_cfg_t cfg = {0};
    return bsp_sdcard_sdmmc_mount(&cfg);
}

esp_err_t bsp_sdcard_unmount(void)
{
    esp_err_t ret = ESP_OK;

    ret |= esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
    bsp_sdcard = NULL;

    if (spi_sd_initialized) {
        ret |= spi_bus_free(BSP_SDSPI_HOST);
        spi_sd_initialized = false;
    }

    return ret;
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

static esp_err_t bsp_touchpad_custom_init(touch_pad_t key)
{
    static bool touch_pad_initialized = false;

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
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_config(key, 0));

    ESP_LOGI(TAG, "Initialized touch button %d", key);

    return ESP_OK;
}

static esp_err_t  bsp_touchpad_custom_deinit(button_driver_t *button_driver)
{
    return ESP_OK;
}

static uint8_t bsp_touchpad_custom_get_key_value(button_driver_t *button_driver)
{
    bsp_button_type_custom_t *custom_btn = __containerof(button_driver, bsp_button_type_custom_t, base);
    uint16_t touch_value;

    touch_pad_read_raw_data(custom_btn->key, &touch_value);

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
    const button_config_t btn_config = {0};
    if ((btn_array_size < BSP_BUTTON_NUM) ||
            (btn_array == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (btn_cnt) {
        *btn_cnt = 0;
    }
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        if (bsp_button_config[i].type == BSP_BUTTON_TYPE_CUSTOM) {
            ret |= bsp_touchpad_custom_init(bsp_button_config[i].cfg.custom.key);
            ret |= iot_button_create(&btn_config, &bsp_button_config[i].cfg.custom.base, &btn_array[i]);
        } else if (bsp_button_config[i].type == BSP_BUTTON_TYPE_GPIO) {
            ret |= iot_button_new_gpio_device(&btn_config, &bsp_button_config[i].cfg.gpio, &btn_array[i]);
        } else {
            ESP_LOGW(TAG, "Unsupported button type!");
        }
        if (btn_cnt) {
            (*btn_cnt)++;
        }
    }
    return ret;
}
