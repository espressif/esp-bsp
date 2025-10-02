/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"

#include "bsp/esp_bsp_devkit.h"
#include "bsp_err_check.h"
#include "button_gpio.h"
#include "button_adc.h"
#include "led_indicator_strips.h"
#include "led_indicator_gpio.h"
#include "led_indicator_ledc.h"
#include "led_indicator_rgb.h"

static const char *TAG = "BSP-devkit";


/**
 * @brief I2C handle for BSP usage
 *
 * In IDF v5.4 you can call i2c_master_get_bus_handle(BSP_I2C_NUM, i2c_master_bus_handle_t *ret_handle)
 * from #include "esp_private/i2c_platform.h" to get this handle
 *
 * For IDF 5.2 and 5.3 you must call bsp_i2c_get_handle()
 */
static i2c_master_bus_handle_t i2c_handle = NULL;
static bool i2c_initialized = false;
sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler
extern blink_step_t const *bsp_led_blink_defaults_lists[];

typedef enum {
    BSP_BUTTON_TYPE_GPIO,
    BSP_BUTTON_TYPE_ADC
} bsp_button_type_t;

typedef struct {
    bsp_button_type_t type;
    union {
        button_gpio_config_t gpio;
        button_adc_config_t  adc;
    } cfg;
} bsp_button_config_t;

static const bsp_button_config_t bsp_button_config[] = {
#if CONFIG_BSP_BUTTONS_NUM > 0
#if CONFIG_BSP_BUTTON_1_TYPE_GPIO
    {
        .type = BSP_BUTTON_TYPE_GPIO,
        .cfg.gpio = {
            .gpio_num = BSP_BUTTON_1_IO,
            .active_level = CONFIG_BSP_BUTTON_1_LEVEL,
        }
    },
#elif CONFIG_BSP_BUTTON_1_TYPE_ADC
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_channel = CONFIG_BSP_BUTTON_1_ADC_CHANNEL,
            .button_index = BSP_BUTTON_1,
            .min = (CONFIG_BSP_BUTTON_1_ADC_VALUE - 100),
            .max = (CONFIG_BSP_BUTTON_1_ADC_VALUE + 100)
        }
    },
#endif // CONFIG_BSP_BUTTON_1_TYPE_x
#endif // CONFIG_BSP_BUTTONS_NUM >= 0

#if CONFIG_BSP_BUTTONS_NUM > 1
#if CONFIG_BSP_BUTTON_2_TYPE_GPIO
    {
        .type = BSP_BUTTON_TYPE_GPIO,
        .cfg.gpio = {
            .gpio_num = BSP_BUTTON_2_IO,
            .active_level = CONFIG_BSP_BUTTON_2_LEVEL,
        }
    },
#elif CONFIG_BSP_BUTTON_2_TYPE_ADC
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_channel = CONFIG_BSP_BUTTON_2_ADC_CHANNEL,
            .button_index = BSP_BUTTON_2,
            .min = (CONFIG_BSP_BUTTON_2_ADC_VALUE - 100),
            .max = (CONFIG_BSP_BUTTON_2_ADC_VALUE + 100)
        }
    },
#endif // CONFIG_BSP_BUTTON_2_TYPE_x
#endif // CONFIG_BSP_BUTTONS_NUM >= 1

#if CONFIG_BSP_BUTTONS_NUM > 2
#if CONFIG_BSP_BUTTON_3_TYPE_GPIO
    {
        .type = BSP_BUTTON_TYPE_GPIO,
        .cfg.gpio = {
            .gpio_num = BSP_BUTTON_3_IO,
            .active_level = CONFIG_BSP_BUTTON_3_LEVEL,
        }
    },
#elif CONFIG_BSP_BUTTON_3_TYPE_ADC
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_channel = CONFIG_BSP_BUTTON_3_ADC_CHANNEL,
            .button_index = BSP_BUTTON_3,
            .min = (CONFIG_BSP_BUTTON_3_ADC_VALUE - 100),
            .max = (CONFIG_BSP_BUTTON_3_ADC_VALUE + 100)
        }
    },
#endif // CONFIG_BSP_BUTTON_3_TYPE_x
#endif // CONFIG_BSP_BUTTONS_NUM >= 2

#if CONFIG_BSP_BUTTONS_NUM > 3
#if CONFIG_BSP_BUTTON_4_TYPE_GPIO
    {
        .type = BSP_BUTTON_TYPE_GPIO,
        .cfg.gpio = {
            .gpio_num = BSP_BUTTON_4_IO,
            .active_level = CONFIG_BSP_BUTTON_4_LEVEL,
        }
    },
#elif CONFIG_BSP_BUTTON_4_TYPE_ADC
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_channel = CONFIG_BSP_BUTTON_4_ADC_CHANNEL,
            .button_index = BSP_BUTTON_4,
            .min = (CONFIG_BSP_BUTTON_4_ADC_VALUE - 100),
            .max = (CONFIG_BSP_BUTTON_4_ADC_VALUE + 100)
        }
    },
#endif // CONFIG_BSP_BUTTON_4_TYPE_x
#endif // CONFIG_BSP_BUTTONS_NUM >= 3

#if CONFIG_BSP_BUTTONS_NUM > 4
#if CONFIG_BSP_BUTTON_5_TYPE_GPIO
    {
        .type = BSP_BUTTON_TYPE_GPIO,
        .cfg.gpio = {
            .gpio_num = BSP_BUTTON_5_IO,
            .active_level = CONFIG_BSP_BUTTON_5_LEVEL,
        }
    },
#elif CONFIG_BSP_BUTTON_5_TYPE_ADC
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_channel = CONFIG_BSP_BUTTON_5_ADC_CHANNEL,
            .button_index = BSP_BUTTON_5,
            .min = (CONFIG_BSP_BUTTON_5_ADC_VALUE - 100),
            .max = (CONFIG_BSP_BUTTON_5_ADC_VALUE + 100)
        }
    }
#endif // CONFIG_BSP_BUTTON_5_TYPE_x
#endif // CONFIG_BSP_BUTTONS_NUM >= 4
};

#if CONFIG_BSP_LED_TYPE_GPIO
static led_indicator_gpio_config_t bsp_leds_gpio_config[] = {
    {
#if CONFIG_BSP_LED_1_LEVEL && CONFIG_BSP_LED_1_GPIO
        .is_active_level_high = CONFIG_BSP_LED_1_LEVEL,
        .gpio_num = CONFIG_BSP_LED_1_GPIO,
#endif
    },
    {
#if CONFIG_BSP_LED_2_LEVEL && CONFIG_BSP_LED_2_GPIO
        .is_active_level_high = CONFIG_BSP_LED_2_LEVEL,
        .gpio_num = CONFIG_BSP_LED_2_GPIO,
#endif
    },
    {
#if CONFIG_BSP_LED_3_LEVEL && CONFIG_BSP_LED_3_GPIO
        .is_active_level_high = CONFIG_BSP_LED_3_LEVEL,
        .gpio_num = CONFIG_BSP_LED_3_GPIO,
#endif
    },
    {
#if CONFIG_BSP_LED_4_LEVEL && CONFIG_BSP_LED_4_GPIO
        .is_active_level_high = CONFIG_BSP_LED_4_LEVEL,
        .gpio_num = CONFIG_BSP_LED_4_GPIO,
#endif
    },
    {
#if CONFIG_BSP_LED_5_LEVEL && CONFIG_BSP_LED_5_GPIO
        .is_active_level_high = CONFIG_BSP_LED_5_LEVEL,
        .gpio_num = CONFIG_BSP_LED_5_GPIO,
#endif
    }
};
#endif // CONFIG_BSP_LED_TYPE_GPIO

#if CONFIG_BSP_LED_TYPE_RGB && CONFIG_BSP_LEDS_NUM > 0
static const led_strip_config_t bsp_leds_rgb_strip_config = {
    .strip_gpio_num = CONFIG_BSP_LED_RGB_GPIO,   // The GPIO that connected to the LED strip's data line
    .max_leds = BSP_LED_NUM,                  // The number of LEDs in the strip,
    .led_model = LED_MODEL_WS2812,            // LED strip model
    .flags.invert_out = false,                // whether to invert the output signal
};

#if CONFIG_BSP_LED_RGB_BACKEND_RMT
static const led_strip_rmt_config_t bsp_leds_rgb_rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    .rmt_channel = 0,
#else
    .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
    .resolution_hz = 10 * 1000 * 1000,     // RMT counter clock frequency = 10MHz
    .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
#endif
};
#elif CONFIG_BSP_LED_RGB_BACKEND_SPI
static led_strip_spi_config_t bsp_leds_rgb_spi_config = {
    .spi_bus = SPI2_HOST,
    .flags.with_dma = true,
};
#else
#error "unsupported LED strip backend"
#endif

static led_indicator_strips_config_t bsp_leds_rgb_config = {
    .led_strip_cfg = bsp_leds_rgb_strip_config,
#if CONFIG_BSP_LED_RGB_BACKEND_RMT
    .led_strip_driver = LED_STRIP_RMT,
    .led_strip_rmt_cfg = bsp_leds_rgb_rmt_config,
#elif CONFIG_BSP_LED_RGB_BACKEND_SPI
    .led_strip_driver = LED_STRIP_SPI,
    .led_strip_spi_cfg = bsp_leds_rgb_spi_config,
#endif
};

#elif CONFIG_BSP_LED_TYPE_RGB_CLASSIC && CONFIG_BSP_LEDS_NUM > 0 // CONFIG_BSP_LED_TYPE_RGB_CLASSIC

static led_indicator_rgb_config_t bsp_leds_rgb_config = {
    .is_active_level_high = CONFIG_BSP_LED_RGB_CLASSIC_LEVEL,
    .timer_num = LEDC_TIMER_0,
    .red_gpio_num = CONFIG_BSP_LED_RGB_RED_GPIO,
    .green_gpio_num = CONFIG_BSP_LED_RGB_GREEN_GPIO,
    .blue_gpio_num = CONFIG_BSP_LED_RGB_BLUE_GPIO,
    .red_channel = LEDC_CHANNEL_0,
    .green_channel = LEDC_CHANNEL_1,
    .blue_channel = LEDC_CHANNEL_2,
};

#endif // CONFIG_BSP_LED_TYPE_RGB

#if CONFIG_BSP_LEDS_NUM > 0
static const led_indicator_config_t bsp_leds_config = {
    .blink_lists = bsp_led_blink_defaults_lists,
    .blink_list_num = BSP_LED_MAX,
};
#endif // CONFIG_BSP_LEDS_NUM > 0

esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_master_bus_config_t i2c_config = {
        .i2c_port = BSP_I2C_NUM,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_config, &i2c_handle));

    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_handle));
    i2c_initialized = false;
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_get_handle(void)
{
    bsp_i2c_init();
    return i2c_handle;
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = CONFIG_BSP_SPIFFS_MOUNT_POINT,
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
#if SOC_SDMMC_HOST_SUPPORTED
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = CONFIG_BSP_SD_MAX_FILES,
        .allocation_unit_size = 16 * 1024
    };

    const sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    const sdmmc_slot_config_t slot_config = {
#if SOC_SDMMC_USE_GPIO_MATRIX
        .clk = BSP_SD_CLK,
        .cmd = BSP_SD_CMD,
        .d0 = BSP_SD_D0,
        .d1 = BSP_SD_D1,
        .d2 = BSP_SD_D2,
        .d3 = BSP_SD_D3,
        .d4 = GPIO_NUM_NC,
        .d5 = GPIO_NUM_NC,
        .d6 = GPIO_NUM_NC,
        .d7 = GPIO_NUM_NC,
#endif
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP,
        .width = 1,
        .flags = 0,
    };

    return esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
#else
    return ESP_OK;
#endif // SOC_SDMMC_HOST_SUPPORTED
}

esp_err_t bsp_sdcard_unmount(void)
{
#if SOC_SDMMC_HOST_SUPPORTED
    return esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
#else
    return ESP_OK;
#endif // SOC_SDMMC_HOST_SUPPORTED
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
        if (bsp_button_config[i].type == BSP_BUTTON_TYPE_GPIO) {
            ret |= iot_button_new_gpio_device(&btn_config, &bsp_button_config[i].cfg.gpio, &btn_array[i]);
        } else if (bsp_button_config[i].type == BSP_BUTTON_TYPE_ADC) {
            ret |= iot_button_new_adc_device(&btn_config, &bsp_button_config[i].cfg.adc, &btn_array[i]);
        } else {
            ESP_LOGW(TAG, "Unsupported button type!");
        }

        if (btn_cnt) {
            (*btn_cnt)++;
        }
    }
    return ret;
}

esp_err_t bsp_led_indicator_create(led_indicator_handle_t led_array[], int *led_cnt, int led_array_size)
{
    esp_err_t ret = ESP_OK;
    if ((led_array_size < BSP_LED_NUM) ||
            (led_array == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (led_cnt) {
        *led_cnt = 0;
    }
    for (int i = 0; i < BSP_LED_NUM; i++) {
#if CONFIG_BSP_LED_TYPE_GPIO
        ret = led_indicator_new_gpio_device(&bsp_leds_config, &bsp_leds_gpio_config[i], &led_array[i]);
#elif CONFIG_BSP_LED_TYPE_RGB
        ret = led_indicator_new_strips_device(&bsp_leds_config, &bsp_leds_rgb_config, &led_array[i]);
#elif CONFIG_BSP_LED_TYPE_RGB_CLASSIC
        ret = led_indicator_new_rgb_device(&bsp_leds_config, &bsp_leds_rgb_config, &led_array[i]);
#endif
        BSP_ERROR_CHECK_RETURN_ERR(ret);
        if (led_cnt) {
            (*led_cnt)++;
        }
    }
    return ret;
}

esp_err_t bsp_led_set(led_indicator_handle_t handle, const bool on)
{
    if (on) {
        led_indicator_start(handle, BSP_LED_ON);
    } else {
        led_indicator_start(handle, BSP_LED_OFF);
    }

    return ESP_OK;
}

esp_err_t bsp_led_set_temperature(led_indicator_handle_t handle, const uint16_t temperature)
{
    return led_indicator_set_color_temperature(handle, temperature);
}
