/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_vfs_fat.h"

#include "bsp/esp_bsp_generic.h"
#include "bsp_err_check.h"

static const char *TAG = "BSP-Gen";

sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler
static bool i2c_initialized = false;
extern blink_step_t const *bsp_led_blink_defaults_lists[];

typedef struct {
    uint16_t hue;
    uint8_t saturation;
} HS_color_t;

static const HS_color_t temp_table[] = {
    {4, 100},  {8, 100},  {11, 100}, {14, 100}, {16, 100}, {18, 100}, {20, 100}, {22, 100}, {24, 100}, {25, 100},
    {27, 100}, {28, 100}, {30, 100}, {31, 100}, {31, 95},  {30, 89},  {30, 85},  {29, 80},  {29, 76},  {29, 73},
    {29, 69},  {28, 66},  {28, 63},  {28, 60},  {28, 57},  {28, 54},  {28, 52},  {27, 49},  {27, 47},  {27, 45},
    {27, 43},  {27, 41},  {27, 39},  {27, 37},  {27, 35},  {27, 33},  {27, 31},  {27, 30},  {27, 28},  {27, 26},
    {27, 25},  {27, 23},  {27, 22},  {27, 21},  {27, 19},  {27, 18},  {27, 17},  {27, 15},  {28, 14},  {28, 13},
    {28, 12},  {29, 10},  {29, 9},   {30, 8},   {31, 7},   {32, 6},   {34, 5},   {36, 4},   {41, 3},   {49, 2},
    {0, 0},    {294, 2},  {265, 3},  {251, 4},  {242, 5},  {237, 6},  {233, 7},  {231, 8},  {229, 9},  {228, 10},
    {227, 11}, {226, 11}, {226, 12}, {225, 13}, {225, 13}, {224, 14}, {224, 14}, {224, 15}, {224, 15}, {223, 16},
    {223, 16}, {223, 17}, {223, 17}, {223, 17}, {222, 18}, {222, 18}, {222, 19}, {222, 19}, {222, 19}, {222, 19},
    {222, 20}, {222, 20}, {222, 20}, {222, 21}, {222, 21}
};

static const button_config_t bsp_button_config[] = {
#if CONFIG_BSP_BUTTONS_NUM > 0
#if CONFIG_BSP_BUTTON_1_TYPE_GPIO
    {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config.gpio_num = BSP_BUTTON_1_IO,
        .gpio_button_config.active_level = CONFIG_BSP_BUTTON_1_LEVEL,
    },
#elif CONFIG_BSP_BUTTON_1_TYPE_ADC
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = CONFIG_BSP_BUTTON_1_ADC_CHANNEL,
        .adc_button_config.button_index = BSP_BUTTON_1,
        .adc_button_config.min = (CONFIG_BSP_BUTTON_1_ADC_VALUE - 100),
        .adc_button_config.max = (CONFIG_BSP_BUTTON_1_ADC_VALUE + 100)
    }
#endif // CONFIG_BSP_BUTTON_1_TYPE_x
#endif // CONFIG_BSP_BUTTONS_NUM >= 0

#if CONFIG_BSP_BUTTONS_NUM > 1
#if CONFIG_BSP_BUTTON_2_TYPE_GPIO
    {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config.gpio_num = BSP_BUTTON_2_IO,
        .gpio_button_config.active_level = CONFIG_BSP_BUTTON_2_LEVEL,
    },

#elif CONFIG_BSP_BUTTON_2_TYPE_ADC
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = CONFIG_BSP_BUTTON_2_ADC_CHANNEL,
        .adc_button_config.button_index = BSP_BUTTON_2,
        .adc_button_config.min = (CONFIG_BSP_BUTTON_2_ADC_VALUE - 100),
        .adc_button_config.max = (CONFIG_BSP_BUTTON_2_ADC_VALUE + 100)
    }
#endif // CONFIG_BSP_BUTTON_2_TYPE_x
#endif // CONFIG_BSP_BUTTONS_NUM >= 1

#if CONFIG_BSP_BUTTONS_NUM > 2
#if CONFIG_BSP_BUTTON_3_TYPE_GPIO
    {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config.gpio_num = BSP_BUTTON_3_IO,
        .gpio_button_config.active_level = CONFIG_BSP_BUTTON_3_LEVEL,
    },

#elif CONFIG_BSP_BUTTON_3_TYPE_ADC
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = CONFIG_BSP_BUTTON_3_ADC_CHANNEL,
        .adc_button_config.button_index = BSP_BUTTON_3,
        .adc_button_config.min = (CONFIG_BSP_BUTTON_3_ADC_VALUE - 100),
        .adc_button_config.max = (CONFIG_BSP_BUTTON_3_ADC_VALUE + 100)
    }
#endif // CONFIG_BSP_BUTTON_3_TYPE_x
#endif // CONFIG_BSP_BUTTONS_NUM >= 2

#if CONFIG_BSP_BUTTONS_NUM > 3
#if CONFIG_BSP_BUTTON_4_TYPE_GPIO
    {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config.gpio_num = BSP_BUTTON_4_IO,
        .gpio_button_config.active_level = CONFIG_BSP_BUTTON_4_LEVEL,
    },

#elif CONFIG_BSP_BUTTON_4_TYPE_ADC
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = CONFIG_BSP_BUTTON_4_ADC_CHANNEL,
        .adc_button_config.button_index = BSP_BUTTON_4,
        .adc_button_config.min = (CONFIG_BSP_BUTTON_4_ADC_VALUE - 100),
        .adc_button_config.max = (CONFIG_BSP_BUTTON_4_ADC_VALUE + 100)
    }
#endif // CONFIG_BSP_BUTTON_4_TYPE_x
#endif // CONFIG_BSP_BUTTONS_NUM >= 3

#if CONFIG_BSP_BUTTONS_NUM > 4
#if CONFIG_BSP_BUTTON_5_TYPE_GPIO
    {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config.gpio_num = BSP_BUTTON_5_IO,
        .gpio_button_config.active_level = CONFIG_BSP_BUTTON_5_LEVEL,
    },

#elif CONFIG_BSP_BUTTON_5_TYPE_ADC
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = CONFIG_BSP_BUTTON_5_ADC_CHANNEL,
        .adc_button_config.button_index = BSP_BUTTON_5,
        .adc_button_config.min = (CONFIG_BSP_BUTTON_5_ADC_VALUE - 100),
        .adc_button_config.max = (CONFIG_BSP_BUTTON_5_ADC_VALUE + 100)
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
    .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
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
    .is_active_level_high = 1,
    .led_strip_cfg = bsp_leds_rgb_strip_config,
#if CONFIG_BSP_LED_RGB_BACKEND_RMT
    .led_strip_driver = LED_STRIP_RMT,
    .led_strip_rmt_cfg = bsp_leds_rgb_rmt_config,
#elif CONFIG_BSP_LED_RGB_BACKEND_SPI
    .led_strip_driver = LED_STRIP_SPI,
    .led_strip_spi_cfg = bsp_leds_rgb_spi_config,
#endif
};
#endif // CONFIG_BSP_LED_TYPE_RGB

static const led_indicator_config_t bsp_leds_config[BSP_LED_NUM] = {
#if CONFIG_BSP_LED_TYPE_RGB
    {
        .mode = LED_STRIPS_MODE,
        .led_indicator_strips_config = &bsp_leds_rgb_config,
        .blink_lists = bsp_led_blink_defaults_lists,
        .blink_list_num = BSP_LED_MAX,
    },
#elif CONFIG_BSP_LED_TYPE_GPIO

#if CONFIG_BSP_LEDS_NUM > 0
    {
        .mode = LED_GPIO_MODE,
        .led_indicator_gpio_config = &bsp_leds_gpio_config[0],
        .blink_lists = bsp_led_blink_defaults_lists,
        .blink_list_num = BSP_LED_MAX,
    },
#endif  // CONFIG_BSP_LEDS_NUM > 0
#if CONFIG_BSP_LEDS_NUM > 1
    {
        .mode = LED_GPIO_MODE,
        .led_indicator_gpio_config = &bsp_leds_gpio_config[1],
        .blink_lists = bsp_led_blink_defaults_lists,
        .blink_list_num = BSP_LED_MAX,
    },
#endif  // CONFIG_BSP_LEDS_NUM > 1
#if CONFIG_BSP_LEDS_NUM > 2
    {
        .mode = LED_GPIO_MODE,
        .led_indicator_gpio_config = &bsp_leds_gpio_config[2],
        .blink_lists = bsp_led_blink_defaults_lists,
        .blink_list_num = BSP_LED_MAX,
    },
#endif  // CONFIG_BSP_LEDS_NUM > 2
#if CONFIG_BSP_LEDS_NUM > 3
    {
        .mode = LED_GPIO_MODE,
        .led_indicator_gpio_config = &bsp_leds_gpio_config[3],
        .blink_lists = bsp_led_blink_defaults_lists,
        .blink_list_num = BSP_LED_MAX,
    },
#endif  // CONFIG_BSP_LEDS_NUM > 3
#if CONFIG_BSP_LEDS_NUM > 4
    {
        .mode = LED_GPIO_MODE,
        .led_indicator_gpio_config = &bsp_leds_gpio_config[4],
        .blink_lists = bsp_led_blink_defaults_lists,
        .blink_list_num = BSP_LED_MAX,
    },
#endif  // CONFIG_BSP_LEDS_NUM > 4
#endif // CONFIG_BSP_LED_TYPE_RGB/CONFIG_BSP_LED_TYPE_GPIO
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
        led_array[i] = led_indicator_create(&bsp_leds_config[i]);
        if (led_array[i] == NULL) {
            ret = ESP_FAIL;
            break;
        }
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
    uint32_t hsv = led_indicator_get_hsv(handle);
    uint16_t hue;
    uint8_t saturation;

    if (temperature < 600) {
        hue = 0;
        saturation = 100;
    } else if (temperature > 10000) {
        hue = 222;
        saturation = 21 + (temperature - 10000) * 41 / 990000;
    } else {
        hue = temp_table[(temperature - 600) / 100].hue;
        saturation = temp_table[(temperature - 600) / 100].saturation;
    }
    saturation = (saturation * 255) / 100;

    SET_SATURATION(hsv, saturation);
    SET_HUE(hsv, hue);

    return led_indicator_set_hsv(handle, hsv);
}
