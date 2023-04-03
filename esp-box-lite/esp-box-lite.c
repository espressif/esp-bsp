/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "bsp/esp-box-lite.h"
#include "bsp/display.h"
#include "esp_lvgl_port.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"

static const char *TAG = "ESP-BOX-LITE";

static lv_indev_t *disp_indev = NULL;   /* Input device (buttons) */
static const audio_codec_data_if_t *i2s_data_if = NULL;  /* Codec data interface */
static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;

const button_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_0, // ADC1 channel 0 is GPIO1
        .adc_button_config.button_index = BSP_BUTTON_PREV,
        .adc_button_config.min = 2310, // middle is 2410mV
        .adc_button_config.max = 2510
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_0, // ADC1 channel 0 is GPIO1
        .adc_button_config.button_index = BSP_BUTTON_ENTER,
        .adc_button_config.min = 1880, // middle is 1980mV
        .adc_button_config.max = 2080
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_0, // ADC1 channel 0 is GPIO1
        .adc_button_config.button_index = BSP_BUTTON_NEXT,
        .adc_button_config.min = 720, // middle is 820mV
        .adc_button_config.max = 920
    },
};

esp_err_t bsp_i2c_init(void)
{
    static bool i2c_initialized = false;

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


esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config, i2s_chan_handle_t *tx_channel, i2s_chan_handle_t *rx_channel)
{
    if (i2s_tx_chan && i2s_rx_chan && i2s_data_if) {
        if (tx_channel) {
            *tx_channel = i2s_tx_chan;
        }
        if (rx_channel) {
            *rx_channel = i2s_rx_chan;
        }

        /* Audio was initialized before */
        return ESP_OK;
    }

    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    BSP_ERROR_CHECK_RETURN_ERR(i2s_new_channel(&chan_cfg, tx_channel, rx_channel));

    /* Setup I2S channels */
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(22050);
    const i2s_std_config_t *p_i2s_cfg = &std_cfg_default;
    if (i2s_config != NULL) {
        p_i2s_cfg = i2s_config;
    }

    if (tx_channel != NULL) {
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_init_std_mode(*tx_channel, p_i2s_cfg));
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_enable(*tx_channel));
    }
    if (rx_channel != NULL) {
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_init_std_mode(*rx_channel, p_i2s_cfg));
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_enable(*rx_channel));
    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = CONFIG_BSP_I2S_NUM,
        .rx_handle = i2s_rx_chan,
        .tx_handle = i2s_tx_chan,
    };
    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    BSP_NULL_CHECK(i2s_data_if, NULL);

    /* Setup power amplifier pin */
    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(BSP_POWER_AMP_IO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
    };
    BSP_ERROR_CHECK_RETURN_ERR(gpio_config(&io_conf));

    return ESP_OK;
}

esp_err_t bsp_audio_poweramp_enable(bool enable)
{
    BSP_ERROR_CHECK_RETURN_ERR(gpio_set_level(BSP_POWER_AMP_IO, enable ? 1 : 0));

    return ESP_OK;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    if (i2s_tx_chan == NULL || i2s_rx_chan == NULL || i2s_data_if == NULL) {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_audio_init(NULL, &i2s_tx_chan, &i2s_rx_chan));
        BSP_ERROR_CHECK_RETURN_ERR(bsp_audio_poweramp_enable(true));
    }
    assert(i2s_data_if);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8156_CODEC_DEFAULT_ADDR,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };

    es8156_codec_cfg_t es8156_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .hw_gain = gain,
    };
    const audio_codec_if_t *es8156_dev = es8156_codec_new(&es8156_cfg);
    BSP_NULL_CHECK(es8156_dev, NULL);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8156_dev,
        .data_if = i2s_data_if,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    if (i2s_tx_chan == NULL || i2s_rx_chan == NULL || i2s_data_if == NULL) {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_audio_init(NULL, &i2s_tx_chan, &i2s_rx_chan));
        BSP_ERROR_CHECK_RETURN_ERR(bsp_audio_poweramp_enable(true));
    }
    assert(i2s_data_if);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES7243E_CODEC_DEFAULT_ADDR,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    es7243e_codec_cfg_t es7243_cfg = {
        .ctrl_if = i2c_ctrl_if,
    };
    const audio_codec_if_t *es7243_dev = es7243e_codec_new(&es7243_cfg);
    BSP_NULL_CHECK(es7243_dev, NULL);

    esp_codec_dev_cfg_t codec_es7243_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = es7243_dev,
        .data_if = i2s_data_if,
    };
    return esp_codec_dev_new(&codec_es7243_dev_cfg);
}

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8
#define LCD_LEDC_CH            CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

static esp_err_t bsp_display_brightness_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0
    };
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    BSP_ERROR_CHECK_RETURN_ERR(ledc_timer_config(&LCD_backlight_timer));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_channel_config(&LCD_backlight_channel));

    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    }

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    uint32_t duty_cycle = (1023 * (100 - brightness_percent)) / 100; // LEDC resolution set to 10bits, thus: 100% = 1023
    BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));

    return ESP_OK;
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    assert(config != NULL && config->max_transfer_sz > 0);

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_PCLK,
        .mosi_io_num = BSP_LCD_DATA0,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = config->max_transfer_sz,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_CS,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, ret_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .color_space = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);
    esp_lcd_panel_mirror(*ret_panel, false, true);
    esp_lcd_panel_swap_xy(*ret_panel, true);
    esp_lcd_panel_invert_color(*ret_panel, true);
    return ret;

err:
    if (*ret_panel) {
        esp_lcd_panel_del(*ret_panel);
    }
    if (*ret_io) {
        esp_lcd_panel_io_del(*ret_io);
    }
    spi_bus_free(BSP_LCD_SPI_NUM);
    return ret;
}

static lv_disp_t *bsp_display_lcd_init(void)
{
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = BSP_LCD_DRAW_BUFF_SIZE * sizeof(uint16_t),
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));

    esp_lcd_panel_disp_on_off(panel_handle, true);

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = true,
            .mirror_x = false,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = true,
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}

static lv_indev_t *bsp_display_indev_init(lv_disp_t *disp)
{
    const lvgl_port_nav_btns_cfg_t btns = {
        .disp = disp,
        .button_prev = &bsp_button_config[BSP_BUTTON_PREV],
        .button_next = &bsp_button_config[BSP_BUTTON_NEXT],
        .button_enter = &bsp_button_config[BSP_BUTTON_ENTER]
    };

    return lvgl_port_add_navigation_buttons(&btns);
}

lv_disp_t *bsp_display_start(void)
{
    lv_disp_t *disp;
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&lvgl_cfg));

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}
