/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
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
#include "esp_vfs_fat.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"

#include <driver/rtc_io.h>
#include "esp_sleep.h"
#include "led_indicator_strips.h"
#include "led_indicator_gpio.h"
#include "led_indicator_ledc.h"
#include "led_indicator_rgb.h"


#include "iot_button.h"
#include "button_gpio.h"
#include "button_adc.h"
#include "iot_knob.h"
#include "bsp/esp32_p4_eye.h"
#include "bsp/display.h"


#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"
#include "esp_cam_sensor_xclk.h"
#include "esp_video_device.h"
#include "esp_video_init.h"

static const char *TAG = "ESP32-P4-EYE";

/* Can be used for i2s_pdm_rx_gpio_config_t and/or i2s_pdm_rx_config_t initialization */
#define BSP_I2S_PDM_RX_GPIO_CFG     \
    {                            \
        .clk = BSP_I2S_CLK,      \
        .din = BSP_I2S_DSIN,     \
        .invert_flags = {        \
            .clk_inv = false,    \
        },                       \
    }

/* This configuration is used by default in bsp_audio_init() */
#define BSP_I2S_PDM_RX_DUPLEX_CFG(_sample_rate)                                         \
    {                                                                                    \
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(_sample_rate),                          \
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,             \
                                                   I2S_SLOT_MODE_MONO),                  \
        .gpio_cfg = BSP_I2S_PDM_RX_GPIO_CFG,                                                \
    }

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *disp;
static esp_lcd_panel_handle_t panel_handle = NULL;
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL; //SD LDO handle
static sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler
static bool spi_sd_initialized = false;
static adc_oneshot_unit_handle_t bsp_adc_handle = NULL;
static adc_cali_handle_t bsp_adc_cali_handle = NULL;
extern blink_step_t const *bsp_led_blink_defaults_lists[];
static i2s_chan_handle_t i2s_tx_chan = NULL;
static i2s_chan_handle_t i2s_rx_chan = NULL;
static const audio_codec_data_if_t *i2s_data_if = NULL;  /* Codec data interface */
static lv_indev_t *disp_indev_enc = NULL;

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

static const bsp_button_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .type = BSP_BUTTON_TYPE_GPIO,
        .cfg.gpio = {
            .gpio_num = BSP_BUTTON_1_IO,
            .active_level = 0,
        }

    },
};

static const button_gpio_config_t bsp_encoder_btn_config = {
    .gpio_num = BSP_ENCODER_PRESS,
    .active_level = 0,
};

static const knob_config_t bsp_encoder_a_b_config = {
    .default_direction = 0,
    .gpio_encoder_a = BSP_ENCODER_A,
    .gpio_encoder_b = BSP_ENCODER_B,
};

typedef enum {
    BSP_NAV_BUTTON_PREV,
    BSP_NAV_BUTTON_NEXT,
    BSP_NAV_BUTTON_ENTER,
    BSP_NAV_BUTTON_NUM
} bsp_nav_buttons_t;

static lv_indev_t *disp_indev_btn = NULL;

static const button_gpio_config_t bsp_nav_btns_config[BSP_NAV_BUTTON_NUM] = {
    {
        .gpio_num = GPIO_NUM_4,
        .active_level = 0,
    },
    {
        .gpio_num = GPIO_NUM_5,
        .active_level = 0,
    },
    {
        .gpio_num = GPIO_NUM_3,
        .active_level = 0,
    }
};


static led_indicator_gpio_config_t bsp_leds_gpio_config[] = {
    {
        .is_active_level_high = 1,
        .gpio_num = BSP_LED_1_IO,
    },
};


static const led_indicator_config_t bsp_leds_config = {
    .blink_lists = bsp_led_blink_defaults_lists,
    .blink_list_num = BSP_LED_MAX,
};



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
        .flags.enable_internal_pullup = true,
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

sdmmc_card_t *bsp_sdcard_get_handle(void)
{
    return bsp_sdcard;
}

void bsp_sdcard_get_sdmmc_host(const int slot, sdmmc_host_t *config)
{
    assert(config);
    sdmmc_host_t host_config = SDMMC_HOST_DEFAULT();
    host_config.slot = slot;
    host_config.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

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
    config->cmd = BSP_SD_CMD;
    config->clk = BSP_SD_CLK;
    config->d0 = BSP_SD_D0;
    config->d1 = BSP_SD_D1;
    config->d2 = BSP_SD_D2;
    config->d3 = BSP_SD_D3;
    config->width = 4;
    config->flags = 0;
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

    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4,
    };
    esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        return ret;
    }
    cfg->host->pwr_ctrl_handle = pwr_ctrl_handle;

#if !CONFIG_FATFS_LONG_FILENAMES
    ESP_LOGW(TAG, "Warning: Long filenames on SD card are disabled in menuconfig!");
#endif
    /* Enable Feature */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_SD, true));

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

    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4,
    };
    esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        return ret;
    }
    cfg->host->pwr_ctrl_handle = pwr_ctrl_handle;

#if !CONFIG_FATFS_LONG_FILENAMES
    ESP_LOGW(TAG, "Warning: Long filenames on SD card are disabled in menuconfig!");
#endif
    /* Enable Feature */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_SD, true));

    return esp_vfs_fat_sdspi_mount(BSP_SD_MOUNT_POINT, cfg->host, cfg->slot.sdspi, cfg->mount, &bsp_sdcard);
}

esp_err_t bsp_sdcard_mount(void)
{
    bsp_sdcard_cfg_t cfg = {0};
    return bsp_sdcard_sdmmc_mount(&cfg);
}

esp_err_t bsp_sdcard_unmount(void)
{
    esp_err_t ret = ESP_OK;

    if (pwr_ctrl_handle) {
        ret |= sd_pwr_ctrl_del_on_chip_ldo(pwr_ctrl_handle);
        pwr_ctrl_handle = NULL;
    }

    ret |= esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
    bsp_sdcard = NULL;
    if (spi_sd_initialized) {
        ret |= spi_bus_free(BSP_SDSPI_HOST);
        spi_sd_initialized = false;
    }

    return ret;
}

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
{
    esp_err_t ret = ESP_FAIL;
    if (i2s_tx_chan && i2s_rx_chan) {
        /* Audio was initialized before */
        return ESP_OK;
    }

    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    BSP_ERROR_CHECK_RETURN_ERR(i2s_new_channel(&chan_cfg, NULL, &i2s_rx_chan));

    /* Setup RX I2S channels */
    const i2s_pdm_rx_config_t pdm_rx_cfg_default = BSP_I2S_PDM_RX_DUPLEX_CFG(16000);

    if (i2s_rx_chan != NULL) {
        ESP_GOTO_ON_ERROR(i2s_channel_init_pdm_rx_mode(i2s_rx_chan, &pdm_rx_cfg_default), err, TAG, "I2S channel initialization failed");
        ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_rx_chan), err, TAG, "I2S channel initialization failed");
    }

    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = CONFIG_BSP_I2S_NUM,
        .rx_handle = i2s_rx_chan,
        .tx_handle = NULL,
    };
    i2s_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    BSP_NULL_CHECK_GOTO(i2s_data_if, err);

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

const audio_codec_data_if_t *bsp_audio_get_codec_itf(void)
{
    return i2s_data_if;
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    const audio_codec_data_if_t *i2s_data_if = bsp_audio_get_codec_itf();
    if (i2s_data_if == NULL) {
        /* Initilize I2C */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
        /* Configure I2S peripheral and Power Amplifier */
        BSP_ERROR_CHECK_RETURN_ERR(bsp_audio_init(NULL));
        i2s_data_if = bsp_audio_get_codec_itf();
    }
    assert(i2s_data_if);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = NULL,
        .data_if = i2s_data_if,
    };
    return esp_codec_dev_new(&codec_dev_cfg);
}

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8
#define LCD_LEDC_CH            CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

esp_err_t bsp_display_brightness_init(void)
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
    /* Enable Feature */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_LCD, true));
    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");
    /* DISPLAY - SPI */
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
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .flags.reset_active_high = 0,
        .rgb_ele_order = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(*ret_io, (const esp_lcd_panel_dev_config_t *)&panel_config, ret_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);
    esp_lcd_panel_invert_color(*ret_panel, true);
    esp_lcd_panel_swap_xy(*ret_panel, false);
    esp_lcd_panel_mirror(*ret_panel, false, false);
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

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = (BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT) * sizeof(uint16_t),
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));

    esp_lcd_panel_disp_on_off(panel_handle, true);

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
        }
    };
    return lvgl_port_add_disp(&disp_cfg);
}
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_indev_t *bsp_display_indev_encoder_init(lv_display_t *disp)
{

    const button_config_t btn_cfg = {0};
    button_handle_t encoder_btn = NULL;
    BSP_ERROR_CHECK_RETURN_NULL(iot_button_new_gpio_device(&btn_cfg, &bsp_encoder_btn_config, &encoder_btn));

    const lvgl_port_encoder_cfg_t encoder = {
        .disp = disp,
        .encoder_a_b = &bsp_encoder_a_b_config,
        .encoder_enter = encoder_btn
    };

    return lvgl_port_add_encoder(&encoder);
}
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_indev_t *bsp_display_indev_nav_buttons_init(lv_display_t *disp)
{
    const button_config_t btn_cfg = {0};
    button_handle_t btns[BSP_NAV_BUTTON_NUM];
    for (int i = 0; i < BSP_NAV_BUTTON_NUM; i++) {
        BSP_ERROR_CHECK_RETURN_NULL(iot_button_new_gpio_device(&btn_cfg, &bsp_nav_btns_config[i], &btns[i]));
    }

    const lvgl_port_nav_btns_cfg_t nav_btns = {
        .disp = disp,
        .button_prev = btns[BSP_NAV_BUTTON_PREV],
        .button_next = btns[BSP_NAV_BUTTON_NEXT],
        .button_enter = btns[BSP_NAV_BUTTON_ENTER]
    };

    return lvgl_port_add_navigation_buttons(&nav_btns);
}
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT,
#if CONFIG_BSP_LCD_DRAW_BUF_DOUBLE
        .double_buffer = 1,
#else
        .double_buffer = 0,
#endif
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
        }
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);
    BSP_NULL_CHECK(disp_indev_enc = bsp_display_indev_encoder_init(disp), NULL);
    BSP_NULL_CHECK(disp_indev_btn = bsp_display_indev_nav_buttons_init(disp), NULL);

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev_enc;
}

void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation)
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

static esp_err_t bsp_lcd_enter_sleep(void)
{
    assert(panel_handle);
    return esp_lcd_panel_disp_on_off(panel_handle, false);
}

static esp_err_t bsp_lcd_exit_sleep(void)
{
    assert(panel_handle);
    return esp_lcd_panel_disp_on_off(panel_handle, true);
}

esp_err_t bsp_display_enter_sleep(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_lcd_enter_sleep());
    BSP_ERROR_CHECK_RETURN_ERR(bsp_display_backlight_off());
    return ESP_OK;
}

esp_err_t bsp_display_exit_sleep(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_lcd_exit_sleep());
    BSP_ERROR_CHECK_RETURN_ERR(bsp_display_backlight_on());
    return ESP_OK;
}
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

esp_err_t bsp_camera_start(const bsp_camera_cfg_t *cfg)
{
    esp_cam_sensor_xclk_handle_t xclk_handle = NULL;

    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    /* Enable Feature */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_CAMERA, true));

    /* Camera Clock Init */
    esp_cam_sensor_xclk_config_t cam_xclk_config = {
        .esp_clock_router_cfg = {
            .xclk_pin = BSP_CAMERA_GPIO_XCLK,
            .xclk_freq_hz = BSP_CAMERA_XCLK_CLOCK_MHZ * 1000000,
        }
    };
    BSP_ERROR_CHECK_RETURN_ERR(esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_ESP_CLOCK_ROUTER, &xclk_handle));
    BSP_ERROR_CHECK_RETURN_ERR(esp_cam_sensor_xclk_start(xclk_handle, &cam_xclk_config));


    /* Camera reset */
    const gpio_config_t rst_io_config = {
        .pin_bit_mask = BIT64(BSP_CAMERA_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    BSP_ERROR_CHECK_RETURN_ERR(gpio_config(&rst_io_config));
    BSP_ERROR_CHECK_RETURN_ERR(gpio_set_level(BSP_CAMERA_RST, 1));

    const esp_video_init_csi_config_t base_csi_config = {
        .sccb_config = {
            .init_sccb = false,
            .i2c_handle = i2c_handle,
            .freq = 400000,
        },
        .reset_pin = BSP_CAMERA_RST,
        .pwdn_pin  = -1,
    };

    esp_video_init_config_t cam_config = {
        .csi      = &base_csi_config,
    };

    return esp_video_init(&cam_config);
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

esp_err_t bsp_voltage_init(void)
{
    /* Initialize ADC and get ADC handle */
    BSP_ERROR_CHECK_RETURN_NULL(bsp_adc_initialize());

    /* Init ADC1 channels */
    const adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    BSP_ERROR_CHECK_RETURN_ERR(adc_oneshot_config_channel(bsp_adc_handle, ADC_CHANNEL_2, &config));

    /* ESP32-S3 supports Curve Fitting calibration scheme */
    const adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = BSP_ADC_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    BSP_ERROR_CHECK_RETURN_ERR(adc_cali_create_scheme_curve_fitting(&cali_config, &bsp_adc_cali_handle));
    return ESP_OK;
}

int bsp_voltage_battery_get(void)
{
    int voltage, adc_raw;

    assert(bsp_adc_handle);
    BSP_ERROR_CHECK(adc_oneshot_read(bsp_adc_handle, ADC_CHANNEL_2, &adc_raw), -1);
    BSP_ERROR_CHECK(adc_cali_raw_to_voltage(bsp_adc_cali_handle, adc_raw, &voltage), -1);
    return voltage * BSP_BATTERY_VOLTAGE_DIV;
}

/* Feature enable */
esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable)
{
    esp_err_t ret = ESP_OK;


    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_VDDSDIO, ESP_PD_OPTION_ON);

    switch (feature) {
    case BSP_FEATURE_SD: {
        const gpio_config_t io_config = {
            .pin_bit_mask = BIT64(BSP_SD_EN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        ESP_ERROR_CHECK(gpio_config(&io_config));
        gpio_set_level(BSP_SD_EN, !enable);
        break;
    }
    case BSP_FEATURE_LCD: {
        ret |= rtc_gpio_init(BSP_LCD_EN);
        ret |= rtc_gpio_set_direction(BSP_LCD_EN, RTC_GPIO_MODE_OUTPUT_ONLY);
        ret |= rtc_gpio_pulldown_dis(BSP_LCD_EN);
        ret |= rtc_gpio_pullup_dis(BSP_LCD_EN);
        ret |= rtc_gpio_hold_dis(BSP_LCD_EN);
        ret |= rtc_gpio_set_level(BSP_LCD_EN, enable);
        ret |= rtc_gpio_hold_en(BSP_LCD_EN);
        break;
    }
    case BSP_FEATURE_CAMERA: {
        ret |= rtc_gpio_init(BSP_CAMERA_EN);
        ret |= rtc_gpio_set_direction(BSP_CAMERA_EN, RTC_GPIO_MODE_OUTPUT_ONLY);
        ret |= rtc_gpio_pulldown_dis(BSP_CAMERA_EN);
        ret |= rtc_gpio_pullup_dis(BSP_CAMERA_EN);
        ret |= rtc_gpio_hold_dis(BSP_CAMERA_EN);
        ret |= rtc_gpio_set_level(BSP_CAMERA_EN, enable);
        ret |= rtc_gpio_hold_en(BSP_CAMERA_EN);
        break;
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
        ret = led_indicator_new_gpio_device(&bsp_leds_config, &bsp_leds_gpio_config[i], &led_array[i]);
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
