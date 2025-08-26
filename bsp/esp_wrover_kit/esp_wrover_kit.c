/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"

#include "iot_button.h"
#include "button_gpio.h"
#include "bsp/esp_wrover_kit.h"
#include "bsp/display.h"
#include "bsp_err_check.h"

static const char *TAG = "Wrover";

static sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler
static bool spi_sd_initialized = false;

static const button_gpio_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .gpio_num = BSP_BUTTON_BOOT_IO,
        .active_level = 0,
    }
};

esp_err_t bsp_leds_init(void)
{
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BIT64(BSP_LED_RED) | BIT64(BSP_LED_GREEN) | BIT64(BSP_LED_BLUE),
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
    config->width = 4;
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

    ret |= esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
    bsp_sdcard = NULL;

    if (spi_sd_initialized) {
        ret |= spi_bus_free(BSP_SDSPI_HOST);
        spi_sd_initialized = false;
    }

    return ret;
}

esp_err_t bsp_button_init(const bsp_button_t btn)
{
    const gpio_config_t btn_io_config = {
        .pin_bit_mask = BIT64(btn),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    BSP_ERROR_CHECK_RETURN_ERR(gpio_config(&btn_io_config));
    return ESP_OK;
}

bool bsp_button_get(const bsp_button_t btn)
{
    return !(bool)gpio_get_level(btn);
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
        ret |= iot_button_new_gpio_device(&btn_config, &bsp_button_config[i], &btn_array[i]);
        if (btn_cnt) {
            (*btn_cnt)++;
        }
    }
    return ret;
}

#define LCD_CMD_BITS         (8)
#define LCD_PARAM_BITS       (8)
#define LCD_LEDC_CH          (CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH)

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
        .hpoint = 0,
        .flags.output_invert = true
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
    // LEDC resolution set to 10bits, thus: 100% = 1023
    uint32_t duty_cycle = (1023 * brightness_percent) / 100;
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
        .sclk_io_num = BSP_LCD_SPI_CLK,
        .mosi_io_num = BSP_LCD_SPI_MOSI,
        .miso_io_num = BSP_LCD_SPI_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = config->max_transfer_sz,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_SPI_CS,
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
        .rgb_ele_order = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);
#ifdef CONFIG_BSP_LCD_ILI9341
    esp_lcd_panel_mirror(*ret_panel, true, false);
#endif
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
    esp_lcd_panel_handle_t panel_handle = NULL;
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = BSP_LCD_DRAW_BUFF_SIZE * sizeof(uint16_t),
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    esp_lcd_panel_disp_off(panel_handle, false);
#else
    esp_lcd_panel_disp_on_off(panel_handle, true);
#endif

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
#ifdef CONFIG_BSP_LCD_ILI9341
            .mirror_x = true,
#else
            .mirror_x = false,
#endif
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

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
        }
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    lv_display_t *disp = NULL;
    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));
    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);
    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return NULL;
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
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
