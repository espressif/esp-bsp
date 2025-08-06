/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_spiffs.h>
#include <esp_vfs_fat.h>

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/spi_master.h>
#include <driver/sdmmc_host.h>
#include <driver/sdspi_host.h>
#include <driver/ledc.h>

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>

#include "bsp/m5stack_core.h"
#include "bsp/display.h"
#include "esp_lcd_ili9341.h"
#include "bsp_err_check.h"
#include "button_gpio.h"

static const char *TAG = "M5Stack";

#define BSP_IP5306_ADDR  0x75

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *disp;
static lv_indev_t *disp_indev = NULL;
/* Button definitions */
typedef enum {
    BSP_BUTTON_PREV,   // Button left
    BSP_BUTTON_ENTER,  // Button middle
    BSP_BUTTON_NEXT,   // Button right
    BSP_BUTTON_NUM
} bsp_button_t;
#endif                               // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static i2c_master_bus_handle_t i2c_handle = NULL;
static bool i2c_initialized = false;
static i2c_master_dev_handle_t ip5306_h = NULL;
static bool spi_initialized = false;
static sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler

esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_master_bus_config_t i2c_config = {
        .i2c_port = BSP_I2C_NUM,
        .scl_io_num = BSP_I2C_SCL,
        .sda_io_num = BSP_I2C_SDA,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_config, &i2c_handle));

    const i2c_device_config_t ip5306_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BSP_IP5306_ADDR,
        .scl_speed_hz = CONFIG_BSP_I2C_CLK_SPEED_HZ,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_master_bus_add_device(i2c_handle, &ip5306_config, &ip5306_h));

    i2c_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    if (i2c_initialized) {
        // Delete I2C device for IP5306 first
        if (i2c_handle != NULL) {
            i2c_master_bus_rm_device(ip5306_h);
            ip5306_h = NULL;
        }
        // Then delete the I2C master bus
        BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_handle));
        i2c_handle = NULL;
        i2c_initialized = false;
    }
    return ESP_OK;
}

esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable)
{
    esp_err_t err = ESP_OK;

    /* Initialize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    switch (feature) {
    case BSP_FEATURE_LCD:
        // backlight is controlled by GPIO
        if (enable) {
            err = bsp_display_backlight_on();
        } else {
            err = bsp_display_backlight_off();
        }
        break;
    }
    return err;
}

static esp_err_t bsp_spi_init(uint32_t max_transfer_sz)
{
    /* SPI was initialized before */
    if (spi_initialized) {
        return ESP_OK;
    }

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num     = BSP_LCD_PCLK,
        .mosi_io_num     = BSP_LCD_MOSI,
        .miso_io_num     = BSP_LCD_MISO,
        .quadwp_io_num   = GPIO_NUM_NC,
        .quadhd_io_num   = GPIO_NUM_NC,
        .max_transfer_sz = max_transfer_sz,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    spi_initialized = true;

    return ESP_OK;
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path       = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files       = CONFIG_BSP_SPIFFS_MAX_FILES,
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
    memset(config, 0, sizeof(sdmmc_host_t));
    ESP_LOGE(TAG, "SD card MMC mode is not supported by HW (Shared SPI)!");
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
    ESP_LOGE(TAG, "SD card MMC mode is not supported by HW (Shared SPI)!");
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
}

esp_err_t bsp_sdcard_sdmmc_mount(bsp_sdcard_cfg_t *cfg)
{
    ESP_LOGE(TAG, "SD card MMC mode is not supported by HW (Shared SPI)!");
    return ESP_ERR_NOT_SUPPORTED;
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

    ESP_RETURN_ON_ERROR(bsp_spi_init((BSP_LCD_H_RES * BSP_LCD_V_RES) * sizeof(uint16_t)), TAG, "");

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
    return bsp_sdcard_sdspi_mount(&cfg);
}

esp_err_t bsp_sdcard_unmount(void)
{
    esp_err_t ret = ESP_OK;

    ret |= esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
    bsp_sdcard = NULL;

    //TODO: Check if LCD initialized (when LCD deinit will be covered by BSP)
    if (spi_initialized) {
        ret |= spi_bus_free(BSP_SDSPI_HOST);
        spi_initialized = false;
    }

    return ret;
}

esp_err_t bsp_speaker_init(void)
{
    // Configure speaker GPIO as output mode
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BSP_SPEAKER_IO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(BSP_SPEAKER_IO, 0);  // Turn off speaker

    return ESP_OK;
}

#define LCD_CMD_BITS   8
#define LCD_PARAM_BITS 8
#define LCD_LEDC_CH CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

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
    uint32_t duty_cycle = (1023 * brightness_percent) / 100; // LEDC resolution set to 10bits, thus: 100% = 1023
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

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel,
                          esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    assert(config != NULL && config->max_transfer_sz > 0);

    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_LCD, true));

    /* Initialize SPI */
    ESP_RETURN_ON_ERROR(bsp_spi_init(config->max_transfer_sz), TAG, "");

    ESP_LOGI(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num       = BSP_LCD_DC,
        .cs_gpio_num       = BSP_LCD_CS,
        .pclk_hz           = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits      = LCD_CMD_BITS,
        .lcd_param_bits    = LCD_PARAM_BITS,
        .spi_mode          = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, ret_io), err, TAG,
                      "New panel IO failed");

    ESP_LOGI(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,  // Shared with Touch reset
        .rgb_ele_order    = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };

    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ili9341(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);
#if CONFIG_BSP_M5STACK_CORE_LCD_INVERT_COLOR
    esp_lcd_panel_invert_color(*ret_panel, true);
#else
    esp_lcd_panel_invert_color(*ret_panel, false);
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

    esp_lcd_panel_disp_on_off(panel_handle, true);

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle     = io_handle,
        .panel_handle  = panel_handle,
        .buffer_size   = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres          = BSP_LCD_H_RES,
        .vres          = BSP_LCD_V_RES,
        .monochrome    = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation =
        {
            .swap_xy  = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma    = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}

static const button_gpio_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .gpio_num = BSP_BUTTON_LEFT,
        .active_level = 0,
    },
    {
        .gpio_num = BSP_BUTTON_MIDDLE,
        .active_level = 0,
    },
    {
        .gpio_num = BSP_BUTTON_RIGHT,
        .active_level = 0,
    },
};

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{
    const button_config_t btn_cfg = {0};
    button_handle_t prev_btn = NULL;
    button_handle_t next_btn = NULL;
    button_handle_t enter_btn = NULL;
    BSP_ERROR_CHECK_RETURN_NULL(iot_button_new_gpio_device(&btn_cfg, &bsp_button_config[BSP_BUTTON_PREV], &prev_btn));
    BSP_ERROR_CHECK_RETURN_NULL(iot_button_new_gpio_device(&btn_cfg, &bsp_button_config[BSP_BUTTON_NEXT], &next_btn));
    BSP_ERROR_CHECK_RETURN_NULL(iot_button_new_gpio_device(&btn_cfg, &bsp_button_config[BSP_BUTTON_ENTER], &enter_btn));

    const lvgl_port_nav_btns_cfg_t btns = {
        .disp = disp,
        .button_prev = prev_btn,
        .button_next = next_btn,
        .button_enter = enter_btn
    };

    return lvgl_port_add_navigation_buttons(&btns);
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg           = {.lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
                                       .buffer_size   = BSP_LCD_DRAW_BUFF_SIZE,
                                       .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
    .flags         = {
        .buff_dma    = true,
        .buff_spiram = false,
    }
                                      };
    cfg.lvgl_port_cfg.task_affinity = 1; /* For camera */
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    // Initialize keypad input device
    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    // Turn on backlight
    bsp_display_backlight_on();

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
}

void bsp_display_rotate(lv_display_t *disp, lv_display_rotation_t rotation)
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

esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size)
{
    esp_err_t ret = ESP_OK;
    if ((btn_array_size < BSP_BUTTON_NUM) || (btn_array == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (btn_cnt) {
        *btn_cnt = 0;
    }
    const button_config_t btn_cfg = {0};
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        ret |= iot_button_new_gpio_device(&btn_cfg, &bsp_button_config[i], &btn_array[i]);
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
#endif  // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
