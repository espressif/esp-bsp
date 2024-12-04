/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <esp_err.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_spiffs.h>
#include <esp_vfs_fat.h>

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/spi_master.h>
#include <driver/sdmmc_host.h>
#include <driver/sdspi_host.h>

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_ops.h>

#include "bsp/m5stack_core.h"
#include "bsp/display.h"
#include "esp_lcd_ili9341.h"
#include "bsp_err_check.h"
#include "esp_codec_dev_defaults.h"

static const char *TAG = "M5Stack";

#define BSP_IP5306_ADDR  0x75

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *disp;
static lv_indev_t *disp_indev = NULL;
#endif                               // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static bool i2c_initialized = false;
static bool spi_initialized = false;

esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_config_t i2c_conf = {.mode             = I2C_MODE_MASTER,
                                   .sda_io_num       = BSP_I2C_SDA,
                                   .sda_pullup_en    = GPIO_PULLUP_DISABLE,
                                   .scl_io_num       = BSP_I2C_SCL,
                                   .scl_pullup_en    = GPIO_PULLUP_DISABLE,
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

uint8_t read8bit(uint8_t sub_addr)
{
    // Read register data
    uint8_t reg_data[1] = {0};
    esp_err_t err = i2c_master_write_read_device(BSP_I2C_NUM, BSP_IP5306_ADDR, &sub_addr, 1, reg_data, sizeof(reg_data),
                    1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write & read register address: %s", esp_err_to_name(err));
    }
    ESP_LOGD(TAG, "IP5306 register %x: 0x%x", sub_addr, reg_data[0]);

    return reg_data[0];
}

esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable)
{
    esp_err_t err = ESP_OK;

    /* Initialize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    uint8_t val = 0;
    switch (feature) {
    case BSP_FEATURE_LCD:  
    case BSP_FEATURE_TOUCH:
    case BSP_FEATURE_SD:
    case BSP_FEATURE_SPEAKER:
        // IP5306 can only control the overall output, not each function separately
        // Read the current BOOST output status
        if (i2c_master_write_read_device(BSP_I2C_NUM, BSP_IP5306_ADDR, 
                                       (uint8_t[]){0x00}, 1, &val, 1, 
                                       1000 / portTICK_PERIOD_MS) != ESP_OK) {
            return ESP_FAIL;
        }
        
        if (enable) {
            val |= 0x02;  // Set BOOST_OUT_BIT
        } else {
            val &= ~0x02; // Clear BOOST_OUT_BIT
        }
        
        // Write back to the register
        const uint8_t write_buf[] = {0x00, val};
        err = i2c_master_write_to_device(BSP_I2C_NUM, BSP_IP5306_ADDR, 
                                       write_buf, sizeof(write_buf), 
                                       1000 / portTICK_PERIOD_MS);
        break;

    case BSP_FEATURE_BATTERY:
        // IP5306 has built-in battery management function, no need to enable separately
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

esp_err_t bsp_sdcard_mount(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_SD, true));

    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files            = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = BSP_LCD_SPI_NUM;
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = BSP_SD_CS;
    slot_config.host_id = host.slot;

    ESP_RETURN_ON_ERROR(bsp_spi_init((BSP_LCD_H_RES * BSP_LCD_V_RES) * sizeof(uint16_t)), TAG, "");

    return esp_vfs_fat_sdspi_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
}

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config)
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

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    // Simple GPIO initialization
    BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL));
    return NULL;  // No longer using codec
}

esp_err_t bsp_speaker_enable(bool enable)
{
    return gpio_set_level(BSP_SPEAKER_IO, enable ? 1 : 0);
}

// Bit number used to represent command and parameter
#define LCD_CMD_BITS   8
#define LCD_PARAM_BITS 8
#define LCD_LEDC_CH    CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

esp_err_t bsp_display_brightness_init(void)
{
    // Configure backlight GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BSP_LCD_BACKLIGHT),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&io_conf);
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
    
    return gpio_set_level(BSP_LCD_BACKLIGHT, brightness_percent > 0);
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
        .color_space    = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };

    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ili9341(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);
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

static void button_init(void)
{
    // Configure buttons as input with pull-up
    gpio_config_t btn_config = {
        .pin_bit_mask = BIT64(BSP_BUTTON_A) | BIT64(BSP_BUTTON_B) | BIT64(BSP_BUTTON_C),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&btn_config));
}

static void button_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static uint32_t last_key = 0;   // Store the last key value
    
    // Read button states (low level means pressed)
    bool btn_a_pressed = !gpio_get_level(BSP_BUTTON_A);  // Left key
    bool btn_b_pressed = !gpio_get_level(BSP_BUTTON_B);  // Enter key
    bool btn_c_pressed = !gpio_get_level(BSP_BUTTON_C);  // Right key

    if(btn_a_pressed) {
        data->key = LV_KEY_LEFT;
        last_key = LV_KEY_LEFT;
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else if(btn_b_pressed) {
        data->key = LV_KEY_ENTER;
        last_key = LV_KEY_ENTER;
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else if(btn_c_pressed) {
        data->key = LV_KEY_RIGHT;
        last_key = LV_KEY_RIGHT;
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else {
        data->state = LV_INDEV_STATE_RELEASED;
        data->key = last_key;    // Keep last key value when released
    }
}

// Function to check if buttons are enabled
static bool is_buttons_enabled(void)
{
    // Read the state of all three buttons
    bool btn_a = gpio_get_level(BSP_BUTTON_A);
    bool btn_b = gpio_get_level(BSP_BUTTON_B);
    bool btn_c = gpio_get_level(BSP_BUTTON_C);
    
    // If all buttons are high (not pressed), buttons are properly initialized
    return (btn_a && btn_b && btn_c);
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{
    // Initialize buttons first
    button_init();
    
    // Check if buttons are available
    if (!is_buttons_enabled()) {
        ESP_LOGW(TAG, "Buttons not enabled, skipping keypad input device");
        return NULL;
    }
    
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_KEYPAD;
    indev_drv.read_cb = button_read;
    indev_drv.disp = disp;
    
    return lv_indev_drv_register(&indev_drv);
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

int8_t bsp_battery_get_level(void)
{
    uint8_t data;
    if (i2c_master_write_read_device(BSP_I2C_NUM, BSP_IP5306_ADDR, 
                                   (uint8_t[]){0x78}, 1, &data, 1, 
                                   1000 / portTICK_PERIOD_MS) == ESP_OK) {
        switch (data >> 4) {
            case 0x00: return 100;
            case 0x08: return 75;
            case 0x0C: return 50;
            case 0x0E: return 25;
            default:   return 0;
        }
    }
    return -1;
}

bool bsp_battery_is_charging(void)
{
    uint8_t val = 0;
    if (i2c_master_write_read_device(BSP_I2C_NUM, BSP_IP5306_ADDR, 
                                   (uint8_t[]){0x71}, 1, &val, 1, 
                                   1000 / portTICK_PERIOD_MS) == ESP_OK) {
        return (val & 0x0C);
    }
    return false;
}

#endif  // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)