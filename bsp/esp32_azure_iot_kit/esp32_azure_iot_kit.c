/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "driver/ledc.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "bsp/esp32_azure_iot_kit.h"
#include "bsp/display.h"
#include "esp_lvgl_port.h"
#include "bsp_err_check.h"

static const char *TAG = "Azure-IoT";

static lv_display_t *disp;
sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler
static bool i2c_initialized = false;

static const button_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config.gpio_num = BSP_BUTTON_MAIN_IO,
        .gpio_button_config.active_level = 0,
    }
};

typedef enum {
    I2C_CLK_100K_LOW_PERIOD = 399,  I2C_CLK_100K_HIGH_PERIOD = 387,
    I2C_CLK_100K_DATA_SETUP = 200,  I2C_CLK_100K_DATA_HOLD = 200,
    I2C_CLK_100K_START_SETUP = 400, I2C_CLK_100K_START_HOLD = 400,
    I2C_CLK_100K_STOP_SETUP = 400,  I2C_CLK_100K_STOP_HOLD = 400,

    I2C_CLK_400K_LOW_PERIOD = 99,   I2C_CLK_400K_HIGH_PERIOD = 87,
    I2C_CLK_400K_DATA_SETUP = 50,   I2C_CLK_400K_DATA_HOLD = 50,
    I2C_CLK_400K_START_SETUP = 100, I2C_CLK_400K_START_HOLD = 100,
    I2C_CLK_400K_STOP_SETUP = 100,  I2C_CLK_400K_STOP_HOLD = 100,

    I2C_CLK_600K_LOW_PERIOD = 65,   I2C_CLK_600K_HIGH_PERIOD = 53,
    I2C_CLK_600K_DATA_SETUP = 33,   I2C_CLK_600K_DATA_HOLD = 33,
    I2C_CLK_600K_START_SETUP = 66,  I2C_CLK_600K_START_HOLD = 66,
    I2C_CLK_600K_STOP_SETUP = 66,   I2C_CLK_600K_STOP_HOLD = 66
} bsp_i2c_clk_params_set_t;

esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = BSP_I2C_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
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

esp_err_t bsp_i2c_set_clk_speed(bsp_i2c_clk_speed_t i2c_clk)
{
    switch (i2c_clk) {
    case I2C_CLK_100KHZ:
        BSP_ERROR_CHECK_RETURN_ERR(i2c_set_period(BSP_I2C_NUM,       I2C_CLK_100K_HIGH_PERIOD, I2C_CLK_100K_LOW_PERIOD));
        BSP_ERROR_CHECK_RETURN_ERR(i2c_set_data_timing(BSP_I2C_NUM,  I2C_CLK_100K_DATA_SETUP,  I2C_CLK_100K_DATA_HOLD));
        BSP_ERROR_CHECK_RETURN_ERR(i2c_set_start_timing(BSP_I2C_NUM, I2C_CLK_100K_START_SETUP, I2C_CLK_100K_START_HOLD));
        BSP_ERROR_CHECK_RETURN_ERR(i2c_set_stop_timing(BSP_I2C_NUM,  I2C_CLK_100K_STOP_SETUP,  I2C_CLK_100K_STOP_HOLD));
        break;

    case I2C_CLK_400KHZ:
        BSP_ERROR_CHECK_RETURN_ERR(i2c_set_period(BSP_I2C_NUM,       I2C_CLK_400K_HIGH_PERIOD, I2C_CLK_400K_LOW_PERIOD));
        BSP_ERROR_CHECK_RETURN_ERR(i2c_set_data_timing(BSP_I2C_NUM,  I2C_CLK_400K_DATA_SETUP,  I2C_CLK_400K_DATA_HOLD));
        BSP_ERROR_CHECK_RETURN_ERR(i2c_set_start_timing(BSP_I2C_NUM, I2C_CLK_400K_START_SETUP, I2C_CLK_400K_START_HOLD));
        BSP_ERROR_CHECK_RETURN_ERR(i2c_set_stop_timing(BSP_I2C_NUM,  I2C_CLK_400K_STOP_SETUP,  I2C_CLK_400K_STOP_HOLD));
        break;

    case I2C_CLK_600KHZ:
        BSP_ERROR_CHECK_RETURN_ERR(i2c_set_period(BSP_I2C_NUM,       I2C_CLK_600K_HIGH_PERIOD, I2C_CLK_600K_LOW_PERIOD));
        BSP_ERROR_CHECK_RETURN_ERR(i2c_set_data_timing(BSP_I2C_NUM,  I2C_CLK_600K_DATA_SETUP,  I2C_CLK_600K_DATA_HOLD));
        BSP_ERROR_CHECK_RETURN_ERR(i2c_set_start_timing(BSP_I2C_NUM, I2C_CLK_600K_START_SETUP, I2C_CLK_600K_START_HOLD));
        BSP_ERROR_CHECK_RETURN_ERR(i2c_set_stop_timing(BSP_I2C_NUM,  I2C_CLK_600K_STOP_SETUP,  I2C_CLK_600K_STOP_HOLD));
        break;

    default:
        break;
    }
    return ESP_OK;
}

esp_err_t bsp_leds_init(void)
{
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BIT64(BSP_LED_AZURE) | BIT64(BSP_LED_WIFI),
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

esp_err_t bsp_sdcard_mount(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;

    return esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    ESP_LOGW(TAG, "This board doesn't support to change brightness of LCD");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;

    ESP_LOGD(TAG, "Initialize I2C bus");
    ESP_RETURN_ON_ERROR(bsp_i2c_init(), TAG, "I2C init failed");

    ESP_LOGD(TAG, "Install panel IO");
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = BSP_LCD_I2C_ADDR,
        .control_phase_bytes = 1,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .dc_bit_offset = 6,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)BSP_I2C_NUM, &io_config, ret_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
        .color_space = BSP_LCD_COLOR_SPACE,
        .reset_gpio_num = -1,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ssd1306(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);
    esp_lcd_panel_mirror(*ret_panel, true, true);
    return ret;

err:
    if (*ret_panel) {
        esp_lcd_panel_del(*ret_panel);
    }
    if (*ret_io) {
        esp_lcd_panel_io_del(*ret_io);
    }
    return ret;
}

static lv_display_t *bsp_display_lcd_init(void)
{
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(NULL, &panel_handle, &io_handle));

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
        .monochrome = true,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = true,
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG()
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(), NULL);

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

esp_err_t bsp_buzzer_init(void)
{
    const ledc_timer_config_t buzzer_timer_conf = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = (ledc_timer_t)CONFIG_BSP_BUZZER_LEDC_TIMER_NUM,
        .freq_hz = CONFIG_BSP_BUZZER_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };

    const esp_err_t ret = ledc_timer_config(&buzzer_timer_conf);
    if (ESP_OK != ret) {
        return ret;
    }

    const ledc_channel_config_t buzzer_channel_conf = {
        .gpio_num = (int)BSP_BUZZER_IO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel =  (ledc_channel_t)CONFIG_BSP_BUZZER_LEDC_CHANNEL_NUM,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = (ledc_timer_t)CONFIG_BSP_BUZZER_LEDC_TIMER_NUM,
        .duty = 0,
        .hpoint = 0
    };
    return ledc_channel_config(&buzzer_channel_conf);
}

esp_err_t bsp_buzzer_set(const bool on)
{
    BSP_ERROR_CHECK_RETURN_ERR(ledc_set_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)CONFIG_BSP_BUZZER_LEDC_CHANNEL_NUM, on ? 127 : 0));
    BSP_ERROR_CHECK_RETURN_ERR(ledc_update_duty(LEDC_HIGH_SPEED_MODE, (ledc_channel_t)CONFIG_BSP_BUZZER_LEDC_CHANNEL_NUM));
    return ESP_OK;
}

esp_err_t bsp_button_init(void)
{
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BIT64(BSP_BUTTON_MAIN_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&led_io_config);
}

bool bsp_button_get(void)
{
    return !(bool)gpio_get_level(BSP_BUTTON_MAIN_IO);
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
