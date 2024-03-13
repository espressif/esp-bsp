/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_private/esp_ldo.h"
#include "esp_lcd_ili9881c.h"
#include "esp_vfs_fat.h"
#include "usb/usb_host.h"

#include "bsp/esp32_p4_function_ev_board.h"
#include "bsp/display.h"
#include "bsp/touch.h"
#include "esp_lcd_touch_gt911.h"
#include "bsp_err_check.h"

static const char *TAG = "ESP32_P4_EV";

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_indev_t *disp_indev = NULL;
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler
static esp_lcd_touch_handle_t tp;   // LCD touch handle
static bool i2c_initialized = false;
static TaskHandle_t usb_host_task;  // USB Host Library task

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

esp_err_t bsp_sdcard_mount(void)
{
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 64 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    const sdmmc_slot_config_t slot_config = {
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
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP,
        .width = 4,
        .flags = 0,
    };

    return esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(BSP_SD_MOUNT_POINT, bsp_sdcard);
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

// Bit number used to represent command and parameter
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

static esp_err_t bsp_enable_dsi_phy_power(void)
{
    // Turn on the power for MIPI DSI PHY, so it can go from "No Power" state to "Shutdown" state
    esp_ldo_unit_handle_t phy_pwr_unit = NULL;
#if BSP_MIPI_DSI_PHY_PWR_LDO_UNIT > 0
    esp_ldo_unit_init_cfg_t ldo_cfg = {
        .unit_id = BSP_MIPI_DSI_PHY_PWR_LDO_UNIT,
        .cfg = {
            .voltage_mv = BSP_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
        },
    };
    ESP_RETURN_ON_ERROR(esp_ldo_init_unit(&ldo_cfg, &phy_pwr_unit), TAG, "LDO init failed");
    ESP_RETURN_ON_ERROR(esp_ldo_enable_unit(phy_pwr_unit), TAG, "LDO enable failed");

    ESP_LOGI(TAG, "MIPI DSI PHY Powered on");
#endif

    return ESP_OK;
}

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    bsp_lcd_handles_t handles;
    ret = bsp_display_new_with_handles(config, &handles);

    *ret_panel = handles.panel;
    *ret_io = handles.io;

    return ret;
}

esp_err_t bsp_display_new_with_handles(const bsp_display_config_t *config, bsp_lcd_handles_t *ret_handles)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");
    ESP_RETURN_ON_ERROR(bsp_enable_dsi_phy_power(), TAG, "DSI PHY power failed");

    /* create MIPI DSI bus first, it will initialize the DSI PHY as well */
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = BSP_LCD_MIPI_DSI_LANE_NUM,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus), TAG, "New DSI bus init failed");

    ESP_LOGI(TAG, "Install MIPI DSI LCD control panel");
    // we use DBI interface to send LCD commands and parameters
    esp_lcd_panel_io_handle_t io;
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,   // according to the LCD ILI9881C spec
        .lcd_param_bits = 8, // according to the LCD ILI9881C spec
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io), err, TAG, "New panel IO failed");

    // create ILI9881C control panel
    esp_lcd_panel_handle_t ili9881c_ctrl_panel;
    esp_lcd_panel_dev_config_t lcd_dev_config = {
        .bits_per_pixel = 16,
        .rgb_ele_order = BSP_LCD_COLOR_SPACE,
        .reset_gpio_num = -1,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ili9881c(io, &lcd_dev_config, &ili9881c_ctrl_panel), err, TAG, "New LCD panel ILI9881C failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(ili9881c_ctrl_panel), err, TAG, "LCD panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(ili9881c_ctrl_panel), err, TAG, "LCD panel init failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_disp_on_off(ili9881c_ctrl_panel, true), err, TAG, "LCD panel ON failed");
    esp_lcd_panel_mirror(ili9881c_ctrl_panel, true, true);

    ESP_LOGI(TAG, "Install MIPI DSI LCD data panel");
    esp_lcd_panel_handle_t ili9881c_panel;
    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = BSP_LCD_PIXEL_CLOCK_MHZ,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .video_timing = {
            .h_size = BSP_LCD_V_RES,
            .v_size = BSP_LCD_H_RES,
            .hsync_back_porch = BSP_LCD_MIPI_DSI_LCD_HBP,
            .hsync_pulse_width = BSP_LCD_MIPI_DSI_LCD_HSYNC,
            .hsync_front_porch = BSP_LCD_MIPI_DSI_LCD_HFP,
            .vsync_back_porch = BSP_LCD_MIPI_DSI_LCD_VBP,
            .vsync_pulse_width = BSP_LCD_MIPI_DSI_LCD_VSYNC,
            .vsync_front_porch = BSP_LCD_MIPI_DSI_LCD_VFP,
        },
        .flags.use_dma2d = true,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_dpi(mipi_dsi_bus, &dpi_config, &ili9881c_panel), err, TAG, "New panel DPI failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(ili9881c_panel), err, TAG, "New panel DPI init failed");

    /* Return all handles */
    ret_handles->io = io;
    ret_handles->mipi_dsi_bus = mipi_dsi_bus;
    ret_handles->panel = ili9881c_panel;
    ret_handles->control = ili9881c_ctrl_panel;

    ESP_LOGI(TAG, "Display initialized");

    return ret;

err:
    if (ili9881c_panel) {
        esp_lcd_panel_del(ili9881c_panel);
    }
    if (ili9881c_ctrl_panel) {
        esp_lcd_panel_del(ili9881c_ctrl_panel);
    }
    if (io) {
        esp_lcd_panel_io_del(io);
    }
    if (mipi_dsi_bus) {
        esp_lcd_del_dsi_bus(mipi_dsi_bus);
    }
    return ret;
}

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());

    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)BSP_I2C_NUM, &tp_io_config, &tp_io_handle), TAG, "");
    return esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, ret_touch);
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    bsp_lcd_handles_t lcd_panels;
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new_with_handles(NULL, &lcd_panels));

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_panels.io,
        .panel_handle = lcd_panels.panel,
        .control_handle = lcd_panels.control,
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .trans_size = (cfg->flags.sw_rotate ? BSP_LCD_H_RES * 50 : 0),
        .hres = BSP_LCD_V_RES,
        .vres = BSP_LCD_H_RES,
        .monochrome = false,
        .mipi_dsi = true,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
            .sw_rotate = cfg->flags.sw_rotate, /* Only SW rotation is supported for 90° and 270° */
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}

static lv_indev_t *bsp_display_indev_init(lv_display_t *disp)
{
    BSP_ERROR_CHECK_RETURN_NULL(bsp_touch_new(NULL, &tp));
    assert(tp);

    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp,
    };

    return lvgl_port_add_touch(&touch_cfg);
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
            .sw_rotate = false,  /* When SPIRAM used for buffer, only SW rotation is supported; For 90° and 270° is not supported HW rotation in this driver. */
        }
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    lv_display_t *disp;

    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_brightness_init());

    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);

    BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev;
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

static void usb_lib_task(void *arg)
{
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
            // The only way this task can be stopped is by calling bsp_usb_host_stop()
        }
    }
}

esp_err_t bsp_usb_host_start(bsp_usb_host_power_mode_t mode, bool limit_500mA)
{
    //Install USB Host driver. Should only be called once in entire application
    ESP_LOGI(TAG, "Installing USB Host");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    BSP_ERROR_CHECK_RETURN_ERR(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    if (xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, &usb_host_task) != pdTRUE) {
        ESP_LOGE(TAG, "Creating USB host lib task failed");
        abort();
    }

    return ESP_OK;
}

esp_err_t bsp_usb_host_stop(void)
{
    usb_host_uninstall();
    if (usb_host_task) {
        vTaskSuspend(usb_host_task);
        vTaskDelete(usb_host_task);
    }
    return ESP_OK;
}
