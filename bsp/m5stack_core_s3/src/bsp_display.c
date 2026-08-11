/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "bsp_err_check.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ili9341.h"
#include "esp_lcd_touch_ft5x06.h"
#include "freertos/task.h"
#include "bsp/display.h"
#include "bsp/touch.h"

#include "bsp/m5stack_core_s3.h"

static const char *TAG = "M5Stack Core S3";
#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *disp;
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static bsp_lcd_handles_t disp_handles;
static lv_indev_t *disp_indev_touch = NULL;
static esp_lcd_touch_handle_t tp;   // LCD touch handle
static bsp_display_panel_id_t s_display_panel_id = BSP_DISPLAY_PANEL_ID_ILI9342C;

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8
#define LCD_LEDC_CH            CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

#define BSP_FT5X06_ADDR                    0x38
#define BSP_FT5X06_CIPHER_REG              0xA3
#define BSP_FT5X06_FIRMID_REG              0xA6
#define BSP_FT5X06_VENDID_REG              0xA8
#define BSP_FT5X06_ILI9342C_FIRMID         0x10
#define BSP_FT5X06_ILI9342E_FIRMID         0x12
#define BSP_FT5X06_M5STACK_VENDOR          0x11
#define BSP_FT5X06_VERSION_I2C_FREQ_HZ     100000
#define BSP_FT5X06_STARTUP_DELAY_MS        300
#define BSP_FT5X06_VERSION_RETRIES         5
#define BSP_FT5X06_RETRY_DELAY_MS          20
#define BSP_FT5X06_I2C_TIMEOUT_MS          100

static const ili9341_lcd_init_cmd_t ili9342e_init_cmds[] = {
    {0xDD, (uint8_t []){0x01}, 1, 0},
    {0x3A, (uint8_t []){0x55}, 1, 0},
    {0x21, NULL, 0, 0},
    {0x36, (uint8_t []){0x08}, 1, 0},
    {0xD5, (uint8_t []){0x00}, 1, 0},
    {0xB1, (uint8_t []){0x22}, 1, 0},
    {0xC8, (uint8_t []){0x38}, 1, 0},
    {0xCB, (uint8_t []){0x1C}, 1, 0},
    {0xC9, (uint8_t []){0x1A}, 1, 0},
    {0xCA, (uint8_t []){0x1A}, 1, 0},
    {0xB7, (uint8_t []){0x5A, 0x41, 0x11, 0x19}, 4, 0},
    {0xE4, (uint8_t []){0x04, 0x08, 0x11, 0x06, 0x12, 0x07, 0x3A, 0x76, 0x47, 0x07, 0x0F, 0x0A, 0x11, 0x19, 0x05}, 15, 0},
    {0xE5, (uint8_t []){0x02, 0x03, 0x07, 0x06, 0x12, 0x07, 0x36, 0x5F, 0x48, 0x06, 0x10, 0x0C, 0x16, 0x14, 0x09}, 15, 0},
    {0x11, NULL, 0, 120},
    {0x29, NULL, 0, 120},
};

static esp_err_t bsp_lcd_read_touch_reg(i2c_master_dev_handle_t touch_handle, uint8_t reg_addr, uint8_t *value)
{
    return i2c_master_transmit_receive(touch_handle, &reg_addr, 1, value, 1, BSP_FT5X06_I2C_TIMEOUT_MS);
}

static esp_err_t bsp_lcd_detect_panel_id(bsp_display_panel_id_t *panel_id)
{
    ESP_RETURN_ON_FALSE(panel_id, ESP_ERR_INVALID_ARG, TAG, "Invalid panel ID output");

    *panel_id = BSP_DISPLAY_PANEL_ID_ILI9342C;

    const i2c_device_config_t touch_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BSP_FT5X06_ADDR,
        .scl_speed_hz = BSP_FT5X06_VERSION_I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t touch_handle = NULL;
    esp_err_t add_device_ret = i2c_master_bus_add_device(bsp_i2c_get_handle(), &touch_config, &touch_handle);
    if (add_device_ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to create touch version I2C device: %s; panel:ILI9342C",
                 esp_err_to_name(add_device_ret));
        return ESP_OK;
    }

    uint8_t touch_cipher = 0;
    uint8_t touch_firmid = 0;
    uint8_t touch_vendid = 0;
    bool touch_info_valid = false;
    esp_err_t work_mode_ret = ESP_FAIL;
    esp_err_t cipher_ret = ESP_FAIL;
    esp_err_t firmid_ret = ESP_FAIL;
    esp_err_t vendid_ret = ESP_FAIL;

    vTaskDelay(pdMS_TO_TICKS(BSP_FT5X06_STARTUP_DELAY_MS));
    for (int retry = 0; retry < BSP_FT5X06_VERSION_RETRIES && !touch_info_valid; retry++) {
        const uint8_t work_mode[] = {0x00, 0x00};
        touch_cipher = 0;
        touch_firmid = 0;
        touch_vendid = 0;
        work_mode_ret = i2c_master_transmit(touch_handle, work_mode, sizeof(work_mode), BSP_FT5X06_I2C_TIMEOUT_MS);
        cipher_ret = bsp_lcd_read_touch_reg(touch_handle, BSP_FT5X06_CIPHER_REG, &touch_cipher);
        firmid_ret = bsp_lcd_read_touch_reg(touch_handle, BSP_FT5X06_FIRMID_REG, &touch_firmid);
        vendid_ret = bsp_lcd_read_touch_reg(touch_handle, BSP_FT5X06_VENDID_REG, &touch_vendid);

        touch_info_valid = work_mode_ret == ESP_OK
                           && firmid_ret == ESP_OK
                           && vendid_ret == ESP_OK
                           && touch_vendid == BSP_FT5X06_M5STACK_VENDOR
                           && (touch_firmid == BSP_FT5X06_ILI9342C_FIRMID
                               || touch_firmid == BSP_FT5X06_ILI9342E_FIRMID);
        if (!touch_info_valid) {
            vTaskDelay(pdMS_TO_TICKS(BSP_FT5X06_RETRY_DELAY_MS));
        }
    }

    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(touch_handle), TAG,
                        "Failed to remove touch version I2C device");

    if (!touch_info_valid) {
        ESP_LOGW(TAG,
                 "CoreS3 touch version read failed (WORK:%s / CIPHER:%s,0x%02X / FIRMID:%s,0x%02X / VENDID:%s,0x%02X), panel:ILI9342C",
                 esp_err_to_name(work_mode_ret), esp_err_to_name(cipher_ret), touch_cipher,
                 esp_err_to_name(firmid_ret), touch_firmid, esp_err_to_name(vendid_ret), touch_vendid);
        return ESP_OK;
    }

    bool use_ili9342e = touch_firmid == BSP_FT5X06_ILI9342E_FIRMID;
    *panel_id = use_ili9342e ? BSP_DISPLAY_PANEL_ID_ILI9342E : BSP_DISPLAY_PANEL_ID_ILI9342C;
    ESP_LOGI(TAG, "CoreS3 touch CIPHER:0x%02X / FIRMID:0x%02X / VENDID:0x%02X, panel:%s",
             touch_cipher, touch_firmid, touch_vendid, use_ili9342e ? "ILI9342E" : "ILI9342C");
    return ESP_OK;
}

static esp_err_t bsp_lcd_apply_ili9342e_init(esp_lcd_panel_io_handle_t io)
{
    ESP_RETURN_ON_FALSE(io, ESP_ERR_INVALID_ARG, TAG, "Invalid LCD panel IO");

    for (size_t i = 0; i < sizeof(ili9342e_init_cmds) / sizeof(ili9342e_init_cmds[0]); i++) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, ili9342e_init_cmds[i].cmd,
                            ili9342e_init_cmds[i].data, ili9342e_init_cmds[i].data_bytes),
                            TAG, "Send ILI9342E command 0x%02X failed", ili9342e_init_cmds[i].cmd);
        if (ili9342e_init_cmds[i].delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(ili9342e_init_cmds[i].delay_ms));
        }
    }

    return ESP_OK;
}

bsp_display_panel_id_t bsp_display_get_panel_id(void)
{
    return s_display_panel_id;
}

esp_err_t bsp_display_brightness_init(void)
{
    /* Everything will be initialized during first set */
    return ESP_OK;
}

esp_err_t bsp_display_brightness_deinit(void)
{
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
    ESP_RETURN_ON_FALSE(config && ret_panel && ret_io, ESP_ERR_INVALID_ARG, TAG, "Invalid argument");

    *ret_panel = NULL;
    *ret_io = NULL;
    bsp_lcd_handles_t handles = {0};
    esp_err_t ret = bsp_display_new_with_handles(config, &handles);
    if (ret != ESP_OK) {
        return ret;
    }

    *ret_panel = handles.panel;
    *ret_io = handles.io;

    return ret;
}

esp_err_t bsp_display_new_with_handles(const bsp_display_config_t *config, bsp_lcd_handles_t *ret_handles)
{
    esp_err_t ret = ESP_OK;
    bsp_display_panel_id_t panel_id = BSP_DISPLAY_PANEL_ID_ILI9342C;
    ESP_RETURN_ON_FALSE(config && ret_handles, ESP_ERR_INVALID_ARG, TAG, "Invalid argument");
    ESP_RETURN_ON_FALSE(config->max_transfer_sz > 0, ESP_ERR_INVALID_ARG, TAG, "Invalid transfer size");
    *ret_handles = (bsp_lcd_handles_t) {
        0
    };
    s_display_panel_id = BSP_DISPLAY_PANEL_ID_ILI9342C;

    /* Enable Feature */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_LCD, true));

    /* DISPLAY - SPI */
    const bsp_spi_cfg_t spi_cfg = {
        .max_transfer_sz = config->max_transfer_sz,
    };
    ESP_RETURN_ON_ERROR(bsp_spi_init(&spi_cfg), TAG, "SPI init failed");

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
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, &ret_handles->io),
                      err, TAG, "New panel IO failed");
    disp_handles.io = ret_handles->io;
    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .flags.reset_active_high = 0,
        .rgb_ele_order = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_ili9341(ret_handles->io, &panel_config, &ret_handles->panel),
                      err, TAG, "New panel failed");
    disp_handles.panel = ret_handles->panel;

    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(ret_handles->panel), err, TAG, "Reset panel failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(ret_handles->panel), err, TAG, "Initialize panel failed");

    esp_err_t touch_enable_ret = bsp_feature_enable(BSP_FEATURE_TOUCH, true);
    if (touch_enable_ret == ESP_OK) {
        ESP_GOTO_ON_ERROR(bsp_lcd_detect_panel_id(&panel_id), err, TAG, "Detect display panel failed");
        if (panel_id == BSP_DISPLAY_PANEL_ID_ILI9342E) {
            ESP_GOTO_ON_ERROR(bsp_lcd_apply_ili9342e_init(ret_handles->io), err, TAG,
                              "Initialize ILI9342E panel failed");
        }
    } else {
        ESP_LOGW(TAG, "Touch version detection unavailable: %s; panel:ILI9342C",
                 esp_err_to_name(touch_enable_ret));
    }

    ESP_GOTO_ON_ERROR(esp_lcd_panel_invert_color(ret_handles->panel, true), err, TAG, "Invert panel colors failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_swap_xy(ret_handles->panel, false), err, TAG, "Set panel swap failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_mirror(ret_handles->panel, false, false), err, TAG, "Set panel mirror failed");

    s_display_panel_id = panel_id;
    ESP_LOGI(TAG, "Display initialized with resolution %dx%d, panel:%s", BSP_LCD_H_RES, BSP_LCD_V_RES,
             panel_id == BSP_DISPLAY_PANEL_ID_ILI9342E ? "ILI9342E" : "ILI9342C");
    return ret;

err:
    bsp_display_delete();
    return ret;
}

void bsp_display_delete(void)
{
    if (disp_handles.panel) {
        esp_lcd_panel_del(disp_handles.panel);
        disp_handles.panel = NULL;
    }
    if (disp_handles.io) {
        esp_lcd_panel_io_del(disp_handles.io);
        disp_handles.io = NULL;
    }
    bsp_spi_deinit();

    bsp_display_brightness_deinit();
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = (BSP_LCD_H_RES * CONFIG_BSP_LCD_DRAW_BUF_HEIGHT) * sizeof(uint16_t),
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &disp_handles.panel, &io_handle));

    esp_lcd_panel_disp_on_off(disp_handles.panel, true);

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = disp_handles.panel,
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
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
            .sw_rotate = false,                /* Avoid tearing is not supported for SW rotation */
#else
            .sw_rotate = cfg->flags.sw_rotate,
#endif
        }
    };
    return lvgl_port_add_disp(&disp_cfg);
}
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

esp_err_t bsp_touch_new(const bsp_touch_config_t *config, esp_lcd_touch_handle_t *ret_touch)
{
    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    /* Enable Feature */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_feature_enable(BSP_FEATURE_TOUCH, true));

    /* Initialize touch */
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC, // Usually shared with LCD reset
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5x06_CONFIG();
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(bsp_i2c_get_handle(), &tp_io_config, &tp_io_handle), TAG, "");

    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, ret_touch),
                        TAG, "New touch driver initialization failed");

    return ESP_OK;
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
static lv_indev_t *bsp_display_indev_touch_init(lv_display_t *disp)
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

static esp_err_t bsp_touch_enter_sleep(void)
{
    assert(tp);
    return esp_lcd_touch_enter_sleep(tp);
}

static esp_err_t bsp_touch_exit_sleep(void)
{
    assert(tp);
    return esp_lcd_touch_exit_sleep(tp);
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
            .sw_rotate = false,
        }
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));
    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);
    BSP_NULL_CHECK(disp_indev_touch = bsp_display_indev_touch_init(disp), NULL);

    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return disp_indev_touch;
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
    assert(disp_handles.panel);
    return esp_lcd_panel_disp_on_off(disp_handles.panel, false);
}

static esp_err_t bsp_lcd_exit_sleep(void)
{
    assert(disp_handles.panel);
    return esp_lcd_panel_disp_on_off(disp_handles.panel, true);
}

esp_err_t bsp_display_enter_sleep(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_lcd_enter_sleep());
    BSP_ERROR_CHECK_RETURN_ERR(bsp_touch_enter_sleep());
    return ESP_OK;
}

esp_err_t bsp_display_exit_sleep(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_lcd_exit_sleep());
    BSP_ERROR_CHECK_RETURN_ERR(bsp_touch_exit_sleep());
    return ESP_OK;
}
#endif // (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
