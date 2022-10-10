/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_sh1107.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "sh1107";

#define LCD_SH1107_I2C_CMD  0X00
#define LCD_SH1107_I2C_RAM  0X40

#define LCD_SH1107_PARAM_ONOFF          0xAE
#define LCD_SH1107_PARAM_MIRROR_X       0xA0
#define LCD_SH1107_PARAM_MIRROR_Y       0xC0
#define LCD_SH1107_PARAM_INVERT_COLOR   0xA6

static esp_err_t panel_sh1107_del(esp_lcd_panel_t *panel);
static esp_err_t panel_sh1107_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_sh1107_init(esp_lcd_panel_t *panel);
static esp_err_t panel_sh1107_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_sh1107_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_sh1107_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_sh1107_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_sh1107_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_sh1107_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    unsigned int bits_per_pixel;
    uint16_t lcd_width;
    uint16_t lcd_height;
    bool swap_axes;
} sh1107_panel_t;

esp_err_t esp_lcd_new_panel_sh1107(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    sh1107_panel_t *sh1107 = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE(panel_dev_config->bits_per_pixel == 1, ESP_ERR_INVALID_ARG, err, TAG, "bpp must be 1");
    esp_lcd_panel_sh1107_config_t *vendor_cfg = (esp_lcd_panel_sh1107_config_t *) panel_dev_config->vendor_config;
    ESP_GOTO_ON_FALSE(vendor_cfg, ESP_ERR_INVALID_ARG, err, TAG, "vendor config cannot be null");
    sh1107 = calloc(1, sizeof(sh1107_panel_t));
    ESP_GOTO_ON_FALSE(sh1107, ESP_ERR_NO_MEM, err, TAG, "no mem for sh1107 panel");

    if (GPIO_IS_VALID_OUTPUT_GPIO(panel_dev_config->reset_gpio_num)) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    sh1107->io = io;
    sh1107->lcd_width = vendor_cfg->lcd_width;
    sh1107->lcd_height = vendor_cfg->lcd_height;
    sh1107->bits_per_pixel = panel_dev_config->bits_per_pixel;
    sh1107->reset_gpio_num = panel_dev_config->reset_gpio_num;
    sh1107->reset_level = panel_dev_config->flags.reset_active_high;
    sh1107->base.del = panel_sh1107_del;
    sh1107->base.reset = panel_sh1107_reset;
    sh1107->base.init = panel_sh1107_init;
    sh1107->base.draw_bitmap = panel_sh1107_draw_bitmap;
    sh1107->base.invert_color = panel_sh1107_invert_color;
    sh1107->base.set_gap = panel_sh1107_set_gap;
    sh1107->base.mirror = panel_sh1107_mirror;
    sh1107->base.swap_xy = panel_sh1107_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    sh1107->base.disp_off = panel_sh1107_disp_on_off;
#else
    sh1107->base.disp_on_off = panel_sh1107_disp_on_off;
#endif
    *ret_panel = &(sh1107->base);
    ESP_LOGD(TAG, "new sh1107 panel @%p", sh1107);

    return ESP_OK;

err:
    if (sh1107) {
        if (GPIO_IS_VALID_OUTPUT_GPIO(panel_dev_config->reset_gpio_num)) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(sh1107);
    }
    return ret;
}

static esp_err_t panel_sh1107_del(esp_lcd_panel_t *panel)
{
    sh1107_panel_t *sh1107 = __containerof(panel, sh1107_panel_t, base);
    if (GPIO_IS_VALID_OUTPUT_GPIO(sh1107->reset_gpio_num)) {
        gpio_reset_pin(sh1107->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del sh1107 panel @%p", sh1107);
    free(sh1107);
    return ESP_OK;
}

static esp_err_t panel_sh1107_reset(esp_lcd_panel_t *panel)
{
    sh1107_panel_t *sh1107 = __containerof(panel, sh1107_panel_t, base);

    // perform hardware reset
    if (GPIO_IS_VALID_OUTPUT_GPIO(sh1107->reset_gpio_num)) {
        gpio_set_level(sh1107->reset_gpio_num, sh1107->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(sh1107->reset_gpio_num, !sh1107->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}

static const uint8_t vendor_specific_init[] = {
    0xAE,   /* turn off OLED display */

    0xdc,   /* set display start line */
    0x00,

    0x81,   /* contrast control */
    0x2f,   /* 128 */

    0x20,   /* Set Memory addressing mode (0x20/0x21) */

    0xA1,   /* Non-flipped horizontal */
    0xC7,   /* Non-flipped vertical */

    0xa8,   /* multiplex ratio */
    0x7f,   /* duty = 1/64 */

    0xd3,   /* set display offset */
    0x60,

    0xd5,   /* set osc division */
    0x51,

    0xd9,   /* set pre-charge period */
    0x22,

    0xdb,   /* set vcomh */
    0x35,

    0xB0,   /* Set page address */

    0xDA,   /* Set com pins */
    0x12,

    0xa4,   /* output ram to display */

    0xa6,   /* normal / inverted colors */

    0xFF, //END
};

static esp_err_t panel_sh1107_init(esp_lcd_panel_t *panel)
{
    sh1107_panel_t *sh1107 = __containerof(panel, sh1107_panel_t, base);
    esp_lcd_panel_io_handle_t io = sh1107->io;

    // vendor specific initialization, it can be different between manufacturers
    // should consult the LCD supplier for initialization sequence code
    int cmd = 0;
    while (vendor_specific_init[cmd] != 0xff) {
        esp_lcd_panel_io_tx_param(io, LCD_SH1107_I2C_CMD, (uint8_t[]) {
            vendor_specific_init[cmd]
        }, 1);
        cmd++;
    }

    return ESP_OK;
}

static esp_err_t panel_sh1107_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    sh1107_panel_t *sh1107 = __containerof(panel, sh1107_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = sh1107->io;
    // adding extra gap
    x_start += sh1107->x_gap;
    x_end += sh1107->x_gap;
    y_start += sh1107->y_gap;
    y_end += sh1107->y_gap;

    uint8_t columnLow = x_start & 0x0F;
    uint8_t columnHigh = (x_start >> 4) & 0x0F;
    uint8_t row1 = 0, row2 = 0;
    uint32_t size = 0;
    const uint16_t *ptr;


    if (sh1107->swap_axes) {
        /* Portrait */
        row1 = y_start >> 3;
        row2 = y_end >> 3;
    } else {
        /* Landscape */
        row1 = x_start >> 3;
        row2 = x_end >> 3;
    }

    for (int i = row1; i < row2 + 1; i++) {
        esp_lcd_panel_io_tx_param(io, LCD_SH1107_I2C_CMD, (uint8_t[]) {
            0x10 | columnHigh
        }, 1);
        esp_lcd_panel_io_tx_param(io, LCD_SH1107_I2C_CMD, (uint8_t[]) {
            0x00 | columnLow
        }, 1);
        esp_lcd_panel_io_tx_param(io, LCD_SH1107_I2C_CMD, (uint8_t[]) {
            0xB0 | i
        }, 1);
        size = y_end - y_start;

        if (sh1107->swap_axes) {
            /* Portrait */
            ptr = color_data + i * sh1107->lcd_height;
        } else {
            /* Landscape */
            ptr = color_data + i * sh1107->lcd_width;
        }

        if (i != row2) {
            esp_lcd_panel_io_tx_color(io, LCD_SH1107_I2C_RAM, (uint8_t *)ptr, size);
        }
    }

    return ESP_OK;
}

static esp_err_t panel_sh1107_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    sh1107_panel_t *sh1107 = __containerof(panel, sh1107_panel_t, base);
    esp_lcd_panel_io_handle_t io = sh1107->io;

    if (invert_color_data) {
        esp_lcd_panel_io_tx_param(io, LCD_SH1107_I2C_CMD, (uint8_t[]) {
            (LCD_SH1107_PARAM_INVERT_COLOR | 0x01)
        }, 1);
    } else {
        esp_lcd_panel_io_tx_param(io, LCD_SH1107_I2C_CMD, (uint8_t[]) {
            LCD_SH1107_PARAM_INVERT_COLOR
        }, 1);
    }
    return ESP_OK;
}

static esp_err_t panel_sh1107_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    sh1107_panel_t *sh1107 = __containerof(panel, sh1107_panel_t, base);
    esp_lcd_panel_io_handle_t io = sh1107->io;
    uint8_t param_x = (LCD_SH1107_PARAM_MIRROR_X | 0x01);
    uint8_t param_y = (LCD_SH1107_PARAM_MIRROR_Y | 0x07);

    if (mirror_x) {
        param_x = LCD_SH1107_PARAM_MIRROR_X;
    }
    if (mirror_y) {
        param_y = (LCD_SH1107_PARAM_MIRROR_Y | 0x08);
    }

    esp_lcd_panel_io_tx_param(io, LCD_SH1107_I2C_CMD, (uint8_t[]) {
        param_x
    }, 1);
    esp_lcd_panel_io_tx_param(io, LCD_SH1107_I2C_CMD, (uint8_t[]) {
        param_y
    }, 1);

    return ESP_OK;
}

static esp_err_t panel_sh1107_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    sh1107_panel_t *sh1107 = __containerof(panel, sh1107_panel_t, base);
    sh1107->swap_axes = swap_axes;

    return ESP_OK;
}

static esp_err_t panel_sh1107_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    sh1107_panel_t *sh1107 = __containerof(panel, sh1107_panel_t, base);
    sh1107->x_gap = x_gap;
    sh1107->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_sh1107_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    sh1107_panel_t *sh1107 = __containerof(panel, sh1107_panel_t, base);
    esp_lcd_panel_io_handle_t io = sh1107->io;
    uint8_t param = 0;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    on_off = !on_off;
#endif

    if (on_off) {
        param = (LCD_SH1107_PARAM_ONOFF | 0x01);
    } else {
        param = LCD_SH1107_PARAM_ONOFF;
    }

    esp_lcd_panel_io_tx_param(io, LCD_SH1107_I2C_CMD, (uint8_t[]) {
        param
    }, 1);

    return ESP_OK;
}
