/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
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
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_ili9881c.h"

#define ILI9881C_CMD_OPCODE (0x22)

#define ILI9881C_GS_PANEL   (1)
#define ILI9881C_SS_PANEL   (1 << 1)
#define ILI9881C_REV_PANEL  (1 << 2)
#define ILI9881C_BGR_PANEL  (1 << 3)

static const char *TAG = "ili9881c";

static esp_err_t panel_ili9881c_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ili9881c_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ili9881c_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ili9881c_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_ili9881c_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_ili9881c_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_ili9881c_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_ili9881c_disp_on_off(esp_lcd_panel_t *panel, bool off);
static esp_err_t panel_ili9881c_sleep(esp_lcd_panel_t *panel, bool sleep);

typedef struct {
    int cmd;           /*<! The specific LCD command */
    const void *data;  /*<! Buffer that holds the command specific data */
    size_t data_bytes; /*<! Size of `data` in memory, in bytes */
} ili9881c_lcd_init_cmd_t;

static const ili9881c_lcd_init_cmd_t vendor_specific_init_code_default[] = {
    // {cmd, { data }, data_size}
    /**** CMD_Page 3 ****/
    {0xFF, (uint8_t []){0x98, 0x81, 0x03}, 3},
    {0x01, (uint8_t []){0x00}, 1},
    {0x02, (uint8_t []){0x00}, 1},
    {0x03, (uint8_t []){0x53}, 1},
    {0x04, (uint8_t []){0x53}, 1},
    {0x05, (uint8_t []){0x13}, 1},
    {0x06, (uint8_t []){0x04}, 1},
    {0x07, (uint8_t []){0x02}, 1},
    {0x08, (uint8_t []){0x02}, 1},
    {0x09, (uint8_t []){0x00}, 1},
    {0x0a, (uint8_t []){0x00}, 1},
    {0x0b, (uint8_t []){0x00}, 1},
    {0x0c, (uint8_t []){0x00}, 1},
    {0x0d, (uint8_t []){0x00}, 1},
    {0x0e, (uint8_t []){0x00}, 1},
    {0x0f, (uint8_t []){0x00}, 1},
    {0x10, (uint8_t []){0x00}, 1},
    {0x11, (uint8_t []){0x00}, 1},
    {0x12, (uint8_t []){0x00}, 1},
    {0x13, (uint8_t []){0x00}, 1},
    {0x14, (uint8_t []){0x00}, 1},
    {0x15, (uint8_t []){0x00}, 1},
    {0x16, (uint8_t []){0x00}, 1},
    {0x17, (uint8_t []){0x00}, 1},
    {0x18, (uint8_t []){0x00}, 1},
    {0x19, (uint8_t []){0x00}, 1},
    {0x1a, (uint8_t []){0x00}, 1},
    {0x1b, (uint8_t []){0x00}, 1},
    {0x1c, (uint8_t []){0x00}, 1},
    {0x1d, (uint8_t []){0x00}, 1},
    {0x1e, (uint8_t []){0xc0}, 1},
    {0x1f, (uint8_t []){0x80}, 1},
    {0x20, (uint8_t []){0x02}, 1},
    {0x21, (uint8_t []){0x09}, 1},
    {0x22, (uint8_t []){0x00}, 1},
    {0x23, (uint8_t []){0x00}, 1},
    {0x24, (uint8_t []){0x00}, 1},
    {0x25, (uint8_t []){0x00}, 1},
    {0x26, (uint8_t []){0x00}, 1},
    {0x27, (uint8_t []){0x00}, 1},
    {0x28, (uint8_t []){0x55}, 1},
    {0x29, (uint8_t []){0x03}, 1},
    {0x2a, (uint8_t []){0x00}, 1},
    {0x2b, (uint8_t []){0x00}, 1},
    {0x2c, (uint8_t []){0x00}, 1},
    {0x2d, (uint8_t []){0x00}, 1},
    {0x2e, (uint8_t []){0x00}, 1},
    {0x2f, (uint8_t []){0x00}, 1},
    {0x30, (uint8_t []){0x00}, 1},
    {0x31, (uint8_t []){0x00}, 1},
    {0x32, (uint8_t []){0x00}, 1},
    {0x33, (uint8_t []){0x00}, 1},
    {0x34, (uint8_t []){0x00}, 1},
    {0x35, (uint8_t []){0x00}, 1},
    {0x36, (uint8_t []){0x00}, 1},
    {0x37, (uint8_t []){0x00}, 1},
    {0x38, (uint8_t []){0x3C}, 1},
    {0x39, (uint8_t []){0x00}, 1},
    {0x3a, (uint8_t []){0x00}, 1},
    {0x3b, (uint8_t []){0x00}, 1},
    {0x3c, (uint8_t []){0x00}, 1},
    {0x3d, (uint8_t []){0x00}, 1},
    {0x3e, (uint8_t []){0x00}, 1},
    {0x3f, (uint8_t []){0x00}, 1},
    {0x40, (uint8_t []){0x00}, 1},
    {0x41, (uint8_t []){0x00}, 1},
    {0x42, (uint8_t []){0x00}, 1},
    {0x43, (uint8_t []){0x00}, 1},
    {0x44, (uint8_t []){0x00}, 1},
    {0x50, (uint8_t []){0x01}, 1},
    {0x51, (uint8_t []){0x23}, 1},
    {0x52, (uint8_t []){0x45}, 1},
    {0x53, (uint8_t []){0x67}, 1},
    {0x54, (uint8_t []){0x89}, 1},
    {0x55, (uint8_t []){0xab}, 1},
    {0x56, (uint8_t []){0x01}, 1},
    {0x57, (uint8_t []){0x23}, 1},
    {0x58, (uint8_t []){0x45}, 1},
    {0x59, (uint8_t []){0x67}, 1},
    {0x5a, (uint8_t []){0x89}, 1},
    {0x5b, (uint8_t []){0xab}, 1},
    {0x5c, (uint8_t []){0xcd}, 1},
    {0x5d, (uint8_t []){0xef}, 1},
    {0x5e, (uint8_t []){0x01}, 1},
    {0x5f, (uint8_t []){0x08}, 1},
    {0x60, (uint8_t []){0x02}, 1},
    {0x61, (uint8_t []){0x02}, 1},
    {0x62, (uint8_t []){0x0A}, 1},
    {0x63, (uint8_t []){0x15}, 1},
    {0x64, (uint8_t []){0x14}, 1},
    {0x65, (uint8_t []){0x02}, 1},
    {0x66, (uint8_t []){0x11}, 1},
    {0x67, (uint8_t []){0x10}, 1},
    {0x68, (uint8_t []){0x02}, 1},
    {0x69, (uint8_t []){0x0F}, 1},
    {0x6a, (uint8_t []){0x0E}, 1},
    {0x6b, (uint8_t []){0x02}, 1},
    {0x6c, (uint8_t []){0x0D}, 1},
    {0x6d, (uint8_t []){0x0C}, 1},
    {0x6e, (uint8_t []){0x06}, 1},
    {0x6f, (uint8_t []){0x02}, 1},
    {0x70, (uint8_t []){0x02}, 1},
    {0x71, (uint8_t []){0x02}, 1},
    {0x72, (uint8_t []){0x02}, 1},
    {0x73, (uint8_t []){0x02}, 1},
    {0x74, (uint8_t []){0x02}, 1},
    {0x75, (uint8_t []){0x06}, 1},
    {0x76, (uint8_t []){0x02}, 1},
    {0x77, (uint8_t []){0x02}, 1},
    {0x78, (uint8_t []){0x0A}, 1},
    {0x79, (uint8_t []){0x15}, 1},
    {0x7a, (uint8_t []){0x14}, 1},
    {0x7b, (uint8_t []){0x02}, 1},
    {0x7c, (uint8_t []){0x10}, 1},
    {0x7d, (uint8_t []){0x11}, 1},
    {0x7e, (uint8_t []){0x02}, 1},
    {0x7f, (uint8_t []){0x0C}, 1},
    {0x80, (uint8_t []){0x0D}, 1},
    {0x81, (uint8_t []){0x02}, 1},
    {0x82, (uint8_t []){0x0E}, 1},
    {0x83, (uint8_t []){0x0F}, 1},
    {0x84, (uint8_t []){0x08}, 1},
    {0x85, (uint8_t []){0x02}, 1},
    {0x86, (uint8_t []){0x02}, 1},
    {0x87, (uint8_t []){0x02}, 1},
    {0x88, (uint8_t []){0x02}, 1},
    {0x89, (uint8_t []){0x02}, 1},
    {0x8A, (uint8_t []){0x02}, 1},
    {0xFF, (uint8_t []){0x98, 0x81, 0x04}, 3},
    {0x6C, (uint8_t []){0x15}, 1},
    {0x6E, (uint8_t []){0x30}, 1},
    {0x6F, (uint8_t []){0x33}, 1},
    {0x8D, (uint8_t []){0x1F}, 1},
    {0x87, (uint8_t []){0xBA}, 1},
    {0x26, (uint8_t []){0x76}, 1},
    {0xB2, (uint8_t []){0xD1}, 1},
    {0x35, (uint8_t []){0x1F}, 1},
    {0x33, (uint8_t []){0x14}, 1},
    {0x3A, (uint8_t []){0xA9}, 1},
    {0x3B, (uint8_t []){0x3D}, 1},
    {0x38, (uint8_t []){0x01}, 1},
    {0x39, (uint8_t []){0x00}, 1},
    {0xFF, (uint8_t []){0x98, 0x81, 0x01}, 3},
    {0x22, (uint8_t []){0x09}, 1},
    {0x31, (uint8_t []){0x00}, 1},
    {0x40, (uint8_t []){0x53}, 1},
    {0x50, (uint8_t []){0xC0}, 1},
    {0x51, (uint8_t []){0xC0}, 1},
    {0x53, (uint8_t []){0x47}, 1},
    {0x55, (uint8_t []){0x46}, 1},
    {0x60, (uint8_t []){0x28}, 1},
    {0x2E, (uint8_t []){0xC8}, 1},
    {0xA0, (uint8_t []){0x01}, 1},
    {0xA1, (uint8_t []){0x10}, 1},
    {0xA2, (uint8_t []){0x1B}, 1},
    {0xA3, (uint8_t []){0x0C}, 1},
    {0xA4, (uint8_t []){0x14}, 1},
    {0xA5, (uint8_t []){0x25}, 1},
    {0xA6, (uint8_t []){0x1A}, 1},
    {0xA7, (uint8_t []){0x1D}, 1},
    {0xA8, (uint8_t []){0x68}, 1},
    {0xA9, (uint8_t []){0x1B}, 1},
    {0xAA, (uint8_t []){0x26}, 1},
    {0xAB, (uint8_t []){0x5B}, 1},
    {0xAC, (uint8_t []){0x1B}, 1},
    {0xAD, (uint8_t []){0x17}, 1},
    {0xAE, (uint8_t []){0x4F}, 1},
    {0xAF, (uint8_t []){0x24}, 1},
    {0xB0, (uint8_t []){0x2A}, 1},
    {0xB1, (uint8_t []){0x4E}, 1},
    {0xB2, (uint8_t []){0x5F}, 1},
    {0xB3, (uint8_t []){0x39}, 1},
    {0xB7, (uint8_t []){0x03}, 1},
    {0xC0, (uint8_t []){0x0F}, 1},
    {0xC1, (uint8_t []){0x1B}, 1},
    {0xC2, (uint8_t []){0x27}, 1},
    {0xC3, (uint8_t []){0x16}, 1},
    {0xC4, (uint8_t []){0x14}, 1},
    {0xC5, (uint8_t []){0x28}, 1},
    {0xC6, (uint8_t []){0x1D}, 1},
    {0xC7, (uint8_t []){0x21}, 1},
    {0xC8, (uint8_t []){0x6C}, 1},
    {0xC9, (uint8_t []){0x1B}, 1},
    {0xCA, (uint8_t []){0x26}, 1},
    {0xCB, (uint8_t []){0x5B}, 1},
    {0xCC, (uint8_t []){0x1B}, 1},
    {0xCD, (uint8_t []){0x1B}, 1},
    {0xCE, (uint8_t []){0x4F}, 1},
    {0xCF, (uint8_t []){0x24}, 1},
    {0xD0, (uint8_t []){0x2A}, 1},
    {0xD1, (uint8_t []){0x4E}, 1},
    {0xD2, (uint8_t []){0x5F}, 1},
    {0xD3, (uint8_t []){0x39}, 1},
    {0xFF, (uint8_t []){0x98, 0x81, 0x00}, 3},
    {0x35, (uint8_t []){0x00}, 1},
    {0x29, (uint8_t []){0x00}, 0},

    //============ Gamma END===========
};

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    int x_gap;
    int y_gap;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_val; // save current value of LCD_CMD_COLMOD register
    uint16_t init_cmds_size;
    bool reset_level;
} ili9881c_panel_t;

esp_err_t esp_lcd_new_panel_ili9881c(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ili9881c_panel_t *ili9881c = (ili9881c_panel_t *)calloc(1, sizeof(ili9881c_panel_t));
    ESP_RETURN_ON_FALSE(ili9881c, ESP_ERR_NO_MEM, TAG, "no mem for ili9881c panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->rgb_ele_order) {
    case LCD_RGB_ELEMENT_ORDER_RGB:
        ili9881c->madctl_val = 0;
        break;
    case LCD_RGB_ELEMENT_ORDER_BGR:
        ili9881c->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported rgb element order");
        break;
    }

    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        ili9881c->colmod_val = 0x55;
        break;
    case 18: // RGB666
        ili9881c->colmod_val = 0x66;
        break;
    case 24: // RGB888
        ili9881c->colmod_val = 0x77;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    // The ID register is on the CMD_Page 1
    uint8_t ID1, ID2, ID3;
    esp_lcd_panel_io_tx_param(io, 0xFF, (uint8_t[]) {
        0x98, 0x81, 0x01
    }, 3);
    esp_lcd_panel_io_rx_param(io, 0x00, &ID1, 1);
    esp_lcd_panel_io_rx_param(io, 0x01, &ID2, 1);
    esp_lcd_panel_io_rx_param(io, 0x02, &ID3, 1);
    ESP_LOGI(TAG, "ID1: 0x%x, ID2: 0x%x, ID3: 0x%x", ID1, ID2, ID3);

    ili9881c->io = io;
    ili9881c->reset_gpio_num = panel_dev_config->reset_gpio_num;
    ili9881c->reset_level = panel_dev_config->flags.reset_active_high;
    ili9881c->base.del = panel_ili9881c_del;
    ili9881c->base.reset = panel_ili9881c_reset;
    ili9881c->base.init = panel_ili9881c_init;
    ili9881c->base.invert_color = panel_ili9881c_invert_color;
    ili9881c->base.set_gap = panel_ili9881c_set_gap;
    ili9881c->base.mirror = panel_ili9881c_mirror;
    ili9881c->base.swap_xy = panel_ili9881c_swap_xy;
    ili9881c->base.disp_on_off = panel_ili9881c_disp_on_off;
    ili9881c->base.disp_sleep = panel_ili9881c_sleep;
    *ret_panel = &ili9881c->base;

    return ESP_OK;

err:
    if (ili9881c) {
        panel_ili9881c_del(&ili9881c->base);
    }
    return ret;
}

static esp_err_t panel_ili9881c_del(esp_lcd_panel_t *panel)
{
    ili9881c_panel_t *ili9881c = __containerof(panel, ili9881c_panel_t, base);

    if (ili9881c->reset_gpio_num >= 0) {
        gpio_reset_pin(ili9881c->reset_gpio_num);
    }
    free(ili9881c);
    return ESP_OK;
}

static esp_err_t panel_ili9881c_reset(esp_lcd_panel_t *panel)
{
    ili9881c_panel_t *ili9881c = __containerof(panel, ili9881c_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9881c->io;

    // perform hardware reset
    if (ili9881c->reset_gpio_num >= 0) {
        gpio_set_level(ili9881c->reset_gpio_num, ili9881c->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(ili9881c->reset_gpio_num, !ili9881c->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else { // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

static esp_err_t panel_ili9881c_init(esp_lcd_panel_t *panel)
{
    ili9881c_panel_t *ili9881c = __containerof(panel, ili9881c_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9881c->io;

    const ili9881c_lcd_init_cmd_t *init_cmds = vendor_specific_init_code_default;
    uint16_t init_cmds_size = sizeof(vendor_specific_init_code_default) / sizeof(ili9881c_lcd_init_cmd_t);

    // back to CMD_Page 0
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, 0xFF, (uint8_t[]) {
        0x98, 0x81, 0x00
    }, 3), TAG, "send command failed");
    // exit sleep mode
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0), TAG,
                        "io tx param failed");
    vTaskDelay(pdMS_TO_TICKS(120));

    for (int i = 0; i < init_cmds_size; i++) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, init_cmds[i].cmd, init_cmds[i].data, init_cmds[i].data_bytes), TAG, "send command failed");
    }

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        ili9881c->madctl_val,
    }, 1), TAG, "send command failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, (uint8_t[]) {
        ili9881c->colmod_val,
    }, 1), TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t panel_ili9881c_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    ili9881c_panel_t *ili9881c = __containerof(panel, ili9881c_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9881c->io;
    int command = 0;
    if (invert_color_data) {
        command = LCD_CMD_INVON;
    } else {
        command = LCD_CMD_INVOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_ili9881c_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ili9881c_panel_t *ili9881c = __containerof(panel, ili9881c_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9881c->io;
    if (mirror_x) {
        ili9881c->madctl_val |= ILI9881C_SS_PANEL;
    } else {
        ili9881c->madctl_val &= ~ILI9881C_SS_PANEL;
    }
    if (mirror_y) {
        ili9881c->madctl_val |= ILI9881C_GS_PANEL;
    } else {
        ili9881c->madctl_val &= ~ILI9881C_GS_PANEL;
    }


    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
        ili9881c->madctl_val
    }, 1), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_ili9881c_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ESP_LOGW(TAG, "Swap XY is not supported in ILI9881C driver. Please use SW rotation.");
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_ili9881c_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    ili9881c_panel_t *ili9881c = __containerof(panel, ili9881c_panel_t, base);
    ili9881c->x_gap = x_gap;
    ili9881c->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_ili9881c_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    ili9881c_panel_t *ili9881c = __containerof(panel, ili9881c_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9881c->io;
    int command = 0;

    if (on_off) {
        command = LCD_CMD_DISPON;
    } else {
        command = LCD_CMD_DISPOFF;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG, "send command failed");
    return ESP_OK;
}

static esp_err_t panel_ili9881c_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    ili9881c_panel_t *ili9881c = __containerof(panel, ili9881c_panel_t, base);
    esp_lcd_panel_io_handle_t io = ili9881c->io;
    int command = 0;
    if (sleep) {
        command = LCD_CMD_SLPIN;
    } else {
        command = LCD_CMD_SLPOUT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, command, NULL, 0), TAG,
                        "io tx param failed");
    vTaskDelay(pdMS_TO_TICKS(100));

    return ESP_OK;
}
