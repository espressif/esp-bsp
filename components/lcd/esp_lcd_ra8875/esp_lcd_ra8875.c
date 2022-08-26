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
#include "esp_lcd_ra8875.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"

#define ESP_RA8875_TIMEOUT_US   (10*1000)

static const char *TAG = "ra8875";

static esp_err_t panel_ra8875_del(esp_lcd_panel_t *panel);
static esp_err_t panel_ra8875_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_ra8875_init(esp_lcd_panel_t *panel);
static esp_err_t panel_ra8875_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_ra8875_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_ra8875_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_ra8875_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_ra8875_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_ra8875_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int wait_gpio_num;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    unsigned int bits_per_pixel;
    uint16_t lcd_width;
    uint16_t lcd_height;
    uint8_t sysr; // save surrent value of System Configuration Register (Color Depth settings and 8-bit/16-bit interface)
    bool swap_axes;
} ra8875_panel_t;

esp_err_t esp_lcd_new_panel_ra8875(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    ra8875_panel_t *ra8875 = NULL;
    esp_lcd_panel_ra8875_config_t *vendor_cfg = (esp_lcd_panel_ra8875_config_t *) panel_dev_config->vendor_config;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    ESP_GOTO_ON_FALSE(vendor_cfg, ESP_ERR_INVALID_ARG, err, TAG, "vendor config cannot be null");
    ra8875 = calloc(1, sizeof(ra8875_panel_t));
    ESP_GOTO_ON_FALSE(ra8875, ESP_ERR_NO_MEM, err, TAG, "no mem for ra8875 panel");

    // Reset GPIO config
    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    // Wait GPIO config
    if (vendor_cfg->wait_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = 1ULL << vendor_cfg->wait_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for WAIT line failed");
    }

    // The RA8875 supports only RGB endian
    // Note: Color space used for compatibility with IDF v4.4
    switch (panel_dev_config->color_space) {
    case ESP_LCD_COLOR_SPACE_RGB:
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }

    switch (panel_dev_config->bits_per_pixel) {
    case 8:
        ra8875->sysr = 0x00;
        break;
    case 16:
        ra8875->sysr = 0x08;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width (supported only 8-bit and 16-bit)");
        break;
    }

    switch (vendor_cfg->mcu_bit_interface) {
    case 8:
        ra8875->sysr |= 0x00;
        break;
    case 16:
        ra8875->sysr |= 0x02;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported bit interface (supported only 8-bit and 16-bit");
        break;
    }

    ra8875->io = io;
    ra8875->lcd_width = vendor_cfg->lcd_width;
    ra8875->lcd_height = vendor_cfg->lcd_height;
    ra8875->wait_gpio_num = vendor_cfg->wait_gpio_num;
    ra8875->bits_per_pixel = panel_dev_config->bits_per_pixel;
    ra8875->reset_gpio_num = panel_dev_config->reset_gpio_num;
    ra8875->reset_level = panel_dev_config->flags.reset_active_high;
    ra8875->base.del = panel_ra8875_del;
    ra8875->base.reset = panel_ra8875_reset;
    ra8875->base.init = panel_ra8875_init;
    ra8875->base.draw_bitmap = panel_ra8875_draw_bitmap;
    ra8875->base.invert_color = panel_ra8875_invert_color;
    ra8875->base.set_gap = panel_ra8875_set_gap;
    ra8875->base.mirror = panel_ra8875_mirror;
    ra8875->base.swap_xy = panel_ra8875_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    ra8875->base.disp_off = panel_ra8875_disp_on_off;
#else
    ra8875->base.disp_on_off = panel_ra8875_disp_on_off;
#endif
    *ret_panel = &(ra8875->base);
    ESP_LOGD(TAG, "new ra8875 panel @%p", ra8875);

    return ESP_OK;

err:
    if (ra8875) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(ra8875);
    }
    return ret;
}

static esp_err_t panel_ra8875_del(esp_lcd_panel_t *panel)
{
    ra8875_panel_t *ra8875 = __containerof(panel, ra8875_panel_t, base);

    if (ra8875->reset_gpio_num >= 0) {
        gpio_reset_pin(ra8875->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del ra8875 panel @%p", ra8875);
    free(ra8875);
    return ESP_OK;
}

static esp_err_t panel_ra8875_reset(esp_lcd_panel_t *panel)
{
    ra8875_panel_t *ra8875 = __containerof(panel, ra8875_panel_t, base);
    esp_lcd_panel_io_handle_t io = ra8875->io;

    // perform hardware reset
    if (ra8875->reset_gpio_num >= 0) {
        gpio_set_level(ra8875->reset_gpio_num, ra8875->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(ra8875->reset_gpio_num, !ra8875->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else { // perform software reset
        esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

typedef struct {
    uint16_t cmd;
    uint8_t data[16];
    uint8_t data_bytes; // Length of data in above data array; 0xFF = end of cmds.
} lcd_init_cmd_t;

static const lcd_init_cmd_t vendor_specific_init[] = {
    /* PLL init */
    {0x88, {0x0b}, 1},
    {0x89, {0x01}, 1},

    /* Pixel Clock */
    {0x04, {0x81}, 1},

    {0x15, {0x03}, 1},
    {0x16, {0x03}, 1},
    {0x17, {0x02}, 1},
    {0x18, {0x00}, 1},

    {0x1b, {0x14}, 1},
    {0x1c, {0x00}, 1},
    {0x1d, {0x06}, 1},
    {0x1e, {0x00}, 1},
    {0x1f, {0x01}, 1},

    {0x30, {0x00}, 1},
    {0x31, {0x00}, 1},
    {0x34, {0x1f}, 1},
    {0x35, {0x03}, 1},

    {0x32, {0x00}, 1},
    {0x33, {0x00}, 1},
    {0x36, {0xdf}, 1},
    {0x37, {0x01}, 1},

    /* Backlight PWM1 - settings */
    {0x8a, {0x95}, 1},
    /* Backlight PWM1 - duty */
    {0x8b, {0x05}, 1},

    /* End */
    {0, {0}, 0xff},
};

static void panel_ra8875_wait(esp_lcd_panel_t *panel)
{
    ra8875_panel_t *ra8875 = __containerof(panel, ra8875_panel_t, base);
    if (ra8875->wait_gpio_num >= 0) {
        uint64_t start = esp_timer_get_time();
        uint64_t now = start;
        while (gpio_get_level(ra8875->wait_gpio_num) == 0 && ((now = esp_timer_get_time()) - start) < ESP_RA8875_TIMEOUT_US);
        if ((now - start) > ESP_RA8875_TIMEOUT_US) {
            ESP_LOGE(TAG, "RA8875 Timeout!");
            ESP_ERROR_CHECK(ESP_ERR_TIMEOUT);
        }
    }
}

static esp_err_t panel_ra8875_tx_param(esp_lcd_panel_t *panel, int lcd_cmd, uint8_t param)
{
    ra8875_panel_t *ra8875 = __containerof(panel, ra8875_panel_t, base);
    esp_lcd_panel_io_handle_t io = ra8875->io;

    panel_ra8875_wait(panel);
    return esp_lcd_panel_io_tx_param(io, lcd_cmd, (uint8_t[]) {
        param,
    }, 1);
}

static esp_err_t panel_ra8875_init(esp_lcd_panel_t *panel)
{
    ra8875_panel_t *ra8875 = __containerof(panel, ra8875_panel_t, base);

    // MCU bit interface, color bits
    panel_ra8875_tx_param(panel, 0x10, ra8875->sysr);

    // vendor specific initialization, it can be different between manufacturers
    // should consult the LCD supplier for initialization sequence code
    int cmd = 0;
    while (vendor_specific_init[cmd].data_bytes != 0xff) {
        panel_ra8875_tx_param(panel, vendor_specific_init[cmd].cmd, vendor_specific_init[cmd].data[0]);
        cmd++;
    }

    // Horizontal Display Width
    uint16_t hdwr = (ra8875->lcd_width / 8) - 1;
    panel_ra8875_tx_param(panel, 0x14, hdwr);

    // Vertical Display Heigh
    uint16_t vdhr = ra8875->lcd_height - 1;
    panel_ra8875_tx_param(panel, 0x19, (vdhr & 0xFF));
    panel_ra8875_tx_param(panel, 0x1a, ((vdhr >> 8) & 0xFF));

    return ESP_OK;
}

static void panel_ra8875_set_window(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end)
{
    ra8875_panel_t *ra8875 = __containerof(panel, ra8875_panel_t, base);

    if (ra8875->swap_axes) {
        int xs = x_start;
        int xe = x_end;

        x_start = y_start;
        y_start = xs;

        x_end = y_end;
        y_end = xe;
    }

    panel_ra8875_tx_param(panel, 0x30, x_start);
    panel_ra8875_tx_param(panel, 0x31, (x_start >> 8));
    panel_ra8875_tx_param(panel, 0x32, y_start);
    panel_ra8875_tx_param(panel, 0x33, (y_start >> 8));
    panel_ra8875_tx_param(panel, 0x34, (x_end - 1));
    panel_ra8875_tx_param(panel, 0x35, ((x_end - 1) >> 8));
    panel_ra8875_tx_param(panel, 0x36, (y_end - 1));
    panel_ra8875_tx_param(panel, 0x37, ((y_end - 1) >> 8));
}

static void panel_ra8875_set_cursor(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end)
{
    ra8875_panel_t *ra8875 = __containerof(panel, ra8875_panel_t, base);

    if (ra8875->swap_axes) {
        int xs = x_start;

        x_start = y_start;
        y_start = xs;
    }

    panel_ra8875_tx_param(panel, 0x46, x_start);
    panel_ra8875_tx_param(panel, 0x47, (x_start >> 8));
    panel_ra8875_tx_param(panel, 0x48, y_start);
    panel_ra8875_tx_param(panel, 0x49, y_start >> 8);
}

static esp_err_t panel_ra8875_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    ra8875_panel_t *ra8875 = __containerof(panel, ra8875_panel_t, base);
    esp_lcd_panel_io_handle_t io = ra8875->io;
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");

    x_start += ra8875->x_gap;
    x_end += ra8875->x_gap;
    y_start += ra8875->y_gap;
    y_end += ra8875->y_gap;

    // define an area of frame memory where MCU can access
    panel_ra8875_set_window(panel, x_start, y_start, x_end, y_end);

    // set cursor
    panel_ra8875_set_cursor(panel, x_start, y_start, x_end, y_end);

    /* Write to graphic RAM */
    size_t len = (x_end - x_start) * (y_end - y_start) * ra8875->bits_per_pixel / 8;
    esp_lcd_panel_io_tx_color(io, 0x02, color_data, len);

    return ESP_OK;
}

static esp_err_t panel_ra8875_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    ESP_LOGE(TAG, "invert color is unsupported");
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_ra8875_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    uint8_t param = 0;

    if (mirror_y) {
        param |= 0x04;
    }
    if (mirror_x) {
        param |= 0x08;
    }

    panel_ra8875_tx_param(panel, 0x20, param);

    return ESP_OK;
}

static esp_err_t panel_ra8875_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    ra8875_panel_t *ra8875 = __containerof(panel, ra8875_panel_t, base);
    ra8875->swap_axes = swap_axes;

    // Graphic mode
    if (ra8875->swap_axes) {
        // Memory Write Direction: Left --> Right then Top --> Down
        panel_ra8875_tx_param(panel, 0x40, 0x08);
    } else {
        // Memory Write Direction: Top --> Down then Left --> Right.
        panel_ra8875_tx_param(panel, 0x40, 0x00);
    }

    return ESP_OK;
}

static esp_err_t panel_ra8875_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    ra8875_panel_t *ra8875 = __containerof(panel, ra8875_panel_t, base);
    ra8875->x_gap = x_gap;
    ra8875->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_ra8875_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    uint8_t param = 0;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    on_off = !on_off;
#endif

    if (on_off) {
        param = 0x80;
    } else {
        param = 0x00;
    }
    panel_ra8875_tx_param(panel, 0x01, param);
    return ESP_OK;
}
