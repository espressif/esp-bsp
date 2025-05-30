/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
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
#include "esp_lcd_lt8912b.h"

#define ENABLE_TEST_PATTERN 0

static const char *TAG = "lt8912b";

static esp_err_t panel_lt8912b_del(esp_lcd_panel_t *panel);
static esp_err_t panel_lt8912b_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_lt8912b_init(esp_lcd_panel_t *panel);
static esp_err_t panel_lt8912b_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_lt8912b_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_lt8912b_disp_on_off(esp_lcd_panel_t *panel, bool off);
static esp_err_t panel_lt8912b_sleep(esp_lcd_panel_t *panel, bool sleep);

static esp_err_t _panel_lt8912b_detect_input_mipi(esp_lcd_panel_t *panel);
static bool _panel_lt8912b_get_hpd(esp_lcd_panel_t *panel);

typedef struct {
    esp_lcd_panel_lt8912b_io_t io;
    esp_lcd_panel_lt8912b_video_timing_t video_timing;
    int reset_gpio_num;
    bool reset_level;
    uint8_t lane_num;
    // To save the original functions of MIPI DPI panel
    esp_err_t (*del)(esp_lcd_panel_t *panel);
    esp_err_t (*init)(esp_lcd_panel_t *panel);
} lt8912b_panel_t;

typedef struct {
    uint8_t cmd;
    uint8_t data;
} lt8912b_lcd_init_cmd_t;

static const lt8912b_lcd_init_cmd_t cmd_digital_clock_en[] = {
    // {cmd, data}

    /* Digital clock en*/
    {0x02, 0xf7},
    {0x08, 0xff},
    {0x09, 0xff},
    {0x0a, 0xff},
    {0x0b, 0x7c},
    {0x0c, 0xff},
};

static const lt8912b_lcd_init_cmd_t cmd_tx_analog[] = {
    // {cmd, data}

    /*Tx Analog*/
    {0x31, 0xe1},
    {0x32, 0xe1},
    {0x33, 0x0c},
    {0x37, 0x00},
    {0x38, 0x22},
    {0x60, 0x82},
};

static const lt8912b_lcd_init_cmd_t cmd_cbus_analog[] = {
    // {cmd, data}

    /*Cbus Analog*/
    {0x39, 0x45},
    {0x3a, 0x00},
    {0x3b, 0x00},
};

static const lt8912b_lcd_init_cmd_t cmd_hdmi_pll_analog[] = {
    // {cmd, data}

    /*HDMI Pll Analog*/
    {0x44, 0x31},
    {0x55, 0x44},
    {0x57, 0x01},
    {0x5a, 0x02},
};

static const lt8912b_lcd_init_cmd_t cmd_audio_iis_mode[] = {
    // {cmd, data}

    /*Audio IIs Mode */
    {0xB2, 0x01}, //DVI mode:0x00; HDMI mode:0x01;
};

static const lt8912b_lcd_init_cmd_t cmd_audio_iis_en[] = {
    // {cmd, data}

    /*Audio IIs En*/
    {0x06, 0x08},
    {0x07, 0xF0},
    {0x34, 0xD2},
    {0x0F, 0x2B},
};

static const lt8912b_lcd_init_cmd_t cmd_lvds[] = {
    // {cmd, data}

    /* Lvds Power Up */
    {0x44, 0x30},
    {0x51, 0x05},

    /*Lvds Bypass*/
    {0x50, 0x24}, //cp=50uA
    {0x51, 0x2d}, //Pix_clk as reference,second order passive LPF PLL
    {0x52, 0x04}, //loopdiv=0;use second-order PLL
    {0x69, 0x0e}, //CP_PRESET_DIV_RATIO
    {0x69, 0x8e},
    {0x6a, 0x00},
    {0x6c, 0xb8}, //RGD_CP_SOFT_K_EN,RGD_CP_SOFT_K[13:8]
    {0x6b, 0x51},

    {0x04, 0xfb}, //core pll reset
    {0x04, 0xff},

    {0x7f, 0x00}, //disable scaler
    {0xa8, 0x13}, //0x13:VSEA ; 0x33:JEIDA;
};

static const lt8912b_lcd_init_cmd_t cmd_dds_config[] = {
    // {cmd, data}

    /*DDS Config*/
    {0x4e, 0x93}, //strm_sw_freq_word[ 7: 0]
    {0x4f, 0x3E}, //strm_sw_freq_word[15: 8]
    {0x50, 0x29}, //strm_sw_freq_word[23:16]
    {0x51, 0x80}, //[0]=strm_sw_freq_word[24]
    {0x1e, 0x4f},
    {0x1f, 0x5e}, //full_value   464
    {0x20, 0x01},
    {0x21, 0x2c}, //full_value1  416
    {0x22, 0x01},
    {0x23, 0xfa}, //full_value2  400
    {0x24, 0x00},
    {0x25, 0xc8}, //full_value3  384
    {0x26, 0x00},
    {0x27, 0x5e}, //empty_value   464
    {0x28, 0x01},
    {0x29, 0x2c}, //empty_value1  416
    {0x2a, 0x01},
    {0x2b, 0xfa}, //empty_value2  400
    {0x2c, 0x00},
    {0x2d, 0xc8}, //empty_value3  384
    {0x2e, 0x00},
    {0x42, 0x64}, //tmr_set[ 7:0]:100us
    {0x43, 0x00}, //tmr_set[15:8]:100us
    {0x44, 0x04}, //timer step
    {0x45, 0x00},
    {0x46, 0x59},
    {0x47, 0x00},
    {0x48, 0xf2},
    {0x49, 0x06},
    {0x4a, 0x00},
    {0x4b, 0x72},
    {0x4c, 0x45},
    {0x4d, 0x00},
    {0x52, 0x08}, // trend step
    {0x53, 0x00},
    {0x54, 0xb2},
    {0x55, 0x00},
    {0x56, 0xe4},
    {0x57, 0x0d},
    {0x58, 0x00},
    {0x59, 0xe4},
    {0x5a, 0x8a},
    {0x5b, 0x00},
    {0x5c, 0x34},
    {0x51, 0x00},
};

esp_err_t esp_lcd_new_panel_lt8912b(const esp_lcd_panel_lt8912b_io_t *io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    ESP_RETURN_ON_FALSE(io && io->main && io->cec_dsi && io->avi && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    lt8912b_vendor_config_t *vendor_config = (lt8912b_vendor_config_t *)panel_dev_config->vendor_config;
    ESP_RETURN_ON_FALSE(vendor_config && vendor_config->mipi_config.dpi_config && vendor_config->mipi_config.dsi_bus, ESP_ERR_INVALID_ARG, TAG, "invalid vendor config");


    esp_err_t ret = ESP_OK;
    lt8912b_panel_t *lt8912b = (lt8912b_panel_t *)calloc(1, sizeof(lt8912b_panel_t));
    ESP_RETURN_ON_FALSE(lt8912b, ESP_ERR_NO_MEM, TAG, "no mem for lt8912b panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    memcpy(&lt8912b->io, io, sizeof(esp_lcd_panel_lt8912b_io_t));
    memcpy(&lt8912b->video_timing, &vendor_config->video_timing, sizeof(esp_lcd_panel_lt8912b_video_timing_t));
    lt8912b->lane_num = vendor_config->mipi_config.lane_num;
    lt8912b->reset_gpio_num = panel_dev_config->reset_gpio_num;
    lt8912b->reset_level = panel_dev_config->flags.reset_active_high;

    // Create MIPI DPI panel
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_dpi(vendor_config->mipi_config.dsi_bus, vendor_config->mipi_config.dpi_config, ret_panel), err, TAG,
                      "create MIPI DPI panel failed");
    ESP_LOGD(TAG, "new MIPI DPI panel @%p", *ret_panel);

    // Save the original functions of MIPI DPI panel
    lt8912b->del = (*ret_panel)->del;
    lt8912b->init = (*ret_panel)->init;
    // Overwrite the functions of MIPI DPI panel
    (*ret_panel)->del = panel_lt8912b_del;
    (*ret_panel)->init = panel_lt8912b_init;
    (*ret_panel)->reset = panel_lt8912b_reset;
    (*ret_panel)->mirror = panel_lt8912b_mirror;
    (*ret_panel)->invert_color = panel_lt8912b_invert_color;
    (*ret_panel)->disp_on_off = panel_lt8912b_disp_on_off;
    (*ret_panel)->disp_sleep = panel_lt8912b_sleep;
    (*ret_panel)->user_data = lt8912b;
    ESP_LOGD(TAG, "new lt8912b panel @%p", lt8912b);

    return ESP_OK;

err:
    if (lt8912b) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(lt8912b);
    }
    return ret;
}

bool esp_lcd_panel_lt8912b_is_ready(esp_lcd_panel_t *panel)
{
    _panel_lt8912b_detect_input_mipi(panel);
    return _panel_lt8912b_get_hpd(panel);
}

static esp_err_t panel_lt8912b_del(esp_lcd_panel_t *panel)
{
    lt8912b_panel_t *lt8912b = (lt8912b_panel_t *)panel->user_data;

    if (lt8912b->reset_gpio_num >= 0) {
        gpio_reset_pin(lt8912b->reset_gpio_num);
    }
    // Delete MIPI DPI panel
    lt8912b->del(panel);
    free(lt8912b);
    return ESP_OK;
}

static esp_err_t panel_lt8912b_reset(esp_lcd_panel_t *panel)
{
    lt8912b_panel_t *lt8912b = (lt8912b_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io_main = lt8912b->io.main;

    // perform hardware reset
    if (lt8912b->reset_gpio_num >= 0) {
        gpio_set_level(lt8912b->reset_gpio_num, lt8912b->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(lt8912b->reset_gpio_num, !lt8912b->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    } else { // perform software reset
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io_main, LCD_CMD_SWRESET, NULL, 0), TAG, "send command failed");
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

static esp_err_t _panel_lt8912b_send_data(esp_lcd_panel_io_handle_t io, uint8_t reg, uint8_t data)
{
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, reg, (uint8_t []) {
        data
    }, 1), TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t _panel_lt8912b_send_data_array(esp_lcd_panel_io_handle_t io, const lt8912b_lcd_init_cmd_t *cmds, uint16_t cmds_size)
{
    for (int i = 0; i < cmds_size; i++) {
        ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io, cmds[i].cmd, cmds[i].data), TAG, "send command failed");
    }

    return ESP_OK;
}

static esp_err_t _panel_lt8912b_send_mipi_analog(esp_lcd_panel_io_handle_t io_main, bool pn_swap)
{
    /* P/N swap or not */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x3e, (pn_swap ? 0xf6 : 0xd6)), TAG, "send command failed");

    /* EQ */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x3f, 0xd4), TAG, "send command failed");
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x41, 0x3c), TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t _panel_lt8912b_send_mipi_basic_set(esp_lcd_panel_io_handle_t io_cec, uint8_t lane_count, bool lane_swap)
{
    /* term en */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec, 0x10, 0x01), TAG, "send command failed");
    /* settle */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec, 0x11, 0x10), TAG, "send command failed");
    /* trail */
    //ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec, 0x12, 0x08), TAG, "send command failed");
    /* 00: 4 lane, 01: 1 lane, 02: 2 lane, 03: 3lane */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec, 0x13, lane_count), TAG, "send command failed");
    /* debug mux */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec, 0x14, 0x00), TAG, "send command failed");
    /* Lane swap */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec, 0x15, (lane_swap ? 0xa8 : 0x00)), TAG, "send command failed");

    /* hshift 3 */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec, 0x1a, 0x03), TAG, "send command failed");
    /* vshift 3 */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec, 0x1b, 0x03), TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t _panel_lt8912b_send_video_setup(esp_lcd_panel_t *panel)
{
    lt8912b_panel_t *lt8912b = (lt8912b_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io_cec_dsi = lt8912b->io.cec_dsi;
    esp_lcd_panel_lt8912b_video_timing_t *video_format = &lt8912b->video_timing;

    /* hwidth */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x18, (uint8_t)(video_format->hs % 256)), TAG, "send command failed");
    /* vwidth */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x19, (uint8_t)(video_format->vs % 256)), TAG, "send command failed");
    /* H_active[7:0] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x1c, (uint8_t)(video_format->hact % 256)), TAG, "send command failed");
    /* H_active[15:8] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x1d, (uint8_t)(video_format->hact / 256)), TAG, "send command failed");
    /* fifo_buff_length 12 */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x2f, 0x0c), TAG, "send command failed");
    /* H_total[7:0] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x34, (uint8_t)(video_format->htotal % 256)), TAG, "send command failed");
    /* H_total[15:8] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x35, (uint8_t)(video_format->htotal / 256)), TAG, "send command failed");
    /* V_total[7:0] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x36, (uint8_t)(video_format->vtotal % 256)), TAG, "send command failed");
    /* V_total[15:8] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x37, (uint8_t)(video_format->vtotal / 256)), TAG, "send command failed");
    /* VBP[7:0] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x38, (uint8_t)(video_format->vbp % 256)), TAG, "send command failed");
    /* VBP[15:8] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x39, (uint8_t)(video_format->vbp / 256)), TAG, "send command failed");
    /* VFP[7:0] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x3a, (uint8_t)(video_format->vfp % 256)), TAG, "send command failed");
    /* VFP[15:8] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x3b, (uint8_t)(video_format->vfp / 256)), TAG, "send command failed");
    /* HBP[7:0] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x3c, (uint8_t)(video_format->hbp % 256)), TAG, "send command failed");
    /* HBP[15:8] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x3d, (uint8_t)(video_format->hbp / 256)), TAG, "send command failed");
    /* HFP[7:0] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x3e, (uint8_t)(video_format->hfp % 256)), TAG, "send command failed");
    /* HFP[15:8] */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x3f, (uint8_t)(video_format->hfp / 256)), TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t _panel_lt8912b_send_avi_infoframe(esp_lcd_panel_t *panel)
{
    lt8912b_panel_t *lt8912b = (lt8912b_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io_main = lt8912b->io.main;
    esp_lcd_panel_io_handle_t io_avi = lt8912b->io.avi;
    esp_lcd_panel_lt8912b_video_timing_t *video_timing = &lt8912b->video_timing;

    uint8_t vic = video_timing->vic;
    uint8_t aspect_ratio = video_timing->aspect_ratio;
    uint8_t pb0, pb2, pb4, sync_polarity;

    /* enable null package */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_avi, 0x3c, 0x41), TAG, "send command failed");

    sync_polarity = (video_timing->h_polarity * 0x02) + (video_timing->v_polarity * 0x01);
    pb2 =  (aspect_ratio << 4) + 0x08;
    pb4 =  vic;
    pb0 = (((pb2 + pb4) <= 0x5f) ? (0x5f - pb2 - pb4) : (0x15f - pb2 - pb4));

    /* sync polarity */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0xab, sync_polarity), TAG, "send command failed");

    /* PB0:check sum */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_avi, 0x43, pb0), TAG, "send command failed");
    /* PB1:RGB888 */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_avi, 0x44, 0x10), TAG, "send command failed");
    /* PB2 */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_avi, 0x45, pb2), TAG, "send command failed");
    /* PB3 */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_avi, 0x46, 0x00), TAG, "send command failed");
    /* PB4:vic */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_avi, 0x47, pb4), TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t _panel_lt8912b_mipi_rx_logic_reset(esp_lcd_panel_io_handle_t io_main)
{
    /* mipi rx reset */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x03, 0x7f), TAG, "send command failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x03, 0xff), TAG, "send command failed");

    /* DDS Reset */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x05, 0xfb), TAG, "send command failed");
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x05, 0xff), TAG, "send command failed");

    return ESP_OK;
}

static esp_err_t _panel_lt8912b_detect_input_mipi(esp_lcd_panel_t *panel)
{
    lt8912b_panel_t *lt8912b = (lt8912b_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io_main = lt8912b->io.main;

    uint8_t hsync_l = 0, hsync_h = 0, vsync_l = 0, vsync_h = 0;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(io_main, 0x9c, &hsync_l, 1), TAG, "read data failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(io_main, 0x9d, &hsync_h, 1), TAG, "read data failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(io_main, 0x9e, &vsync_l, 1), TAG, "read data failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(io_main, 0x9f, &vsync_h, 1), TAG, "read data failed");

    ESP_LOGI(TAG, "Detected MIPI input. H sync: 0x%02x 0x%02x, V sync: 0x%02x 0x%02x", hsync_h, hsync_l, vsync_h, vsync_l);


    /* MIPI Video Setup */
    //ESP_RETURN_ON_ERROR(_panel_lt8912b_send_video_setup(panel), TAG, "send command failed");

    /* AVI Infoframe */
    //ESP_RETURN_ON_ERROR(_panel_lt8912b_send_avi_infoframe(panel), TAG, "send command failed");

    /* MIPI RX Logic Reset */
    //ESP_RETURN_ON_ERROR(_panel_lt8912b_mipi_rx_logic_reset(io_main), TAG, "send command failed");

    return ESP_OK;
}

static bool _panel_lt8912b_get_hpd(esp_lcd_panel_t *panel)
{
    lt8912b_panel_t *lt8912b = (lt8912b_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io_main = lt8912b->io.main;

    uint8_t data = 0;
    esp_lcd_panel_io_rx_param(io_main, 0xc1, &data, 1);

    if ((data & 0x80) == 0x80) {
        return true;
    }
    return false;
}

static esp_err_t _panel_lt8912b_lvds_output(esp_lcd_panel_io_handle_t io_main, bool on)
{
    if (on) {
        /* lvds pll reset */
        ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x02, 0xf7), TAG, "read data failed");
        /* scaler module reset */
        ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x02, 0xff), TAG, "read data failed");
        /* lvds tx module reset */
        ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x03, 0xcb), TAG, "read data failed");

        ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x03, 0xfb), TAG, "read data failed");
        ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x03, 0xff), TAG, "read data failed");

        /* enbale lvds output */
        ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x44, 0x30), TAG, "read data failed");

        ESP_LOGI(TAG, "LT8912 LVDS output enabled!");
    } else {
        ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x44, 0x31), TAG, "read data failed");
        ESP_LOGI(TAG, "LT8912 LVDS output disabled!");
    }

    return ESP_OK;
}

static esp_err_t _panel_lt8912b_hdmi_output(esp_lcd_panel_io_handle_t io_main, bool on)
{
    if (on) {
        /* enable hdmi output */
        ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x33, 0x0e), TAG, "read data failed");
        ESP_LOGI(TAG, "LT8912 HDMI output enabled!");
    } else {
        /* disable hdmi output */
        ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_main, 0x33, 0x0c), TAG, "read data failed");
        ESP_LOGI(TAG, "LT8912 HDMI output disabled!");
    }

    return ESP_OK;
}

#if ENABLE_TEST_PATTERN
static esp_err_t _panel_lt8912b_send_test(esp_lcd_panel_t *panel)
{
    uint32_t DDS_initial_value;
    uint8_t pclk_Mhz;

    lt8912b_panel_t *lt8912b = (lt8912b_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io_cec_dsi = lt8912b->io.cec_dsi;
    esp_lcd_panel_lt8912b_video_timing_t *video_format = &lt8912b->video_timing;

    /************* Pattern resolution set *************/
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x72, 0x12), TAG, "read data failed");
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x73, (uint8_t)((video_format->hs + video_format->hbp) % 256)), TAG, "send command failed"); //RGD_PTN_DE_DLY[7:0]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x74, (uint8_t)((video_format->hs + video_format->hbp) / 256)), TAG, "send command failed"); //RGD_PTN_DE_DLY[11:8]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x75, (uint8_t)((video_format->vs + video_format->vbp) % 256)), TAG, "send command failed"); //RGD_PTN_DE_TOP[6:0]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x76, (uint8_t)(video_format->hact % 256)), TAG, "send command failed"); //RGD_PTN_DE_CNT[7:0]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x77, (uint8_t)(video_format->vact % 256)), TAG, "send command failed"); //RGD_PTN_DE_LIN[7:0]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x78, ((((uint8_t)(video_format->vact / 256)) << 4) + ((uint8_t)(video_format->hact / 256)))), TAG, "send command failed"); //RGD_PTN_DE_LIN[10:8],RGD_PTN_DE_CNT[11:8]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x79, (uint8_t)(video_format->htotal % 256)), TAG, "send command failed"); //RGD_PTN_H_TOTAL[7:0]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x7a, (uint8_t)(video_format->vtotal % 256)), TAG, "send command failed"); //RGD_PTN_V_TOTAL[7:0]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x7b, ((((uint8_t)(video_format->vtotal / 256)) << 4) + ((uint8_t)(video_format->htotal / 256)))), TAG, "send command failed"); //RGD_PTN_V_TOTAL[10:8],RGD_PTN_H_TOTAL[11:8]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x7c, (uint8_t)(video_format->hs % 256)), TAG, "send command failed"); //RGD_PTN_HWIDTH[7:0]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x7d, ((((uint8_t)(video_format->hs / 256)) << 6) + ((uint8_t)(video_format->vs % 256)))), TAG, "send command failed"); //RGD_PTN_HWIDTH[9:8],RGD_PTN_VWIDTH[5:0]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x70, 0x80), TAG, "send command failed"); //pattern enable
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x71, 0x51), TAG, "send command failed");
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x42, 0x12), TAG, "send command failed");
    /**************************************************/

    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x1e, 0x67), TAG, "send command failed"); //h v d pol hdmi sel pll sel


    /************* Pattern pixl clock set *************/
    uint32_t DDS_initial_value = (uint32_t)(video_format->pclk_mhz * 0x16C16);
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x4e, (uint8_t)(DDS_initial_value & 0x000000ff)), TAG, "send command failed"); //strm_sw_freq_word[ 7: 0]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x4f, (uint8_t)((DDS_initial_value & 0x0000ff00) >> 8)), TAG, "send command failed"); //strm_sw_freq_word[15: 8]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x50, (uint8_t)((DDS_initial_value & 0x00ff0000) >> 16)), TAG, "send command failed"); //strm_sw_freq_word[23:16]
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x51, 0x80), TAG, "send command failed");

//  ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x4e, 0x3E), TAG, "send command failed");  //strm_sw_freq_word[ 7: 0]
//  ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x4f, 0xE9), TAG, "send command failed");  //strm_sw_freq_word[15: 8]
//  ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x50, 0xD3), TAG, "send command failed");  //strm_sw_freq_word[23:16]
//  ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data(io_cec_dsi, 0x51, 0x80), TAG, "send command failed");  //pattern en
    /**************************************************/

    ESP_LOGW(TAG, "Test pattern enabled!");

    return ESP_OK;
}
#endif //ENABLE_TEST_PATTERN

#if 0 //For testing purposes
static esp_err_t _panel_lt8912b_read_dds(esp_lcd_panel_t *panel)
{
    lt8912b_panel_t *lt8912b = (lt8912b_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io_cec_dsi = lt8912b->io.cec_dsi;

    uint8_t reg_920c = 0, reg_920d = 0, reg_920e = 0, reg_920f = 0;

    for (int i = 0; i < 10; i++) {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(io_cec_dsi, 0x0c, &reg_920c, 1), TAG, "read data failed");
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(io_cec_dsi, 0x0d, &reg_920d, 1), TAG, "read data failed");
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(io_cec_dsi, 0x0e, &reg_920e, 1), TAG, "read data failed");
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(io_cec_dsi, 0x0f, &reg_920f, 1), TAG, "read data failed");

        ESP_LOGI(TAG, "DDS: 0x%02x 0x%02x 0x%02x 0x%02x", reg_920c, reg_920d, reg_920e, reg_920f);
        if ((reg_920e == 0xd2) && (reg_920d < 0xff) && (reg_920d > 0xd0)) { //shall update threshold here base on actual dds result.
            ESP_LOGI(TAG, "lvds_check_dds: stable!");
        }
    }

    return ESP_OK;
}
#endif

static esp_err_t panel_lt8912b_init(esp_lcd_panel_t *panel)
{
    lt8912b_panel_t *lt8912b = (lt8912b_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io_main = lt8912b->io.main;
    esp_lcd_panel_io_handle_t io_cec_dsi = lt8912b->io.cec_dsi;
    esp_lcd_panel_io_handle_t io_avi = lt8912b->io.avi;

    /* Digital Clock En */
    uint16_t cmds_size = sizeof(cmd_digital_clock_en) / sizeof(lt8912b_lcd_init_cmd_t);
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data_array(io_main, cmd_digital_clock_en, cmds_size), TAG, "send command failed");

    /* Tx Analog */
    cmds_size = sizeof(cmd_tx_analog) / sizeof(lt8912b_lcd_init_cmd_t);
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data_array(io_main, cmd_tx_analog, cmds_size), TAG, "send command failed");

    /* Cbus Analog */
    cmds_size = sizeof(cmd_cbus_analog) / sizeof(lt8912b_lcd_init_cmd_t);
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data_array(io_main, cmd_cbus_analog, cmds_size), TAG, "send command failed");

    /* HDMI Pll Analog */
    cmds_size = sizeof(cmd_hdmi_pll_analog) / sizeof(lt8912b_lcd_init_cmd_t);
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data_array(io_main, cmd_hdmi_pll_analog, cmds_size), TAG, "send command failed");

    /* Mipi Analog - Set P/N swap */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_mipi_analog(io_main, false), TAG, "send command failed");

    /* Mipi Basic Set - lines count, lines swapped */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_mipi_basic_set(io_cec_dsi, 2, false), TAG, "send command failed");

    /* DDS Config */
    cmds_size = sizeof(cmd_dds_config) / sizeof(lt8912b_lcd_init_cmd_t);
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data_array(io_cec_dsi, cmd_dds_config, cmds_size), TAG, "send command failed");

    /* MIPI Video Setup */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_video_setup(panel), TAG, "send command failed");

    /* MIPI Input detection */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_detect_input_mipi(panel), TAG, "send command failed");

    /* MIPI Video Setup */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_video_setup(panel), TAG, "send command failed");

    /* AVI Infoframe */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_avi_infoframe(panel), TAG, "send command failed");

    /* MIPI RX Logic Reset */
    ESP_RETURN_ON_ERROR(_panel_lt8912b_mipi_rx_logic_reset(io_main), TAG, "send command failed");

    /* Audio IIs Mode */
    cmds_size = sizeof(cmd_audio_iis_mode) / sizeof(lt8912b_lcd_init_cmd_t);
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data_array(io_main, cmd_audio_iis_mode, cmds_size), TAG, "send command failed");

    /* Audio IIs En */
    cmds_size = sizeof(cmd_audio_iis_en) / sizeof(lt8912b_lcd_init_cmd_t);
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data_array(io_avi, cmd_audio_iis_en, cmds_size), TAG, "send command failed");

    /* Lvds Bypass */
    cmds_size = sizeof(cmd_lvds) / sizeof(lt8912b_lcd_init_cmd_t);
    ESP_RETURN_ON_ERROR(_panel_lt8912b_send_data_array(io_main, cmd_lvds, cmds_size), TAG, "send command failed");

    /* LVDS Enable */
    cmds_size = sizeof(cmd_lvds) / sizeof(lt8912b_lcd_init_cmd_t);
    ESP_RETURN_ON_ERROR(_panel_lt8912b_lvds_output(io_main, false), TAG, "send command failed");

    /* HDMI Enable */
    cmds_size = sizeof(cmd_lvds) / sizeof(lt8912b_lcd_init_cmd_t);
    ESP_RETURN_ON_ERROR(_panel_lt8912b_hdmi_output(io_main, true), TAG, "send command failed");

#if ENABLE_TEST_PATTERN
    /* Show test pattern - debug */
    _panel_lt8912b_send_test(panel);
#endif //ENABLE_TEST_PATTERN

    ESP_RETURN_ON_ERROR(lt8912b->init(panel), TAG, "init MIPI DPI panel failed");

    return ESP_OK;
}

static esp_err_t panel_lt8912b_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_lt8912b_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    ESP_LOGW(TAG, "Mirror is not supported in LT8912B driver. Please use SW rotation.");
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_lt8912b_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_lt8912b_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    esp_err_t ret = ESP_OK;
    lt8912b_panel_t *lt8912b = (lt8912b_panel_t *)panel->user_data;
    esp_lcd_panel_io_handle_t io_main = lt8912b->io.main;

    if (sleep) {
        ret |= _panel_lt8912b_send_data(io_main, 0x54, 0x1d);
        ret |= _panel_lt8912b_send_data(io_main, 0x51, 0x15);
        ret |= _panel_lt8912b_send_data(io_main, 0x44, 0x31);
        ret |= _panel_lt8912b_send_data(io_main, 0x41, 0xbd);
        ret |= _panel_lt8912b_send_data(io_main, 0x5c, 0x11);
    } else {
        ret |= _panel_lt8912b_send_data(io_main, 0x5c, 0x10);
        ret |= _panel_lt8912b_send_data(io_main, 0x54, 0x1c);
        ret |= _panel_lt8912b_send_data(io_main, 0x51, 0x2d);
        ret |= _panel_lt8912b_send_data(io_main, 0x44, 0x30);
        ret |= _panel_lt8912b_send_data(io_main, 0x41, 0xbc);

        vTaskDelay(pdMS_TO_TICKS(10));
        ret |= _panel_lt8912b_send_data(io_main, 0x03, 0x7f);
        vTaskDelay(pdMS_TO_TICKS(10));
        ret |= _panel_lt8912b_send_data(io_main, 0x03, 0xff);

        vTaskDelay(pdMS_TO_TICKS(10));
        ret |= _panel_lt8912b_send_data(io_main, 0x05, 0xfb);
        vTaskDelay(pdMS_TO_TICKS(10));
        ret |= _panel_lt8912b_send_data(io_main, 0x05, 0xff);
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    return ret;
}
