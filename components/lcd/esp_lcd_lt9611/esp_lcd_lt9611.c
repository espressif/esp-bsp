/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_lcd_lt9611.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_interface.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LT9611_PAGE_CONTROL        (0xFF)
#define LT9611_4LANES              (0x00)
#define LT9611_2LANES              (0x02)
#define LT9611_DEFAULT_LANES       (2)
#define LT9611_I2C_ADDRESS         (0x3B)
#define LT9611_I2C_CLK_HZ          (400000)
#define LT9611_I2C_TIMEOUT_MS      (1000)
#define LT9611_I2C_BURST_MAX       (16)

static const char *TAG = "lt9611";

typedef struct {
    i2c_master_dev_handle_t i2c_dev;
    esp_lcd_video_timing_t video_timing;
    uint32_t pclk_khz;
    int reset_gpio_num;
    bool reset_level;
    uint8_t lane_num;
    bool hsync_active_low;
    bool vsync_active_low;
    bool dpi_started;
    uint8_t current_page;
    bool page_valid;
    bool power_on;
    esp_err_t (*del)(esp_lcd_panel_t *panel);
    esp_err_t (*init)(esp_lcd_panel_t *panel);
} lt9611_panel_t;

static esp_err_t panel_lt9611_del(esp_lcd_panel_t *panel);
static esp_err_t panel_lt9611_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_lt9611_init(esp_lcd_panel_t *panel);
static esp_err_t panel_lt9611_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_lt9611_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_lt9611_disp_on_off(esp_lcd_panel_t *panel, bool on_off);
static esp_err_t panel_lt9611_sleep(esp_lcd_panel_t *panel, bool sleep);
static esp_err_t lt9611_configure_video_path(lt9611_panel_t *lt9611);

static uint32_t panel_lt9611_cal_htotal(const esp_lcd_video_timing_t *t)
{
    return t->h_size + t->hsync_pulse_width + t->hsync_back_porch + t->hsync_front_porch;
}

static uint32_t panel_lt9611_cal_vtotal(const esp_lcd_video_timing_t *t)
{
    return t->v_size + t->vsync_pulse_width + t->vsync_back_porch + t->vsync_front_porch;
}

static esp_err_t lt9611_i2c_write(lt9611_panel_t *lt9611, const uint8_t *data, size_t len)
{
    return i2c_master_transmit(lt9611->i2c_dev, data, len, LT9611_I2C_TIMEOUT_MS);
}

static esp_err_t lt9611_select_page(lt9611_panel_t *lt9611, uint8_t page)
{
    if (lt9611->page_valid && lt9611->current_page == page) {
        return ESP_OK;
    }
    const uint8_t buf[] = { LT9611_PAGE_CONTROL, page };
    ESP_RETURN_ON_ERROR(lt9611_i2c_write(lt9611, buf, sizeof(buf)), TAG, "select page 0x%02x failed", page);
    lt9611->current_page = page;
    lt9611->page_valid = true;
    return ESP_OK;
}

static esp_err_t lt9611_write_reg(lt9611_panel_t *lt9611, uint16_t reg, uint8_t val)
{
    ESP_RETURN_ON_ERROR(lt9611_select_page(lt9611, (uint8_t)(reg >> 8)), TAG, "select page failed");
    const uint8_t buf[] = { (uint8_t)(reg & 0xFF), val };
    ESP_RETURN_ON_ERROR(lt9611_i2c_write(lt9611, buf, sizeof(buf)), TAG, "write register 0x%04x failed", reg);
    return ESP_OK;
}

static esp_err_t lt9611_read_reg(lt9611_panel_t *lt9611, uint16_t reg, uint8_t *val)
{
    ESP_RETURN_ON_ERROR(lt9611_select_page(lt9611, (uint8_t)(reg >> 8)), TAG, "select page failed");
    const uint8_t addr = (uint8_t)(reg & 0xFF);
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(lt9611->i2c_dev, &addr, 1, val, 1, LT9611_I2C_TIMEOUT_MS),
                        TAG, "read register 0x%04x failed", reg);
    return ESP_OK;
}

static esp_err_t lt9611_write_seq(lt9611_panel_t *lt9611, const uint16_t *regs,
                                  const uint8_t *vals, size_t count)
{
    // Consecutive registers in the same page can share one I2C transaction
    // (register address + payload, auto-increment within the page).
    size_t i = 0;
    while (i < count) {
        const uint8_t page = (uint8_t)(regs[i] >> 8);
        const uint8_t start = (uint8_t)(regs[i] & 0xFF);
        size_t n = 1;
        while ((i + n) < count && n < LT9611_I2C_BURST_MAX &&
                ((uint8_t)(regs[i + n] >> 8) == page) &&
                ((uint8_t)(regs[i + n] & 0xFF) == (uint8_t)(start + n))) {
            n++;
        }

        ESP_RETURN_ON_ERROR(lt9611_select_page(lt9611, page), TAG, "select page failed");
        uint8_t buf[1 + LT9611_I2C_BURST_MAX];
        buf[0] = start;
        memcpy(&buf[1], &vals[i], n);
        ESP_RETURN_ON_ERROR(lt9611_i2c_write(lt9611, buf, n + 1), TAG, "write sequence failed");
        i += n;
    }
    return ESP_OK;
}

static bool lt9611_get_hpd(lt9611_panel_t *lt9611)
{
    uint8_t reg_val = 0;
    if (lt9611_read_reg(lt9611, 0x825E, &reg_val) != ESP_OK) {
        return false;
    }
    return (reg_val & 0x05) != 0;
}

static esp_err_t lt9611_power_on(lt9611_panel_t *lt9611)
{
    if (lt9611->power_on) {
        return ESP_OK;
    }

    static const uint16_t regs[] = {
        0x8101,                         // Select crystal clock
        0x821B, 0x821C,                 // Frequency meter timer 2
        0x82CB, 0x82CC,                 // Frequency meter timer 1
        0x8251,                         // Initialize interrupts
        0x8258, 0x8259,                 // HPD interrupt and debounce
        0x829E,                         // Video-check interrupt
        0x8004, 0x8006, 0x800A, 0x800B,
        0x800D, 0x8011,                 // Enable normal operating power
    };
    static const uint8_t vals[] = {
        0x18,
        0x69, 0x78,
        0x69, 0x78,
        0x01, 0x0A, 0x80,
        0xF7,
        0xF0, 0xF0, 0x80, 0x40, 0xEF, 0xFA,
    };
    ESP_RETURN_ON_ERROR(lt9611_write_seq(lt9611, regs, vals, sizeof(vals)), TAG, "power on failed");
    lt9611->power_on = true;
    return ESP_OK;
}

static esp_err_t lt9611_mipi_input_analog(lt9611_panel_t *lt9611)
{
    static const uint16_t regs[] = {
        0x8106, 0x810A, 0x810B,         // Port A RX current, LDO, and LP receiver
        0x8111, 0x8115, 0x8116,         // Port B RX current, LDO, and LP receiver
        0x811C,                         // Port A clock lane no-LP mode
        0x8120,                         // Port B clock lane with-LP mode
    };
    static const uint8_t vals[] = {
        0x40, 0xFE, 0xBF,
        0x40, 0xFE, 0xBF,
        0x03, 0x03,
    };
    return lt9611_write_seq(lt9611, regs, vals, sizeof(vals));
}

static esp_err_t lt9611_mipi_input_digital(lt9611_panel_t *lt9611)
{
    uint8_t lane_cfg = LT9611_2LANES;
    switch (lt9611->lane_num) {
    case 2:
        lane_cfg = LT9611_2LANES;
        break;
    case 4:
        lane_cfg = LT9611_4LANES;
        break;
    default:
        ESP_LOGE(TAG, "unsupported lane number %u", lt9611->lane_num);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8300, lane_cfg), TAG, "set lane failed"); // Lane count
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x830A, 0x00), TAG, "set port config failed"); // Port A only
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x824F, 0x80), TAG, "write 0x824f failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8250, 0x10), TAG, "write 0x8250 failed"); // Port A byte clock
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8303, 0x00), TAG, "write 0x8303 failed"); // Disable port swap
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8302, 0x0A), TAG, "write 0x8302 failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8306, 0x0A), TAG, "write 0x8306 failed");
    return ESP_OK;
}

static esp_err_t lt9611_mipi_video_setup(lt9611_panel_t *lt9611)
{
    const esp_lcd_video_timing_t *t = &lt9611->video_timing;
    uint32_t htotal = panel_lt9611_cal_htotal(t);
    uint32_t vtotal = panel_lt9611_cal_vtotal(t);
    uint32_t hsync_porch = t->hsync_pulse_width + t->hsync_back_porch;
    uint32_t vsync_porch = t->vsync_pulse_width + t->vsync_back_porch;

    // Program the MIPI input geometry used by the PCR and HDMI timing path.
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x830D, (uint8_t)(vtotal / 256)), TAG, "vtotal high failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x830E, (uint8_t)(vtotal % 256)), TAG, "vtotal low failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x830F, (uint8_t)(t->v_size / 256)), TAG, "vact high failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8310, (uint8_t)(t->v_size % 256)), TAG, "vact low failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8311, (uint8_t)(htotal / 256)), TAG, "htotal high failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8312, (uint8_t)(htotal % 256)), TAG, "htotal low failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8313, (uint8_t)(t->h_size / 256)), TAG, "hact high failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8314, (uint8_t)(t->h_size % 256)), TAG, "hact low failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8315, (uint8_t)(t->vsync_pulse_width % 256)), TAG, "vsync failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8316, (uint8_t)(t->hsync_pulse_width % 256)), TAG, "hsync failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8317, (uint8_t)(t->vsync_front_porch % 256)), TAG, "vfp failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8318, (uint8_t)(vsync_porch % 256)), TAG, "vsync porch failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8319, (uint8_t)(t->hsync_front_porch % 256)), TAG, "hfp failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x831A,
                                         (uint8_t)((hsync_porch / 256) | ((t->hsync_front_porch / 256) << 4))),
                        TAG, "hsync porch high failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x831B, (uint8_t)(hsync_porch % 256)), TAG,
                        "hsync porch low failed");
    return ESP_OK;
}

static esp_err_t lt9611_pll_setup(lt9611_panel_t *lt9611, unsigned int *postdiv)
{
    const uint32_t pclk = lt9611->pclk_khz;
    static const uint16_t regs[] = {
        // TX PLL initialization
        0x8123, 0x8124, 0x8125, 0x8126, 0x812C, 0x812F, 0x8126, 0x8127, 0x8128,
        0x812A, // Improve PLL lock reliability
    };
    static const uint8_t vals[] = {
        0x40, 0x64, 0x80, 0x55, 0x37, 0x01, 0x55, 0x66, 0x88, 0x20,
    };

    ESP_RETURN_ON_ERROR(lt9611_write_seq(lt9611, regs, vals, sizeof(vals)), TAG, "PLL init failed");

    if (pclk > 150000) {
        ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x812D, 0x88), TAG, "post-divider failed");
        *postdiv = 1;
    } else if (pclk > 70000) {
        ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x812D, 0x99), TAG, "post-divider failed");
        *postdiv = 2;
    } else {
        ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x812D, 0xAA), TAG, "post-divider failed");
        *postdiv = 4;
    }

    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x82E3, (uint8_t)(pclk >> 17)), TAG, "pclk[19:16] failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x82E4, (uint8_t)(pclk >> 9)), TAG, "pclk[15:8] failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x82E5, (uint8_t)(pclk >> 1)), TAG, "pclk[7:0] failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x82DE, 0x20), TAG, "PLL reset 1 failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x82DE, 0xE0), TAG, "PLL reset 2 failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8016, 0xF1), TAG, "PLL reset 3 failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8016, 0xF3), TAG, "PLL reset 4 failed");
    return ESP_OK;
}

static esp_err_t lt9611_pcr_setup(lt9611_panel_t *lt9611, unsigned int postdiv)
{
    unsigned int pcr_m = (unsigned int)(lt9611->pclk_khz * 5U * postdiv / 27000U);
    uint8_t pol = 0x10;

    if (lt9611->hsync_active_low) {
        pol |= 0x02;
    }
    if (lt9611->vsync_active_low) {
        pol |= 0x01;
    }

    static const uint16_t regs[] = {
        0x830B, 0x830C, 0x8348, 0x8349,
        0x8321, 0x8324, 0x8325, 0x832A, // PCR stage 1
        0x834A,                         // PCR stage 2
        0x832D, 0x8331,                 // PCR MK limits
    };
    static const uint8_t vals[] = {
        0x01, 0x10, 0x00, 0x81,
        0x4A, 0x71, 0x30, 0x01,
        0x40,
        0x38, 0x08,
    };
    ESP_RETURN_ON_ERROR(lt9611_write_seq(lt9611, regs, vals, sizeof(vals)), TAG, "PCR base failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x831D, pol), TAG, "sync polarity failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8326, (uint8_t)pcr_m), TAG, "PCR M failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8011, 0x5A), TAG, "PCR reset 1 failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x8011, 0xFA), TAG, "PCR reset 2 failed");
    return ESP_OK;
}

static esp_err_t lt9611_hdmi_tx_digital(lt9611_panel_t *lt9611)
{
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x82D6, 0x8C), TAG, "HDMI digital failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x82D7, 0x04), TAG, "HDMI digital 2 failed");
    return ESP_OK;
}

static esp_err_t lt9611_hdmi_avi_infoframe(lt9611_panel_t *lt9611)
{
    const esp_lcd_video_timing_t *t = &lt9611->video_timing;
    uint8_t vic = 0;
    uint8_t picture_aspect = 0;
    if (t->h_size == 1920 && t->v_size == 1080) {
        picture_aspect = 2;
        if (lt9611->pclk_khz == 148500 && panel_lt9611_cal_htotal(t) == 2200) {
            vic = 16;
        }
    } else if (t->h_size == 1280 && t->v_size == 720) {
        vic = 4;
        picture_aspect = 2;
    } else if (t->h_size == 640 && t->v_size == 480) {
        vic = 1;
        picture_aspect = 1;
    }

    // HDMI AVI InfoFrame: header, checksum, and 13 payload bytes.
    uint8_t frame[17] = {
        0x82, 0x02, 0x0D, 0x00,
        0x00, (uint8_t)(picture_aspect << 4), 0x00, vic,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00,
    };
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(frame); i++) {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    frame[3] = (uint8_t)(0U - checksum);

    for (size_t i = 0; i < sizeof(frame); i++) {
        ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, (uint16_t)(0x8440 + i), frame[i]),
                            TAG, "write AVI InfoFrame failed");
    }

    uint8_t infoframe_enable = 0;
    ESP_RETURN_ON_ERROR(lt9611_read_reg(lt9611, 0x843D, &infoframe_enable),
                        TAG, "read InfoFrame enable failed");
    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x843D, infoframe_enable | 0x08),
                        TAG, "enable AVI InfoFrame failed");
    return ESP_OK;
}

static esp_err_t lt9611_hdmi_tx_phy(lt9611_panel_t *lt9611)
{
    static const uint16_t regs[] = {
        0x8130,
        0x8131, // HDMI DC mode
        0x8132, 0x8133, 0x8134, 0x8135, 0x8136, 0x8137,
        0x813F, 0x8140, 0x8141, 0x8142, 0x8143, 0x8144,
    };
    static const uint8_t vals[] = {
        0x6A, 0x44, 0x4A, 0x0B, 0x00, 0x00, 0x00, 0x44,
        0x0F, 0xA0, 0xA0, 0xA0, 0xA0, 0x0A,
    };
    return lt9611_write_seq(lt9611, regs, vals, sizeof(vals));
}

static esp_err_t lt9611_configure_video_path(lt9611_panel_t *lt9611)
{
    unsigned int postdiv = 0;

    ESP_RETURN_ON_ERROR(lt9611_mipi_input_digital(lt9611), TAG, "MIPI digital failed");
    ESP_RETURN_ON_ERROR(lt9611_pll_setup(lt9611, &postdiv), TAG, "PLL setup failed");
    ESP_RETURN_ON_ERROR(lt9611_mipi_video_setup(lt9611), TAG, "video setup failed");
    ESP_RETURN_ON_ERROR(lt9611_pcr_setup(lt9611, postdiv), TAG, "PCR setup failed");
    ESP_RETURN_ON_ERROR(lt9611_power_on(lt9611), TAG, "power on failed");
    ESP_RETURN_ON_ERROR(lt9611_mipi_input_analog(lt9611), TAG, "MIPI analog failed");
    ESP_RETURN_ON_ERROR(lt9611_hdmi_avi_infoframe(lt9611), TAG, "AVI InfoFrame failed");
    ESP_RETURN_ON_ERROR(lt9611_hdmi_tx_digital(lt9611), TAG, "HDMI digital failed");
    ESP_RETURN_ON_ERROR(lt9611_hdmi_tx_phy(lt9611), TAG, "HDMI PHY failed");

    vTaskDelay(pdMS_TO_TICKS(500));
    return lt9611_write_reg(lt9611, 0x8130, 0xEA);
}

esp_err_t esp_lcd_new_panel_lt9611(i2c_master_bus_handle_t i2c_bus,
                                   const esp_lcd_panel_dev_config_t *panel_dev_config,
                                   esp_lcd_panel_handle_t *ret_panel)
{
    ESP_RETURN_ON_FALSE(i2c_bus && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    lt9611_vendor_config_t *vendor_config = (lt9611_vendor_config_t *)panel_dev_config->vendor_config;
    ESP_RETURN_ON_FALSE(vendor_config && vendor_config->mipi_config.dpi_config &&
                        vendor_config->mipi_config.dsi_bus, ESP_ERR_INVALID_ARG, TAG, "invalid vendor config");

    esp_err_t ret = ESP_OK;
    lt9611_panel_t *lt9611 = (lt9611_panel_t *)calloc(1, sizeof(lt9611_panel_t));
    ESP_RETURN_ON_FALSE(lt9611, ESP_ERR_NO_MEM, TAG, "no memory for LT9611 panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure reset GPIO failed");
    }

    const i2c_device_config_t i2c_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LT9611_I2C_ADDRESS,
        .scl_speed_hz = LT9611_I2C_CLK_HZ,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_config, &lt9611->i2c_dev),
                      err, TAG, "add I2C device failed");
    lt9611->video_timing = vendor_config->mipi_config.dpi_config->video_timing;
    lt9611->pclk_khz = (uint32_t)(vendor_config->mipi_config.dpi_config->dpi_clock_freq_mhz * 1000.0f + 0.5f);
    lt9611->lane_num = vendor_config->mipi_config.lane_num ?
                       vendor_config->mipi_config.lane_num : LT9611_DEFAULT_LANES;
    lt9611->hsync_active_low = vendor_config->flags.hsync_active_low;
    lt9611->vsync_active_low = vendor_config->flags.vsync_active_low;
    lt9611->reset_gpio_num = panel_dev_config->reset_gpio_num;
    lt9611->reset_level = panel_dev_config->flags.reset_active_high;

    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_dpi(vendor_config->mipi_config.dsi_bus,
                                            vendor_config->mipi_config.dpi_config, ret_panel),
                      err, TAG, "create MIPI DPI panel failed");

    lt9611->del = (*ret_panel)->del;
    lt9611->init = (*ret_panel)->init;
    (*ret_panel)->del = panel_lt9611_del;
    (*ret_panel)->init = panel_lt9611_init;
    (*ret_panel)->reset = panel_lt9611_reset;
    (*ret_panel)->mirror = panel_lt9611_mirror;
    (*ret_panel)->invert_color = panel_lt9611_invert_color;
    (*ret_panel)->disp_on_off = panel_lt9611_disp_on_off;
    (*ret_panel)->disp_sleep = panel_lt9611_sleep;
    (*ret_panel)->user_data = lt9611;
    return ESP_OK;

err:
    if (lt9611) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        if (lt9611->i2c_dev) {
            i2c_master_bus_rm_device(lt9611->i2c_dev);
        }
        free(lt9611);
    }
    return ret;
}

bool esp_lcd_panel_lt9611_is_connected(esp_lcd_panel_t *panel)
{
    ESP_RETURN_ON_FALSE(panel && panel->user_data, false, TAG, "invalid panel");
    lt9611_panel_t *lt9611 = (lt9611_panel_t *)panel->user_data;
    return lt9611_get_hpd(lt9611);
}

static esp_err_t panel_lt9611_del(esp_lcd_panel_t *panel)
{
    lt9611_panel_t *lt9611 = (lt9611_panel_t *)panel->user_data;
    if (lt9611->reset_gpio_num >= 0) {
        gpio_reset_pin(lt9611->reset_gpio_num);
    }
    if (lt9611->i2c_dev) {
        i2c_master_bus_rm_device(lt9611->i2c_dev);
    }
    lt9611->del(panel);
    free(lt9611);
    return ESP_OK;
}

static esp_err_t panel_lt9611_reset(esp_lcd_panel_t *panel)
{
    lt9611_panel_t *lt9611 = (lt9611_panel_t *)panel->user_data;

    if (lt9611->reset_gpio_num >= 0) {
        gpio_set_level(lt9611->reset_gpio_num, lt9611->reset_level);
        vTaskDelay(pdMS_TO_TICKS(20));
        gpio_set_level(lt9611->reset_gpio_num, !lt9611->reset_level);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    lt9611->page_valid = false;
    lt9611->power_on = false;
    lt9611->dpi_started = false;
    return ESP_OK;
}

static esp_err_t panel_lt9611_init(esp_lcd_panel_t *panel)
{
    lt9611_panel_t *lt9611 = (lt9611_panel_t *)panel->user_data;

    ESP_RETURN_ON_ERROR(lt9611_write_reg(lt9611, 0x80EE, 0x01), TAG, "enable register access failed");

    if (!lt9611->dpi_started) {
        // Start the DPI source before enabling the LT9611 HDMI output.
        ESP_RETURN_ON_ERROR(lt9611->init(panel), TAG, "initialize MIPI DPI panel failed");
        lt9611->dpi_started = true;
    }
    ESP_RETURN_ON_ERROR(lt9611_configure_video_path(lt9611), TAG, "configure video path failed");

    ESP_LOGI(TAG, "LT9611 initialization done (HPD=%s)", lt9611_get_hpd(lt9611) ? "yes" : "no");
    return ESP_OK;
}

static esp_err_t panel_lt9611_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    (void)panel;
    (void)invert_color_data;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_lt9611_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    (void)panel;
    (void)mirror_x;
    (void)mirror_y;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t panel_lt9611_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    lt9611_panel_t *lt9611 = (lt9611_panel_t *)panel->user_data;
    return lt9611_write_reg(lt9611, 0x8130, on_off ? 0xEA : 0x6A);
}

static esp_err_t panel_lt9611_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    lt9611_panel_t *lt9611 = (lt9611_panel_t *)panel->user_data;

    if (sleep) {
        static const uint16_t regs[] = {
            0x8102, // Power down MIPI receiver
            0x8123, 0x8130, 0x8011,
        };
        static const uint8_t vals[] = {
            0x48, 0x80, 0x00, 0x0A,
        };
        ESP_RETURN_ON_ERROR(lt9611_write_seq(lt9611, regs, vals, sizeof(vals)), TAG, "enter sleep failed");
        lt9611->power_on = false;
    } else {
        static const uint16_t regs[] = {
            0x8102, 0x8123, 0x8130, 0x8011,
        };
        static const uint8_t vals[] = {
            0x12, 0x40, 0xEA, 0xFA,
        };
        ESP_RETURN_ON_ERROR(lt9611_write_seq(lt9611, regs, vals, sizeof(vals)), TAG, "wake analog failed");
        ESP_RETURN_ON_ERROR(panel_lt9611_init(panel), TAG, "wake from sleep failed");
    }
    return ESP_OK;
}
