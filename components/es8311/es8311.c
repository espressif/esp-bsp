/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "es8311.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "es8311_reg.h"

/*!< MCLK_DIV_FRE is the frequency division coefficient of LRCLK */
#define MCLK_DIV_FRE        32u // this is valid for 16bit data resolution and MCLK from SCLK pin

typedef struct {
    i2c_port_t port;
    uint16_t dev_addr;
} es8311_dev_t;

/*
 * Clock coefficient structer
 */
struct _coeff_div {
    uint32_t mclk;        /* mclk frequency */
    uint32_t rate;        /* sample rate */
    uint8_t pre_div;      /* the pre divider with range from 1 to 8 */
    uint8_t pre_multi;    /* the pre multiplier with 0: 1x, 1: 2x, 2: 4x, 3: 8x selection */
    uint8_t adc_div;      /* adcclk divider */
    uint8_t dac_div;      /* dacclk divider */
    uint8_t fs_mode;      /* double speed or single speed, =0, ss, =1, ds */
    uint8_t lrck_h;       /* adclrck divider and daclrck divider */
    uint8_t lrck_l;
    uint8_t bclk_div;     /* sclk divider */
    uint8_t adc_osr;      /* adc osr */
    uint8_t dac_osr;      /* dac osr */
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
    /*!<mclk     rate   pre_div  mult  adc_div dac_div fs_mode lrch  lrcl  bckdiv osr */
    /* 8k */
    {12288000, 8000, 0x06, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 8000, 0x03, 0x01, 0x03, 0x03, 0x00, 0x05, 0xff, 0x18, 0x10, 0x10},
    {16384000, 8000, 0x08, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {8192000, 8000, 0x04, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000, 8000, 0x03, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {4096000, 8000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000, 8000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {2048000, 8000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000, 8000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1024000, 8000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 11.025k */
    {11289600, 11025, 0x04, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {5644800, 11025, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {2822400, 11025, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1411200, 11025, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 12k */
    {12288000, 12000, 0x04, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000, 12000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000, 12000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000, 12000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 16k */
    {12288000, 16000, 0x03, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 16000, 0x03, 0x01, 0x03, 0x03, 0x00, 0x02, 0xff, 0x0c, 0x10, 0x10},
    {16384000, 16000, 0x04, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {8192000, 16000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000, 16000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {4096000, 16000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000, 16000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {2048000, 16000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000, 16000, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1024000, 16000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 22.05k */
    {11289600, 22050, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {5644800, 22050, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {2822400, 22050, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1411200, 22050, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {705600, 22050, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 24k */
    {12288000, 24000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 24000, 0x03, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000, 24000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000, 24000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000, 24000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 32k */
    {12288000, 32000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 32000, 0x03, 0x02, 0x03, 0x03, 0x00, 0x02, 0xff, 0x0c, 0x10, 0x10},
    {16384000, 32000, 0x02, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {8192000, 32000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000, 32000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {4096000, 32000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000, 32000, 0x03, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {2048000, 32000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000, 32000, 0x03, 0x03, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10},
    {1024000, 32000, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 44.1k */
    {11289600, 44100, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {5644800, 44100, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {2822400, 44100, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1411200, 44100, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 48k */
    {12288000, 48000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 48000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000, 48000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000, 48000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000, 48000, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},

    /* 64k */
    {12288000, 64000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 64000, 0x03, 0x02, 0x03, 0x03, 0x01, 0x01, 0x7f, 0x06, 0x10, 0x10},
    {16384000, 64000, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {8192000, 64000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000, 64000, 0x01, 0x02, 0x03, 0x03, 0x01, 0x01, 0x7f, 0x06, 0x10, 0x10},
    {4096000, 64000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000, 64000, 0x01, 0x03, 0x03, 0x03, 0x01, 0x01, 0x7f, 0x06, 0x10, 0x10},
    {2048000, 64000, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000, 64000, 0x01, 0x03, 0x01, 0x01, 0x01, 0x00, 0xbf, 0x03, 0x18, 0x18},
    {1024000, 64000, 0x01, 0x03, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10},

    /* 88.2k */
    {11289600, 88200, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {5644800, 88200, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {2822400, 88200, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1411200, 88200, 0x01, 0x03, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10},

    /* 96k */
    {12288000, 96000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {18432000, 96000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {6144000, 96000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {3072000, 96000, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10},
    {1536000, 96000, 0x01, 0x03, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10},
};

static char *TAG = "ES8311";

static int es8311_write_reg(es8311_handle_t dev, uint8_t reg_addr, uint8_t data)
{
    es8311_dev_t *es = (es8311_dev_t *) dev;
    esp_err_t  ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, es->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, data, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(es->port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

int es8311_read_reg(es8311_handle_t dev, uint8_t reg_addr)
{
    es8311_dev_t *es = (es8311_dev_t *) dev;
    uint8_t data;
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, es->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, reg_addr, true);
    assert(ESP_OK == ret);
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, es->dev_addr | I2C_MASTER_READ, true);
    assert(ESP_OK == ret);
    ret = i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(es->port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return data;
}

/*
* look for the coefficient in coeff_div[] table
*/
static int get_coeff(uint32_t mclk, uint32_t rate)
{
    for (int i = 0; i < (sizeof(coeff_div) / sizeof(coeff_div[0])); i++) {
        if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk) {
            return i;
        }
    }

    return -1;
}

esp_err_t es8311_sample_frequency_config(es8311_handle_t dev, int mclk_frequency, int sample_frequency)
{
    esp_err_t ret = ESP_OK;
    uint8_t regv;

    /* Get clock coefficients from coefficient table */
    int coeff = get_coeff(mclk_frequency, sample_frequency);

    if (coeff < 0) {
        ESP_LOGE(TAG, "Unable to configure sample rate %dHz with %dHz MCLK", sample_frequency, mclk_frequency);
        return ESP_ERR_INVALID_ARG;
    }

    const struct _coeff_div *const selected_coeff = &coeff_div[coeff];

    /* register 0x02 */
    regv = es8311_read_reg(dev, ES8311_CLK_MANAGER_REG02) & 0x07;
    regv |= (selected_coeff->pre_div - 1) << 5;
    regv |= selected_coeff->pre_multi << 3;
    ret |= es8311_write_reg(dev, ES8311_CLK_MANAGER_REG02, regv);

    /* register 0x03 */
    const uint8_t reg03 = (selected_coeff->fs_mode << 6) | selected_coeff->adc_osr;
    ret |= es8311_write_reg(dev, ES8311_CLK_MANAGER_REG03, reg03);

    /* register 0x04 */
    ret |= es8311_write_reg(dev, ES8311_CLK_MANAGER_REG04, selected_coeff->dac_osr);

    /* register 0x05 */
    const uint8_t reg05 = ((selected_coeff->adc_div - 1) << 4) | (selected_coeff->dac_div - 1);
    ret |= es8311_write_reg(dev, ES8311_CLK_MANAGER_REG05, reg05);

    /* register 0x06 */
    regv = es8311_read_reg(dev, ES8311_CLK_MANAGER_REG06) & 0xE0;

    if (selected_coeff->bclk_div < 19) {
        regv |= (selected_coeff->bclk_div - 1) << 0;
    } else {
        regv |= (selected_coeff->bclk_div) << 0;
    }

    ret |= es8311_write_reg(dev, ES8311_CLK_MANAGER_REG06, regv);

    /* register 0x07 */
    regv = es8311_read_reg(dev, ES8311_CLK_MANAGER_REG07) & 0xC0;
    regv |= selected_coeff->lrck_h << 0;
    ret |= es8311_write_reg(dev, ES8311_CLK_MANAGER_REG07, regv);

    /* register 0x08 */
    ret |= es8311_write_reg(dev, ES8311_CLK_MANAGER_REG08, selected_coeff->lrck_l);

    return ret;
}

static esp_err_t es8311_clock_config(es8311_handle_t dev, const es8311_clock_config_t *const clk_cfg)
{
    esp_err_t ret = ESP_OK;
    uint8_t reg06;
    uint8_t reg01 = 0x3F; // Enable all clocks

    /* Select clock source for internal MCLK */
    if (!clk_cfg->mclk_from_mclk_pin) {
        reg01 |= BIT(7); // Select BCLK pin
    }

    if (clk_cfg->mclk_inverted) {
        reg01 |= BIT(6); // Invert MCLK pin
    }
    ret |= es8311_write_reg(dev, ES8311_CLK_MANAGER_REG01, reg01);

    reg06 = es8311_read_reg(dev, ES8311_CLK_MANAGER_REG06);
    if (clk_cfg->sclk_inverted) {
        reg06 |= BIT(5);
    } else {
        reg06 &= ~BIT(5);
    }
    ret |= es8311_write_reg(dev, ES8311_CLK_MANAGER_REG06, reg06);

    /* Configure clock dividers */
    // 16 bit resolution and MCLK from SCLK pin is assumed here
    return es8311_sample_frequency_config(dev, clk_cfg->sample_frequency * MCLK_DIV_FRE, clk_cfg->sample_frequency);
}

static esp_err_t es8311_resolution_config(const es8311_resolution_t res, uint8_t *reg)
{
    switch (res) {
    case ES8311_RESOLUTION_16:
        *reg |= (3 << 2);
        break;
    case ES8311_RESOLUTION_18:
        *reg |= (2 << 2);
        break;
    case ES8311_RESOLUTION_20:
        *reg |= (1 << 2);
        break;
    case ES8311_RESOLUTION_24:
        *reg |= (0 << 2);
        break;
    case ES8311_RESOLUTION_32:
        *reg |= (4 << 2);
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

static esp_err_t es8311_fmt_config(es8311_handle_t dev, const es8311_resolution_t res_in, const es8311_resolution_t res_out)
{
    esp_err_t ret = ESP_OK;
    uint8_t reg09 = 0; // SDP In
    uint8_t reg0a = 0; // SDP Out

    ESP_LOGI(TAG, "ES8311 in Slave mode and I2S format");
    uint8_t reg00 = es8311_read_reg(dev, ES8311_RESET_REG00);
    reg00 &= 0xBF;
    ret |= es8311_write_reg(dev, ES8311_RESET_REG00, reg00); // Slave serial port - default

    /* Setup SDP In and Out resolution */
    es8311_resolution_config(res_in, &reg09);
    es8311_resolution_config(res_out, &reg0a);

    ret |= es8311_write_reg(dev, ES8311_SDPIN_REG09, reg09);
    ret |= es8311_write_reg(dev, ES8311_SDPOUT_REG0A, reg0a);

    return ret;
}

esp_err_t es8311_microphone_config(es8311_handle_t dev, bool digital_mic)
{
    uint8_t reg14 = 0x1A; // enable analog MIC and max PGA gain

    /* PDM digital microphone enable or disable */
    if (digital_mic) {
        reg14 |= BIT(6);
    }
    es8311_write_reg(dev, ES8311_ADC_REG17, 0xC8); // Set ADC gain @todo move this to ADC config section

    return es8311_write_reg(dev, ES8311_SYSTEM_REG14, reg14);
}

esp_err_t es8311_init(es8311_handle_t dev, const es8311_clock_config_t *const clk_cfg, const es8311_resolution_t res_in, const es8311_resolution_t res_out)
{
    esp_err_t ret = ESP_OK;
    if (clk_cfg->sample_frequency <= 8000) {
        ESP_LOGE(TAG, "ES8311 init needs frequency > 8000Hz, such as 32000Hz, 44100Hz");
        return ESP_ERR_INVALID_ARG;
    }

    /* Reset ES8311 to its default */
    ret |= es8311_write_reg(dev, ES8311_RESET_REG00, 0x1F);
    vTaskDelay(pdMS_TO_TICKS(20));
    ret |= es8311_write_reg(dev, ES8311_RESET_REG00, 0x00);
    ret |= es8311_write_reg(dev, ES8311_RESET_REG00, 0x80); // Power-on command

    /* Setup clock: source, polarity and clock dividers */
    ret |= es8311_clock_config(dev, clk_cfg);

    /* Setup audio format (fmt): master/slave, resolution, I2S */
    ret |= es8311_fmt_config(dev, res_in, res_out);

    ret |= es8311_write_reg(dev, ES8311_SYSTEM_REG0D, 0x01); // Power up analog circuitry - NOT default
    ret |= es8311_write_reg(dev, ES8311_SYSTEM_REG0E, 0x02); // Enable analog PGA, enable ADC modulator - NOT default
    ret |= es8311_write_reg(dev, ES8311_SYSTEM_REG12, 0x00); // power-up DAC - NOT default
    ret |= es8311_write_reg(dev, ES8311_SYSTEM_REG13, 0x10); // Enable output to HP drive - NOT default
    ret |= es8311_write_reg(dev, ES8311_ADC_REG1C, 0x6A); // ADC Equalizer bypass, cancel DC offset in digital domain
    ret |= es8311_write_reg(dev, ES8311_DAC_REG37, 0x48); // DAC ramprate and bypass DAC equalizer - NOT default

    return ret;
}

void es8311_delete(es8311_handle_t dev)
{
    free(dev);
}

esp_err_t es8311_voice_volume_set(es8311_handle_t dev, int volume, int *volume_set)
{
    if (volume < 0) {
        volume = 0;
    } else if (volume > 100) {
        volume = 100;
    }

    int reg32;
    if (volume == 0) {
        reg32 = 0;
    } else {
        reg32 = ((volume) * 256 / 100) - 1;
    }

    // provide user with real volume set
    if (volume_set != NULL) {
        *volume_set = volume;
    }
    return es8311_write_reg(dev, ES8311_DAC_REG32, reg32);
}

esp_err_t es8311_voice_volume_get(es8311_handle_t dev, int *volume)
{
    uint8_t reg32 = es8311_read_reg(dev, ES8311_DAC_REG32);

    if (reg32 == 0) {
        *volume = 0;
    } else {
        *volume = ((reg32 * 100) / 256) + 1;
    }
    return ESP_OK;
}

esp_err_t es8311_voice_mute(es8311_handle_t dev, bool mute)
{
    uint8_t reg31 = es8311_read_reg(dev, ES8311_DAC_REG31);

    if (mute) {
        reg31 |= BIT(6) | BIT(5);
    } else {
        reg31 &= ~(BIT(6) | BIT(5));
    }

    return es8311_write_reg(dev, ES8311_DAC_REG31, reg31);
}

esp_err_t es8311_microphone_gain_set(es8311_handle_t dev, es8311_mic_gain_t gain_db)
{
    return es8311_write_reg(dev, ES8311_ADC_REG16, gain_db); // ADC gain scale up
}

void es8311_register_dump(es8311_handle_t dev)
{
    for (int reg = 0; reg < 0x4A; reg++) {
        uint8_t value = es8311_read_reg(dev, reg);
        ESP_LOGI(TAG, "REG:%02x: %02x", reg, value);
    }
}

es8311_handle_t es8311_create(const i2c_port_t port, const uint16_t dev_addr)
{
    es8311_dev_t *sensor = (es8311_dev_t *) calloc(1, sizeof(es8311_dev_t));
    sensor->port = port;
    sensor->dev_addr = dev_addr << 1;
    return (es8311_handle_t) sensor;
}
