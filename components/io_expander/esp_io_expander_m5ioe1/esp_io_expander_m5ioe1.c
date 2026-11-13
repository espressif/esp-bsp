/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_io_expander.h"
#include "esp_io_expander_m5ioe1.h"

/* I2C communication related */
#define I2C_TIMEOUT_MS          (1000)
#define I2C_CLK_SPEED           (100000)
/* Firmware I2C slave needs time to handle a transaction before the next one. */
#define I2C_POST_DELAY_US       (500)

#define IO_COUNT                (14)
#define IO_MASK                 ((1U << IO_COUNT) - 1)

/* Register address. GPIO pairs are little-endian: low byte is IO1..IO8, high byte is IO9..IO14. */
#define REG_REV                 (0x02)
#define REG_GPIO_MODE_L         (0x03)
#define REG_GPIO_OUT_L          (0x05)
#define REG_GPIO_IN_L           (0x07)
#define REG_GPIO_PU_L           (0x09)
#define REG_GPIO_PD_L           (0x0B)
#define REG_GPIO_IE_L           (0x0D)
#define REG_GPIO_DRV_L          (0x13)

/* Direction bit 1 = output. Drive bit 1 = open-drain. */
#define DIR_REG_DEFAULT_VAL     (0x0000)
#define OUT_REG_DEFAULT_VAL     (0x0000)
#define HIGHZ_REG_DEFAULT_VAL   (0x0000)
#define PULL_EN_DEFAULT_VAL     (0x0000)
#define PULL_SEL_DEFAULT_VAL    (0x0000)

/**
 * @brief Device Structure Type
 *
 */
typedef struct {
    esp_io_expander_t base;
    i2c_master_dev_handle_t i2c_handle;
    struct {
        uint16_t direction;
        uint16_t output;
        uint16_t highz;
        uint16_t pull_en;
        uint16_t pull_sel;
    } regs;
} esp_io_expander_m5ioe1_t;

static char *TAG = "m5ioe1";

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_highz_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_highz_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_pullup_en_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_pullup_en_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_pullup_sel_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_pullup_sel_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t reset(esp_io_expander_t *handle);
static esp_err_t del(esp_io_expander_t *handle);

static esp_err_t write_reg16(esp_io_expander_m5ioe1_t *dev, uint8_t reg, uint16_t value)
{
    uint8_t data[] = {reg, value & 0xff, (value >> 8) & 0xff};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev->i2c_handle, data, sizeof(data), I2C_TIMEOUT_MS), TAG,
                        "Write reg 0x%02X failed", reg);
    esp_rom_delay_us(I2C_POST_DELAY_US);
    return ESP_OK;
}

static esp_err_t read_reg(esp_io_expander_m5ioe1_t *dev, uint8_t reg, uint8_t *value)
{
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(dev->i2c_handle, &reg, 1, value, 1, I2C_TIMEOUT_MS), TAG,
                        "Read reg 0x%02X failed", reg);
    esp_rom_delay_us(I2C_POST_DELAY_US);
    return ESP_OK;
}

static esp_err_t read_reg16(esp_io_expander_m5ioe1_t *dev, uint8_t reg, uint16_t *value)
{
    uint8_t temp[2] = {0, 0};
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(dev->i2c_handle, &reg, 1, temp, sizeof(temp), I2C_TIMEOUT_MS), TAG,
                        "Read reg 0x%02X failed", reg);
    *value = ((uint16_t)temp[1] << 8) | temp[0];
    esp_rom_delay_us(I2C_POST_DELAY_US);
    return ESP_OK;
}

static esp_err_t apply_pull_regs(esp_io_expander_m5ioe1_t *dev)
{
    /* pull_sel 1 = pull-up. pull_en gates the selected resistor. PU and PD are separate registers. */
    uint16_t en = dev->regs.pull_en & IO_MASK;
    uint16_t sel = dev->regs.pull_sel & IO_MASK;
    uint16_t pu = en & sel;
    uint16_t pd = en & (uint16_t)(~sel) & IO_MASK;

    ESP_RETURN_ON_ERROR(write_reg16(dev, REG_GPIO_PU_L, pu), TAG, "Write pull-up reg failed");
    ESP_RETURN_ON_ERROR(write_reg16(dev, REG_GPIO_PD_L, pd), TAG, "Write pull-down reg failed");
    return ESP_OK;
}

esp_err_t esp_io_expander_new_i2c_m5ioe1(i2c_master_bus_handle_t i2c_bus, uint32_t dev_addr,
        esp_io_expander_handle_t *handle_ret)
{
    ESP_RETURN_ON_FALSE(handle_ret != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle_ret");

    // Allocate memory for driver object
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)calloc(1, sizeof(esp_io_expander_m5ioe1_t));
    ESP_RETURN_ON_FALSE(m5ioe1 != NULL, ESP_ERR_NO_MEM, TAG, "Malloc failed");

    // Add new I2C device
    esp_err_t ret = ESP_OK;
    const i2c_device_config_t i2c_dev_cfg = {
        .device_address = dev_addr,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &m5ioe1->i2c_handle), err, TAG,
                      "Add new I2C device failed");

    uint8_t revision = 0;
    ESP_GOTO_ON_ERROR(read_reg(m5ioe1, REG_REV, &revision), err1, TAG, "Read revision failed");
    ESP_LOGI(TAG, "Chip revision: 0x%02X", revision);

    m5ioe1->base.config.io_count = IO_COUNT;
    m5ioe1->base.config.flags.dir_out_bit_zero = 0;       // 0 = input, 1 = output
    m5ioe1->base.config.flags.input_high_bit_zero = 0;    // 0 = low, 1 = high
    m5ioe1->base.config.flags.output_high_bit_zero = 0;   // 0 = low, 1 = high
    m5ioe1->base.config.flags.pullup_high_bit_zero = 0;   // pull select 0 = pull-down, 1 = pull-up
    m5ioe1->base.read_input_reg = read_input_reg;
    m5ioe1->base.write_output_reg = write_output_reg;
    m5ioe1->base.read_output_reg = read_output_reg;
    m5ioe1->base.write_direction_reg = write_direction_reg;
    m5ioe1->base.read_direction_reg = read_direction_reg;
    m5ioe1->base.write_highz_reg = write_highz_reg;
    m5ioe1->base.read_highz_reg = read_highz_reg;
    m5ioe1->base.write_pullup_en_reg = write_pullup_en_reg;
    m5ioe1->base.read_pullup_en_reg = read_pullup_en_reg;
    m5ioe1->base.write_pullup_sel_reg = write_pullup_sel_reg;
    m5ioe1->base.read_pullup_sel_reg = read_pullup_sel_reg;
    m5ioe1->base.del = del;
    m5ioe1->base.reset = reset;

    /* Reset configuration and register status */
    ESP_GOTO_ON_ERROR(reset(&m5ioe1->base), err1, TAG, "Reset failed");

    *handle_ret = &m5ioe1->base;
    return ESP_OK;
err1:
    i2c_master_bus_rm_device(m5ioe1->i2c_handle);
err:
    free(m5ioe1);
    return ret;
}

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);

    uint16_t temp = 0;
    ESP_RETURN_ON_ERROR(read_reg16(m5ioe1, REG_GPIO_IN_L, &temp), TAG, "Read input reg failed");
    *value = temp & IO_MASK;
    return ESP_OK;
}

static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);
    value &= IO_MASK;

    ESP_RETURN_ON_ERROR(write_reg16(m5ioe1, REG_GPIO_OUT_L, value), TAG, "Write output reg failed");
    m5ioe1->regs.output = value;
    return ESP_OK;
}

static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);

    *value = m5ioe1->regs.output;
    return ESP_OK;
}

static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);
    value &= IO_MASK;

    ESP_RETURN_ON_ERROR(write_reg16(m5ioe1, REG_GPIO_MODE_L, value), TAG, "Write direction reg failed");
    m5ioe1->regs.direction = value;
    return ESP_OK;
}

static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);

    *value = m5ioe1->regs.direction;
    return ESP_OK;
}

static esp_err_t write_highz_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);
    value &= IO_MASK;

    /* Drive register: 0 = push-pull, 1 = open-drain. */
    ESP_RETURN_ON_ERROR(write_reg16(m5ioe1, REG_GPIO_DRV_L, value), TAG, "Write drive reg failed");
    m5ioe1->regs.highz = value;
    return ESP_OK;
}

static esp_err_t read_highz_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);

    *value = m5ioe1->regs.highz;
    return ESP_OK;
}

static esp_err_t write_pullup_en_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);
    value &= IO_MASK;

    uint16_t prev = m5ioe1->regs.pull_en;
    m5ioe1->regs.pull_en = value;
    esp_err_t ret = apply_pull_regs(m5ioe1);
    if (ret != ESP_OK) {
        m5ioe1->regs.pull_en = prev;
    }
    return ret;
}

static esp_err_t read_pullup_en_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);

    *value = m5ioe1->regs.pull_en;
    return ESP_OK;
}

static esp_err_t write_pullup_sel_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);
    value &= IO_MASK;

    uint16_t prev = m5ioe1->regs.pull_sel;
    m5ioe1->regs.pull_sel = value;
    esp_err_t ret = apply_pull_regs(m5ioe1);
    if (ret != ESP_OK) {
        m5ioe1->regs.pull_sel = prev;
    }
    return ret;
}

static esp_err_t read_pullup_sel_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);

    *value = m5ioe1->regs.pull_sel;
    return ESP_OK;
}

static esp_err_t reset(esp_io_expander_t *handle)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);

    ESP_RETURN_ON_ERROR(write_direction_reg(handle, DIR_REG_DEFAULT_VAL), TAG, "Write dir reg failed");
    ESP_RETURN_ON_ERROR(write_output_reg(handle, OUT_REG_DEFAULT_VAL), TAG, "Write output reg failed");
    ESP_RETURN_ON_ERROR(write_highz_reg(handle, HIGHZ_REG_DEFAULT_VAL), TAG, "Write drive reg failed");
    m5ioe1->regs.pull_sel = PULL_SEL_DEFAULT_VAL;
    m5ioe1->regs.pull_en = PULL_EN_DEFAULT_VAL;
    ESP_RETURN_ON_ERROR(apply_pull_regs(m5ioe1), TAG, "Write pull reg failed");
    /* Leave interrupt outputs quiet after init. */
    ESP_RETURN_ON_ERROR(write_reg16(m5ioe1, REG_GPIO_IE_L, 0), TAG, "Clear interrupt enable failed");

    return ESP_OK;
}

static esp_err_t del(esp_io_expander_t *handle)
{
    esp_io_expander_m5ioe1_t *m5ioe1 = (esp_io_expander_m5ioe1_t *)__containerof(handle, esp_io_expander_m5ioe1_t, base);

    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(m5ioe1->i2c_handle), TAG, "Remove I2C device failed");
    free(m5ioe1);
    return ESP_OK;
}
