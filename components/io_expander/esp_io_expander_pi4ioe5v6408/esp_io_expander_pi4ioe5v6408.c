/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <string.h>
#include <stdlib.h>
#include "esp_bit_defs.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_io_expander.h"
#include "esp_io_expander_pi4ioe5v6408.h"

/* I2C communication related */
#define I2C_TIMEOUT_MS          (50)
#define I2C_CLK_SPEED           (400000)

#define IO_COUNT                (8)

/* PI4IOE5V6408 register addresses */
#define PI4IO_REG_CHIP_RESET    0x01
#define PI4IO_REG_IO_DIR        0x03
#define PI4IO_REG_OUT_SET       0x05
#define PI4IO_REG_OUT_H_IM      0x07
#define PI4IO_REG_IN_DEF_STA    0x09
#define PI4IO_REG_PULL_EN       0x0B
#define PI4IO_REG_PULL_SEL      0x0D
#define PI4IO_REG_IN_STA        0x0F
#define PI4IO_REG_INT_MASK      0x11
#define PI4IO_REG_IRQ_STA       0x13

/* Default register value on power-up */
#define DIR_REG_DEFAULT_VAL     (0xff)
#define OUT_REG_DEFAULT_VAL     (0x00)
#define HIGHZ_REG_DEFAULT_VAL   (0xff)
#define PULLEN_REG_DEFAULT_VAL  (0xff)
#define PULLSEL_REG_DEFAULT_VAL (0x00)

/**
 * @brief Device Structure Type
 *
 */
typedef struct {
    esp_io_expander_t base;
    i2c_master_dev_handle_t i2c_handle;
    struct {
        uint8_t direction;
        uint8_t output;
        uint8_t highz;
        uint8_t pullup_en;
        uint8_t pullup_sel;
    } regs;
} esp_io_expander_pi4ioe5v6408_t;

static char *TAG = "pi4ioe5v6408";

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

esp_err_t esp_io_expander_new_i2c_pi4ioe5v6408(i2c_master_bus_handle_t i2c_bus, uint32_t dev_addr,
        esp_io_expander_handle_t *handle_ret)
{
    ESP_RETURN_ON_FALSE(handle_ret != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle_ret");

    // Allocate memory for driver object
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)calloc(1,
            sizeof(esp_io_expander_pi4ioe5v6408_t));
    ESP_RETURN_ON_FALSE(pi4ioe != NULL, ESP_ERR_NO_MEM, TAG, "Malloc failed");

    // Add new I2C device
    esp_err_t ret = ESP_OK;
    const i2c_device_config_t i2c_dev_cfg = {
        .device_address = dev_addr,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &pi4ioe->i2c_handle), err, TAG,
                      "Add new I2C device failed");

    pi4ioe->base.config.io_count = IO_COUNT;
    pi4ioe->base.config.flags.dir_out_bit_zero = 0;  // PI4IOE: 0=input, 1=output
    pi4ioe->base.config.flags.output_high_bit_zero = 0;  // PI4IOE: 0=low, 1=high
    pi4ioe->base.config.flags.pullup_high_bit_zero = 0;  // PI4IOE: 0=Pull-down, 1=Pull-up
    pi4ioe->base.read_input_reg = read_input_reg;
    pi4ioe->base.write_output_reg = write_output_reg;
    pi4ioe->base.read_output_reg = read_output_reg;
    pi4ioe->base.write_direction_reg = write_direction_reg;
    pi4ioe->base.read_direction_reg = read_direction_reg;
    pi4ioe->base.write_highz_reg = write_highz_reg;
    pi4ioe->base.read_highz_reg = read_highz_reg;
    pi4ioe->base.write_pullup_en_reg = write_pullup_en_reg;
    pi4ioe->base.read_pullup_en_reg = read_pullup_en_reg;
    pi4ioe->base.write_pullup_sel_reg = write_pullup_sel_reg;
    pi4ioe->base.read_pullup_sel_reg = read_pullup_sel_reg;
    pi4ioe->base.del = del;
    pi4ioe->base.reset = reset;

    /* Reset configuration and register status */
    ESP_GOTO_ON_ERROR(reset(&pi4ioe->base), err1, TAG, "Reset failed");

    *handle_ret = &pi4ioe->base;
    return ESP_OK;
err1:
    i2c_master_bus_rm_device(pi4ioe->i2c_handle);
err:
    free(pi4ioe);
    return ret;
}

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);

    uint8_t temp = 0;
    uint8_t reg_addr = PI4IO_REG_IN_STA;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(pi4ioe->i2c_handle, &reg_addr, 1, &temp, sizeof(temp), I2C_TIMEOUT_MS),
                        TAG, "Read input reg failed");
    *value = temp;
    return ESP_OK;
}

static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);
    value &= 0xff;

    uint8_t data[] = {PI4IO_REG_OUT_SET, value};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, data, sizeof(data), I2C_TIMEOUT_MS), TAG,
                        "Write output reg failed");
    pi4ioe->regs.output = value;
    return ESP_OK;
}

static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);

    *value = pi4ioe->regs.output;
    return ESP_OK;
}

static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);
    value &= 0xff;

    uint8_t data[] = {PI4IO_REG_IO_DIR, value};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, data, sizeof(data), I2C_TIMEOUT_MS), TAG,
                        "Write direction reg failed");
    pi4ioe->regs.direction = value;
    return ESP_OK;
}

static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);

    *value = pi4ioe->regs.direction;
    return ESP_OK;
}

static esp_err_t write_highz_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);
    value &= 0xff;

    uint8_t data[] = {PI4IO_REG_OUT_H_IM, value};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, data, sizeof(data), I2C_TIMEOUT_MS), TAG,
                        "Write highz reg failed");
    pi4ioe->regs.highz = value;
    return ESP_OK;
}

static esp_err_t read_highz_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);

    *value = pi4ioe->regs.highz;
    return ESP_OK;
}

static esp_err_t write_pullup_en_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);
    value &= 0xff;

    uint8_t data[] = {PI4IO_REG_PULL_EN, value};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, data, sizeof(data), I2C_TIMEOUT_MS), TAG,
                        "Write highz reg failed");
    pi4ioe->regs.pullup_en = value;
    return ESP_OK;
}

static esp_err_t read_pullup_en_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);

    *value = pi4ioe->regs.pullup_en;
    return ESP_OK;
}

static esp_err_t write_pullup_sel_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);
    value &= 0xff;

    uint8_t data[] = {PI4IO_REG_PULL_SEL, value};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, data, sizeof(data), I2C_TIMEOUT_MS), TAG,
                        "Write highz reg failed");
    pi4ioe->regs.pullup_sel = value;
    return ESP_OK;
}

static esp_err_t read_pullup_sel_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);

    *value = pi4ioe->regs.pullup_sel;
    return ESP_OK;
}

static esp_err_t reset(esp_io_expander_t *handle)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);

    // Reset chip
    uint8_t write_buf[2] = {PI4IO_REG_CHIP_RESET, 0xFF};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, write_buf, 2, I2C_TIMEOUT_MS), TAG, "Chip reset failed");

    // Read reset status
    uint8_t read_buf[1] = {0};
    write_buf[0] = PI4IO_REG_CHIP_RESET;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(pi4ioe->i2c_handle, write_buf, 1, read_buf, 1, I2C_TIMEOUT_MS), TAG,
                        "Read reset status failed");

    // Set default direction (all inputs)
    ESP_RETURN_ON_ERROR(write_direction_reg(handle, DIR_REG_DEFAULT_VAL), TAG, "Write dir reg failed");
    // Set default Hight-Z (all low)
    ESP_RETURN_ON_ERROR(write_highz_reg(handle, HIGHZ_REG_DEFAULT_VAL), TAG, "Write High-Z reg failed");
    // Set default Pull Sel (all low)
    ESP_RETURN_ON_ERROR(write_pullup_sel_reg(handle, PULLSEL_REG_DEFAULT_VAL), TAG, "Write Pull Select reg failed");
    // Set default Pull En (all low)
    ESP_RETURN_ON_ERROR(write_pullup_en_reg(handle, PULLEN_REG_DEFAULT_VAL), TAG, "Write Pull Enabled failed");
    // Set default output (all low)
    ESP_RETURN_ON_ERROR(write_output_reg(handle, OUT_REG_DEFAULT_VAL), TAG, "Write output reg failed");

    return ESP_OK;
}

static esp_err_t del(esp_io_expander_t *handle)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle,
            esp_io_expander_pi4ioe5v6408_t, base);

    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(pi4ioe->i2c_handle), TAG, "Remove I2C device failed");
    free(pi4ioe);
    return ESP_OK;
}
