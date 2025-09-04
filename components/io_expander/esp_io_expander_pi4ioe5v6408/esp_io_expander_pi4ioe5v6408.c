/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
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
    } regs;
} esp_io_expander_pi4ioe5v6408_t;

static char *TAG = "pi4ioe5v6408";

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t reset(esp_io_expander_t *handle);
static esp_err_t del(esp_io_expander_t *handle);

esp_err_t esp_io_expander_new_i2c_pi4ioe5v6408(i2c_master_bus_handle_t i2c_bus, uint32_t dev_addr, esp_io_expander_handle_t *handle_ret)
{
    ESP_RETURN_ON_FALSE(handle_ret != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle_ret");

    // Allocate memory for driver object
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)calloc(1, sizeof(esp_io_expander_pi4ioe5v6408_t));
    ESP_RETURN_ON_FALSE(pi4ioe != NULL, ESP_ERR_NO_MEM, TAG, "Malloc failed");

    // Add new I2C device
    esp_err_t ret = ESP_OK;
    const i2c_device_config_t i2c_dev_cfg = {
        .device_address = dev_addr,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg, &pi4ioe->i2c_handle), err, TAG, "Add new I2C device failed");

    pi4ioe->base.config.io_count = IO_COUNT;
    pi4ioe->base.config.flags.dir_out_bit_zero = 0;  // PI4IOE: 0=input, 1=output
    pi4ioe->base.config.flags.output_high_bit_zero = 0;  // PI4IOE: 0=low, 1=high
    pi4ioe->base.read_input_reg = read_input_reg;
    pi4ioe->base.write_output_reg = write_output_reg;
    pi4ioe->base.read_output_reg = read_output_reg;
    pi4ioe->base.write_direction_reg = write_direction_reg;
    pi4ioe->base.read_direction_reg = read_direction_reg;
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
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle, esp_io_expander_pi4ioe5v6408_t, base);

    uint8_t temp = 0;
    uint8_t reg_addr = PI4IO_REG_IN_STA;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(pi4ioe->i2c_handle, &reg_addr, 1, &temp, sizeof(temp), I2C_TIMEOUT_MS), TAG, "Read input reg failed");
    *value = temp;
    return ESP_OK;
}

static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle, esp_io_expander_pi4ioe5v6408_t, base);
    value &= 0xff;

    uint8_t data[] = {PI4IO_REG_OUT_SET, value};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, data, sizeof(data), I2C_TIMEOUT_MS), TAG, "Write output reg failed");
    pi4ioe->regs.output = value;
    return ESP_OK;
}

static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle, esp_io_expander_pi4ioe5v6408_t, base);

    *value = pi4ioe->regs.output;
    return ESP_OK;
}

static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle, esp_io_expander_pi4ioe5v6408_t, base);
    value &= 0xff;

    uint8_t data[] = {PI4IO_REG_IO_DIR, value};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, data, sizeof(data), I2C_TIMEOUT_MS), TAG, "Write direction reg failed");
    pi4ioe->regs.direction = value;
    return ESP_OK;
}

static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle, esp_io_expander_pi4ioe5v6408_t, base);

    *value = pi4ioe->regs.direction;
    return ESP_OK;
}

static esp_err_t reset(esp_io_expander_t *handle)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle, esp_io_expander_pi4ioe5v6408_t, base);

    // Reset chip
    uint8_t write_buf[2] = {PI4IO_REG_CHIP_RESET, 0xFF};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, write_buf, 2, I2C_TIMEOUT_MS), TAG, "Chip reset failed");

    // Read reset status
    uint8_t read_buf[1] = {0};
    write_buf[0] = PI4IO_REG_CHIP_RESET;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(pi4ioe->i2c_handle, write_buf, 1, read_buf, 1, I2C_TIMEOUT_MS), TAG, "Read reset status failed");

    // Set default direction (all inputs)
    ESP_RETURN_ON_ERROR(write_direction_reg(handle, DIR_REG_DEFAULT_VAL), TAG, "Write dir reg failed");
    // Set default output (all low)
    ESP_RETURN_ON_ERROR(write_output_reg(handle, OUT_REG_DEFAULT_VAL), TAG, "Write output reg failed");

    return ESP_OK;
}

static esp_err_t del(esp_io_expander_t *handle)
{
    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle, esp_io_expander_pi4ioe5v6408_t, base);

    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(pi4ioe->i2c_handle), TAG, "Remove I2C device failed");
    free(pi4ioe);
    return ESP_OK;
}

esp_err_t esp_io_expander_pi4ioe5v6408_config_registers(esp_io_expander_handle_t handle, const esp_io_expander_pi4ioe5v6408_config_t *config)
{
    if (handle == NULL || config == NULL) {
        ESP_LOGE(TAG, "Invalid handle or config");
        return ESP_ERR_INVALID_ARG;
    }

    esp_io_expander_pi4ioe5v6408_t *pi4ioe = (esp_io_expander_pi4ioe5v6408_t *)__containerof(handle, esp_io_expander_pi4ioe5v6408_t, base);
    uint8_t write_buf[2] = {0};
    uint8_t read_buf[1] = {0};

    // Reset chip first
    write_buf[0] = PI4IO_REG_CHIP_RESET;
    write_buf[1] = 0xFF;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, write_buf, 2, I2C_TIMEOUT_MS), TAG, "Chip reset failed");

    // Read reset status
    write_buf[0] = PI4IO_REG_CHIP_RESET;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(pi4ioe->i2c_handle, write_buf, 1, read_buf, 1, I2C_TIMEOUT_MS), TAG, "Read reset status failed");

    // Set direction register
    write_buf[0] = PI4IO_REG_IO_DIR;
    write_buf[1] = config->io_dir;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, write_buf, 2, I2C_TIMEOUT_MS), TAG, "Set direction failed");

    // Set output high impedance
    write_buf[0] = PI4IO_REG_OUT_H_IM;
    write_buf[1] = config->out_h_im;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, write_buf, 2, I2C_TIMEOUT_MS), TAG, "Set output high impedance failed");

    // Set pull up/down select
    write_buf[0] = PI4IO_REG_PULL_SEL;
    write_buf[1] = config->pull_sel;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, write_buf, 2, I2C_TIMEOUT_MS), TAG, "Set pull select failed");

    // Enable/disable pull up/down
    write_buf[0] = PI4IO_REG_PULL_EN;
    write_buf[1] = config->pull_en;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, write_buf, 2, I2C_TIMEOUT_MS), TAG, "Set pull enable failed");

    // Set input default status (if needed)
    if (config->in_def_sta != 0xFF) {  // 0xFF means skip this register
        write_buf[0] = PI4IO_REG_IN_DEF_STA;
        write_buf[1] = config->in_def_sta;
        ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, write_buf, 2, I2C_TIMEOUT_MS), TAG, "Set input default status failed");
    }

    // Set interrupt mask (if needed)
    if (config->int_mask != 0xFF) {  // 0xFF means skip this register
        write_buf[0] = PI4IO_REG_INT_MASK;
        write_buf[1] = config->int_mask;
        ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, write_buf, 2, I2C_TIMEOUT_MS), TAG, "Set interrupt mask failed");
    }

    // Set output values
    write_buf[0] = PI4IO_REG_OUT_SET;
    write_buf[1] = config->out_set;
    ESP_RETURN_ON_ERROR(i2c_master_transmit(pi4ioe->i2c_handle, write_buf, 2, I2C_TIMEOUT_MS), TAG, "Set output values failed");

    // Update internal register cache
    pi4ioe->regs.direction = config->io_dir;
    pi4ioe->regs.output = config->out_set;

    return ESP_OK;
}
