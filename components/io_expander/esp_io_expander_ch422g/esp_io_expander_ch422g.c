/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileCopyrightText: 2025 Frédéric Nadeau
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
#include "esp_io_expander_ch422g.h"

/* I2C communication related */
#define I2C_TIMEOUT_MS          (1000)
#define I2C_CLK_SPEED           (400000)

#define IO_COUNT                (8)

/* Register address */
#define GENERAL_PURPOSE_OUTPUT_ADDR   (0x24)
#define SET_IO_ADDR          (0x38)
#define READ_IO_ADDR         (0x26)

/* Default register value on power-up */
#define DIR_REG_DEFAULT_VAL     (0xFF)
#define OUT_REG_DEFAULT_VAL     (0x00)

/**
 * @brief Device Structure Type
 *
 */
typedef struct {
    esp_io_expander_t base;
    i2c_master_dev_handle_t i2c_gpo_handle;
    i2c_master_dev_handle_t i2c_set_io_handle;
    i2c_master_dev_handle_t i2c_read_io_handle;
    struct {
        uint8_t direction;
        uint8_t output;
    } regs;
} esp_io_expander_ch422g_t;

static const char *TAG = "ch422g";

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value);
static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value);
static esp_err_t reset(esp_io_expander_t *handle);
static esp_err_t del(esp_io_expander_t *handle);

esp_err_t esp_io_expander_new_i2c_ch422g(i2c_master_bus_handle_t i2c_bus, esp_io_expander_handle_t *handle_ret)
{
    ESP_RETURN_ON_FALSE(handle_ret != NULL, ESP_ERR_INVALID_ARG, TAG, "Invalid handle_ret");

    // Allocate memory for driver object
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)calloc(1, sizeof(esp_io_expander_ch422g_t));
    ESP_RETURN_ON_FALSE(ch422g != NULL, ESP_ERR_NO_MEM, TAG, "Malloc failed");

    // Add new I2C device
    esp_err_t ret = ESP_OK;
    const i2c_device_config_t i2c_dev_cfg1 = {
        .device_address = GENERAL_PURPOSE_OUTPUT_ADDR,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg1, &ch422g->i2c_gpo_handle), err, TAG, "Add new I2C device (GENERAL_PURPOSE_OUTPUT_ADDR) failed");

    const i2c_device_config_t i2c_dev_cfg2 = {
        .device_address = SET_IO_ADDR,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg2, &ch422g->i2c_set_io_handle), err, TAG, "Add new I2C device (SET_IO_ADDR) failed");

    const i2c_device_config_t i2c_dev_cfg3 = {
        .device_address = READ_IO_ADDR,
        .scl_speed_hz = I2C_CLK_SPEED,
    };
    ESP_GOTO_ON_ERROR(i2c_master_bus_add_device(i2c_bus, &i2c_dev_cfg3, &ch422g->i2c_read_io_handle), err, TAG, "Add new I2C device (READ_IO_ADDR) failed");

    ch422g->base.config.io_count = IO_COUNT;
    ch422g->base.config.flags.dir_out_bit_zero = 0;
    ch422g->base.read_input_reg = read_input_reg;
    ch422g->base.write_output_reg = write_output_reg;
    ch422g->base.read_output_reg = read_output_reg;
    ch422g->base.write_direction_reg = write_direction_reg;
    ch422g->base.read_direction_reg = read_direction_reg;
    ch422g->base.del = del;
    ch422g->base.reset = reset;

    /* Reset configuration and register status */
    ESP_GOTO_ON_ERROR(reset(&ch422g->base), err, TAG, "Reset failed");

    *handle_ret = &ch422g->base;
    return ESP_OK;
err:
    if (ch422g->i2c_gpo_handle)
        i2c_master_bus_rm_device(ch422g->i2c_gpo_handle);
    if (ch422g->i2c_set_io_handle)
        i2c_master_bus_rm_device(ch422g->i2c_set_io_handle);
    if (ch422g->i2c_read_io_handle)
        i2c_master_bus_rm_device(ch422g->i2c_read_io_handle);

    free(ch422g);
    return ret;
}

static esp_err_t read_input_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)__containerof(handle, esp_io_expander_ch422g_t, base);

    uint8_t temp = 0;
    ESP_RETURN_ON_ERROR(i2c_master_receive(ch422g->i2c_read_io_handle, &temp, sizeof(temp), I2C_TIMEOUT_MS), TAG, "Read input reg failed");
    *value = temp;
    return ESP_OK;
}

static esp_err_t write_output_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)__containerof(handle, esp_io_expander_ch422g_t, base);
    value &= 0xff;

    uint8_t data[] = {value};
    ESP_RETURN_ON_ERROR(i2c_master_transmit(ch422g->i2c_set_io_handle, data, sizeof(data), I2C_TIMEOUT_MS), TAG, "Write output reg failed");
    ch422g->regs.output = value;
    return ESP_OK;
}

static esp_err_t read_output_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)__containerof(handle, esp_io_expander_ch422g_t, base);

    *value = ch422g->regs.output;
    return ESP_OK;
}

static esp_err_t write_direction_reg(esp_io_expander_handle_t handle, uint32_t value)
{
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)__containerof(handle, esp_io_expander_ch422g_t, base);
    value &= 0xff;

    /* CH422G limitation: all bidirectional I/Os must be inputs or outputs together.
     * Accept only 0x00 (all inputs) or 0xFF (all outputs) and reject mixed masks.
     */
    if (value != 0x00 && value != 0xFF) {
        ESP_LOGE(TAG, "Invalid direction mask 0x%02" PRIx32 ", CH422G only supports all-input (0x00) or all-output (0xFF)", value);
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t data[] = {0x00};
    if (value == 0xFF) {
        /* Any non-zero config means 'all outputs' for the device; use 0x01 as per datasheet/README. */
        data[0] = 0x01;
    }
    ESP_RETURN_ON_ERROR(i2c_master_transmit(ch422g->i2c_gpo_handle, data, sizeof(data), I2C_TIMEOUT_MS), TAG, "Write direction reg failed");
    /* Cache the logical direction as a full mask for higher layers: 0x00 = all in, 0xFF = all out. */
    ch422g->regs.direction = (value == 0x00) ? 0x00 : 0xFF;
    return ESP_OK;
}

static esp_err_t read_direction_reg(esp_io_expander_handle_t handle, uint32_t *value)
{
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)__containerof(handle, esp_io_expander_ch422g_t, base);

    *value = ch422g->regs.direction;
    return ESP_OK;
}

static esp_err_t reset(esp_io_expander_t *handle)
{
    ESP_RETURN_ON_ERROR(write_direction_reg(handle, DIR_REG_DEFAULT_VAL), TAG, "Write dir reg failed");
    ESP_RETURN_ON_ERROR(write_output_reg(handle, OUT_REG_DEFAULT_VAL), TAG, "Write output reg failed");
    return ESP_OK;
}

static esp_err_t del(esp_io_expander_t *handle)
{
    esp_io_expander_ch422g_t *ch422g = (esp_io_expander_ch422g_t *)__containerof(handle, esp_io_expander_ch422g_t, base);

    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(ch422g->i2c_gpo_handle), TAG, "Remove I2C device (GENERAL_PURPOSE_OUTPUT_ADDR) failed");
    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(ch422g->i2c_set_io_handle), TAG, "Remove I2C device (SET_IO_ADDR) failed");
    ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(ch422g->i2c_read_io_handle), TAG, "Remove I2C device (READ_IO_ADDR) failed");
    free(ch422g);
    return ESP_OK;
}
