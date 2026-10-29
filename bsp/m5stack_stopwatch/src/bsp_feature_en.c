/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_check.h"
#include "driver/gpio.h"

#include "bsp/m5stack_stopwatch.h"

#define I2C_TIMEOUT_MS            (1000)
#define BSP_M5PM1_ADDR            (0x6E)
#define BSP_M5PM1_REG_PWR_CFG     (0x06)
#define BSP_M5PM1_LDO3V3_EN       (1 << 2)  /* PM_3V3_L1_EN, IMU rail */
#define BSP_M5PM1_EXT_5V_EN       (1 << 3)  /* PM_EXT_5V_EN, Grove boost */

static i2c_master_dev_handle_t m5pm1_handle = NULL;

static i2c_master_dev_handle_t bsp_m5pm1_init(void)
{
    if (m5pm1_handle) {
        return m5pm1_handle;
    }

    if (bsp_i2c_init() != ESP_OK) {
        return NULL;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BSP_M5PM1_ADDR,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(bsp_i2c_get_handle(), &dev_cfg, &m5pm1_handle) != ESP_OK) {
        m5pm1_handle = NULL;
    }
    return m5pm1_handle;
}

static esp_err_t bsp_m5pm1_set_enable(uint8_t mask, bool enable)
{
    i2c_master_dev_handle_t pmic = bsp_m5pm1_init();
    if (pmic == NULL) {
        return ESP_FAIL;
    }

    uint8_t reg = BSP_M5PM1_REG_PWR_CFG;
    uint8_t value = 0;
    esp_err_t err = i2c_master_transmit_receive(pmic, &reg, 1, &value, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t next = enable ? (uint8_t)(value | mask) : (uint8_t)(value & ~mask);
    if (next == value) {
        return ESP_OK;
    }

    uint8_t data[2] = {reg, next};
    return i2c_master_transmit(pmic, data, sizeof(data), I2C_TIMEOUT_MS);
}

/* Feature enable */
esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable)
{
    esp_err_t ret = ESP_OK;

    switch (feature) {
    case BSP_FEATURE_LCD: {
        esp_io_expander_handle_t io_expander = bsp_io_expander_init();
        ret |= esp_io_expander_set_dir(io_expander, BSP_LCD_EN, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_expander, BSP_LCD_EN, enable);
        break;
    }
    case BSP_FEATURE_SPEAKER: {
        esp_io_expander_handle_t io_expander = bsp_io_expander_init();
        ret |= esp_io_expander_set_dir(io_expander, BSP_SPEAKER_EN, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_expander, BSP_SPEAKER_EN, enable);
        break;
    }
    case BSP_FEATURE_MIC: {
        esp_io_expander_handle_t io_expander = bsp_io_expander_init();
        ret |= esp_io_expander_set_dir(io_expander, BSP_MIC_EN, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_expander, BSP_MIC_EN, enable);
        break;
    }
    case BSP_FEATURE_IMU: {
        ret |= bsp_m5pm1_set_enable(BSP_M5PM1_LDO3V3_EN, enable);
        break;
    }
    case BSP_FEATURE_EXT_5V: {
        ret |= bsp_m5pm1_set_enable(BSP_M5PM1_EXT_5V_EN, enable);
        break;
    }
    }

    return ret;
}
