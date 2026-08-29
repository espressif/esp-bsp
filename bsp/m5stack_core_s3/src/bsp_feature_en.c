/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_check.h"
#include "bsp_err_check.h"
#include "driver/gpio.h"
#include "bsp/m5stack_core_s3.h"

#define I2C_TIMEOUT_MS          (1000)

#define BSP_AXP2101_ADDR        (0x34)
#define BSP_AXP2101_REG_EN      (0x90)  /* LDO ON/OFF Control */
#define BSP_AXP2101_REG_SPEAKER (0x92)  /* ALDO1 Voltage settings */
#define BSP_AXP2101_REG_MIC     (0x93)  /* ALDO2 Voltage settings */
#define BSP_AXP2101_REG_CAM     (0x94)  /* ALDO3 Voltage settings */
#define BSP_AXP2101_REG_SD      (0x95)  /* ALDO4 Voltage settings */
#define BSP_AXP2101_REG_LCD_BL  (0x99)  /* DLDO1 Voltage settings */


static const char *TAG = "M5Stack Core S3";

static i2c_master_dev_handle_t bsp_axp2101_handle = NULL;

static esp_err_t bsp_pmu_axp2101_set_voltage(uint8_t reg, int voltage_mv);

/* Feature enable */
esp_err_t bsp_feature_enable(bsp_feature_t feature, bool enable)
{
    esp_err_t ret = ESP_OK;

    switch (feature) {
    case BSP_FEATURE_SD: {
        esp_io_expander_handle_t io_expander = bsp_io_expander_init();
        ret |= esp_io_expander_set_dir(io_expander, BSP_SD_EN, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_expander, BSP_SD_EN, enable);
        /* Set voltage for SD Card: 3.3V */
        ret |= bsp_pmu_axp2101_set_voltage(BSP_AXP2101_REG_SD, (enable ? 3300 : 0));
        break;
    }
    case BSP_FEATURE_LCD: {
        esp_io_expander_handle_t io_expander = bsp_io_expander_init();
        ret |= esp_io_expander_set_dir(io_expander, BSP_LCD_EN, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_expander, BSP_LCD_EN, enable);
        break;
    }
    case BSP_FEATURE_TOUCH: {
        esp_io_expander_handle_t io_expander = bsp_io_expander_init();
        ret |= esp_io_expander_set_dir(io_expander, BSP_TOUCH_EN, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_expander, BSP_TOUCH_EN, enable);
        break;
    }
    case BSP_FEATURE_SPEAKER: {
        esp_io_expander_handle_t io_expander = bsp_io_expander_init();
        ret |= esp_io_expander_set_dir(io_expander, BSP_SPEAKER_EN, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_expander, BSP_SPEAKER_EN, enable);
        /* Set voltage for speaker codec: 1.8V */
        ret |= bsp_pmu_axp2101_set_voltage(BSP_AXP2101_REG_SPEAKER, (enable ? 1800 : 0));
        break;
    }
    case BSP_FEATURE_MIC: {
        esp_io_expander_handle_t io_expander = bsp_io_expander_init();
        ret |= esp_io_expander_set_dir(io_expander, BSP_MIC_EN, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_expander, BSP_MIC_EN, enable);
        /* Set voltage for microphone codec: 3.3V */
        ret |= bsp_pmu_axp2101_set_voltage(BSP_AXP2101_REG_MIC, (enable ? 3300 : 0));
        break;
    }
    case BSP_FEATURE_CAMERA: {
        esp_io_expander_handle_t io_expander = bsp_io_expander_init();
        ret |= esp_io_expander_set_dir(io_expander, BSP_CAMERA_EN, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_expander, BSP_CAMERA_EN, enable);
        /* Set voltage for Camera: 3.3V */
        ret |= bsp_pmu_axp2101_set_voltage(BSP_AXP2101_REG_CAM, (enable ? 3300 : 0));
        break;
    }
    case BSP_FEATURE_USB: {
        esp_io_expander_handle_t io_expander = bsp_io_expander_init();
        ret |= esp_io_expander_set_dir(io_expander, BSP_USB_EN, IO_EXPANDER_OUTPUT);
        ret |= esp_io_expander_set_level(io_expander, BSP_USB_EN, enable);
        break;
    }
    }

    return ret;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    }
    if (brightness_percent < 0) {
        brightness_percent = 0;
    }

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    int voltage_mv = 2500 + ((uint32_t)brightness_percent * (3300 - 2500)) / 100;
    if (brightness_percent == 0) {
        voltage_mv = 0;
    }
    return bsp_pmu_axp2101_set_voltage(BSP_AXP2101_REG_LCD_BL, voltage_mv);
}

static esp_err_t bsp_pmu_axp2101_init(void)
{
    if (bsp_axp2101_handle) {
        return ESP_OK;
    }

    /* Initilize I2C */
    BSP_ERROR_CHECK_RETURN_NULL(bsp_i2c_init());

    /* Initialize PMU AXP2101 */
    const i2c_device_config_t axp2101_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BSP_AXP2101_ADDR,
        .scl_speed_hz = 400000,
    };
    return i2c_master_bus_add_device(bsp_i2c_get_handle(), &axp2101_config, &bsp_axp2101_handle);
}

static esp_err_t bsp_pmu_axp_2101_enable(uint8_t reg, bool enable)
{
    uint8_t reg_en = 0b00000000;
    switch (reg) {
    case BSP_AXP2101_REG_MIC:
        /* ALDO1 EN */
        reg_en = 0b00000001;
        break;
    case BSP_AXP2101_REG_SPEAKER:
        /* ALDO2 EN */
        reg_en = 0b00000010;
        break;
    case BSP_AXP2101_REG_CAM:
        /* BLDO1 EN, BLDO2 EN, ALDO3 EN */
        reg_en = 0b00110100;
        break;
    case BSP_AXP2101_REG_SD:
        /* ALDO4 EN */
        reg_en = 0b00001000;
        break;
    case BSP_AXP2101_REG_LCD_BL:
        /* DLDO1 EN */
        reg_en = 0b10000000;
        break;
    }

    uint8_t temp = 0;
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(bsp_axp2101_handle, (uint8_t[]) {
        BSP_AXP2101_REG_EN
    }, 1, &temp, sizeof(temp), I2C_TIMEOUT_MS), TAG, "Read input reg failed");

    uint8_t set_data = (temp | reg_en);
    if (set_data != temp) {
        uint8_t data[2] = {BSP_AXP2101_REG_EN, set_data};
        return i2c_master_transmit(bsp_axp2101_handle, data, sizeof(data), I2C_TIMEOUT_MS);
    }

    return ESP_OK;
}

static esp_err_t bsp_pmu_axp2101_set_voltage(uint8_t reg, int voltage_mv)
{
    ESP_RETURN_ON_FALSE(voltage_mv <= 3300, ESP_ERR_INVALID_ARG, TAG,
                        "Maximum allowed voltage is 3300 mV");
    ESP_RETURN_ON_FALSE(reg >= 0x92 && reg <= 0x99, ESP_ERR_INVALID_ARG, TAG,
                        "Allowed registers to set are: 0x92 - 0x99");

    /* Initialize PMU AXP2101 */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_pmu_axp2101_init());

    uint8_t power = ((voltage_mv - 500) / 100);
    if (voltage_mv < 500) {
        power = 0;
    }
    uint8_t data[2] = {reg, power};
    BSP_ERROR_CHECK_RETURN_ERR(i2c_master_transmit(bsp_axp2101_handle, data, sizeof(data), I2C_TIMEOUT_MS));

    /* Enable/disable LDO */
    return bsp_pmu_axp_2101_enable(reg, (voltage_mv > 0));
}
