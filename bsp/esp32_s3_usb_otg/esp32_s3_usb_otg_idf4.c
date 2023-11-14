/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "bsp/esp32_s3_usb_otg.h"
#include "bsp_err_check.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

static esp_adc_cal_characteristics_t bsp_adc_chars;

esp_err_t bsp_adc_initialize(void)
{
    esp_err_t ret = ESP_OK;
    BSP_ERROR_CHECK_RETURN_ERR(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP_FIT));
    esp_adc_cal_characterize(BSP_ADC_UNIT, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &bsp_adc_chars);

    /* ADC1 config */
    BSP_ERROR_CHECK_RETURN_ERR(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    return ret;
}

esp_err_t bsp_voltage_init(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(bsp_adc_initialize());
    BSP_ERROR_CHECK_RETURN_ERR(adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11));
    BSP_ERROR_CHECK_RETURN_ERR(adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11));

    return ESP_OK;
}

int bsp_voltage_battery_get(void)
{
    int voltage, adc_raw;

    adc_raw = adc1_get_raw(ADC1_CHANNEL_5);
    voltage = esp_adc_cal_raw_to_voltage(adc_raw, &bsp_adc_chars);
    return voltage * BSP_BATTERY_VOLTAGE_DIV;
}

int bsp_voltage_usb_get(void)
{
    int voltage, adc_raw;

    adc_raw = adc1_get_raw(ADC_CHANNEL_0);
    voltage = esp_adc_cal_raw_to_voltage(adc_raw, &bsp_adc_chars);
    return voltage * BSP_USB_HOST_VOLTAGE_DIV;
}
