/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "bsp/esp32_s3_korvo_1.h"
#include "bsp_err_check.h"
#include "esp_adc_cal.h"

static esp_adc_cal_characteristics_t bsp_adc_chars;

esp_err_t bsp_adc_initialize(void)
{
    esp_err_t ret = ESP_OK;
    BSP_ERROR_CHECK_RETURN_ERR(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP_FIT));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &bsp_adc_chars);

    /* ADC1 config */
    BSP_ERROR_CHECK_RETURN_ERR(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    return ret;
}
