/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bsp_err_check.h"
#include "bsp/esp32_s3_korvo_2.h"
static adc_cali_handle_t bsp_adc_cali_handle = NULL;
static adc_oneshot_unit_handle_t bsp_adc_handle = NULL;

esp_err_t bsp_voltage_init(void)
{
    /* Initialize ADC and get ADC handle */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_adc_initialize());
    bsp_adc_handle = bsp_adc_get_handle();

    /* Init ADC1 channels */
    const adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    BSP_ERROR_CHECK_RETURN_ERR(adc_oneshot_config_channel(bsp_adc_handle, ADC_CHANNEL_5, &config));

    /* ESP32-S3 supports Curve Fitting calibration scheme */
    const adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = BSP_ADC_UNIT,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    BSP_ERROR_CHECK_RETURN_ERR(adc_cali_create_scheme_curve_fitting(&cali_config, &bsp_adc_cali_handle));
    return ESP_OK;
}

int bsp_voltage_battery_get(void)
{
    int voltage, adc_raw;

    assert(bsp_adc_handle);
    BSP_ERROR_CHECK(adc_oneshot_read(bsp_adc_handle, ADC_CHANNEL_5, &adc_raw), -1);
    BSP_ERROR_CHECK(adc_cali_raw_to_voltage(bsp_adc_cali_handle, adc_raw, &voltage), -1);
    return voltage * BSP_BATTERY_VOLTAGE_DIV;
}
