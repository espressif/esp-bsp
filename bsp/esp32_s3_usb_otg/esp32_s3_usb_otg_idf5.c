/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "bsp/esp32_s3_usb_otg.h"
#include "bsp_err_check.h"

static adc_oneshot_unit_handle_t bsp_adc_handle = NULL;
static adc_cali_handle_t bsp_adc_cali_handle; /* ADC1 calibration handle */


esp_err_t bsp_adc_initialize(void)
{
    /* ADC was initialized before */
    if (bsp_adc_handle != NULL) {
        return ESP_OK;
    }

    /* Initialize ADC */
    const adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = BSP_ADC_UNIT,
    };
    BSP_ERROR_CHECK_RETURN_ERR(adc_oneshot_new_unit(&init_config1, &bsp_adc_handle));

    return ESP_OK;
}

adc_oneshot_unit_handle_t bsp_adc_get_handle(void)
{
    return bsp_adc_handle;
}

esp_err_t bsp_voltage_init(void)
{
    /* Initialize ADC and get ADC handle */
    BSP_ERROR_CHECK_RETURN_NULL(bsp_adc_initialize());

    /* Init ADC1 channels */
    const adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    BSP_ERROR_CHECK_RETURN_ERR(adc_oneshot_config_channel(bsp_adc_handle, ADC_CHANNEL_0, &config));
    BSP_ERROR_CHECK_RETURN_ERR(adc_oneshot_config_channel(bsp_adc_handle, ADC_CHANNEL_5, &config));

    /* ESP32-S3 supports Curve Fitting calibration scheme */
    const adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
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

int bsp_voltage_usb_get(void)
{
    int voltage, adc_raw;

    assert(bsp_adc_handle);
    BSP_ERROR_CHECK(adc_oneshot_read(bsp_adc_handle, ADC_CHANNEL_0, &adc_raw), -1);
    BSP_ERROR_CHECK(adc_cali_raw_to_voltage(bsp_adc_cali_handle, adc_raw, &voltage), -1);
    return (float)voltage * BSP_USB_HOST_VOLTAGE_DIV;
}
