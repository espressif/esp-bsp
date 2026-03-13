/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include "esp_err.h"
#include "esp_log.h"
#include "bsp_err_check.h"
#include "iot_button.h"
#include "button_gpio.h"
#include "button_adc.h"

#include "bsp/esp32_s3_korvo_2.h"

static const char *TAG = "ESP32-S3-Korvo-2";
static adc_oneshot_unit_handle_t bsp_adc_handle = NULL;


typedef enum {
    BSP_BUTTON_TYPE_GPIO,
    BSP_BUTTON_TYPE_ADC,
} bsp_button_type_t;

typedef struct {
    bsp_button_type_t type;
    union {
        button_gpio_config_t gpio;
        button_adc_config_t  adc;
    } cfg;
} bsp_button_config_t;

static const bsp_button_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_handle = &bsp_adc_handle,
            .adc_channel = ADC_CHANNEL_4,
            .button_index = BSP_BUTTON_REC,
            .min = (2310), // middle is 2410
            .max = (2510)
        }

    },
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_handle = &bsp_adc_handle,
            .adc_channel = ADC_CHANNEL_4,
            .button_index = BSP_BUTTON_MUTE,
            .min = (1880), // middle is 1980
            .max = (2080)
        }

    },
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_handle = &bsp_adc_handle,
            .adc_channel = ADC_CHANNEL_4,
            .button_index = BSP_BUTTON_PLAY,
            .min = (1550), // middle is 1650
            .max = (1750)
        }

    },
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_handle = &bsp_adc_handle,
            .adc_channel = ADC_CHANNEL_4,
            .button_index = BSP_BUTTON_SET,
            .min = (1010), // middle is 1110
            .max = (1210)
        }

    },
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_handle = &bsp_adc_handle,
            .adc_channel = ADC_CHANNEL_4,
            .button_index = BSP_BUTTON_VOLDOWN,
            .min = (720), // middle is 820
            .max = (920)
        }

    },
    {
        .type = BSP_BUTTON_TYPE_ADC,
        .cfg.adc = {
            .adc_handle = &bsp_adc_handle,
            .adc_channel = ADC_CHANNEL_4,
            .button_index = BSP_BUTTON_VOLUP,
            .min = (280), // middle is 380
            .max = (480)
        }

    },
};




esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size)
{
    /* Initialize ADC and get ADC handle */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_adc_initialize());
    bsp_adc_handle = bsp_adc_get_handle();

    esp_err_t ret = ESP_OK;
    const button_config_t btn_config = {0};
    if ((btn_array_size < BSP_BUTTON_NUM) ||
            (btn_array == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (btn_cnt) {
        *btn_cnt = 0;
    }
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        if (bsp_button_config[i].type == BSP_BUTTON_TYPE_GPIO) {
            ret |= iot_button_new_gpio_device(&btn_config, &bsp_button_config[i].cfg.gpio, &btn_array[i]);
        } else if (bsp_button_config[i].type == BSP_BUTTON_TYPE_ADC) {
            ret |= iot_button_new_adc_device(&btn_config, &bsp_button_config[i].cfg.adc, &btn_array[i]);
        } else {
            ESP_LOGW(TAG, "Unsupported button type!");
        }

        if (btn_cnt) {
            (*btn_cnt)++;
        }
    }
    return ret;
}
