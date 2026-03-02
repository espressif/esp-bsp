/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_err.h"
#include "esp_log.h"
#include "bsp_err_check.h"
#include "driver/touch_sens.h"
#include "iot_button.h"
#include "button_gpio.h"

#include "bsp/echoear.h"

static const char *TAG = "EchoEar";


static esp_err_t  bsp_touchpad_custom_deinit(button_driver_t *button_driver);
static uint8_t bsp_touchpad_custom_get_key_value(button_driver_t *button_driver);

typedef enum {
    BSP_BUTTON_TYPE_GPIO,
    BSP_BUTTON_TYPE_TOUCHPAD
} bsp_button_type_t;

typedef struct {
    button_driver_t base;
    int             key;
} bsp_button_type_touchpad_t;

typedef struct {
    bsp_button_type_t type;
    union {
        button_gpio_config_t gpio;
        bsp_button_type_touchpad_t custom;
    } cfg;
} bsp_button_config_t;

static const bsp_button_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .type = BSP_BUTTON_TYPE_TOUCHPAD,
        .cfg.custom = {
            .base = {
                .get_key_level = bsp_touchpad_custom_get_key_value,
                .del = bsp_touchpad_custom_deinit,
            },
            .key = BSP_BUTTON_1_CH,
        }
    },
    {
        .type = BSP_BUTTON_TYPE_TOUCHPAD,
        .cfg.custom = {
            .base = {
                .get_key_level = bsp_touchpad_custom_get_key_value,
                .del = bsp_touchpad_custom_deinit,
            },
            .key = BSP_BUTTON_2_CH,
        }
    },
};



#define BSP_TOUCHPAD_RETRIES        (3)
#define BSP_TOUCHPAD_THRESH_RATIO   (0.015f)
#define BSP_TOUCHPAD_CHAN_COUNT     (2)

static bool bsp_touchpad_sens_initialized = false;
static touch_sensor_handle_t bsp_touchpad_sens_handle = NULL;
static touch_channel_handle_t bsp_touchpad_chan_handles[TOUCH_MAX_CHAN_ID] = { NULL };
static uint32_t bsp_touchpad_initial_values[TOUCH_MAX_CHAN_ID];

static esp_err_t bsp_touchpad_custom_init()
{
    if (bsp_touchpad_sens_initialized) {
        return ESP_OK;
    }

    /* Controller with default sample config */
    touch_sensor_sample_config_t sample_cfgs[] = {
        TOUCH_SENSOR_V2_DEFAULT_SAMPLE_CONFIG(500, TOUCH_VOLT_LIM_L_0V5, TOUCH_VOLT_LIM_H_2V2),
    };
    touch_sensor_config_t sens_cfg = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(TOUCH_SAMPLE_CFG_NUM, sample_cfgs);
    BSP_ERROR_CHECK_RETURN_ERR(touch_sensor_new_controller(&sens_cfg, &bsp_touchpad_sens_handle));

    /* Create channels with default config */
    touch_channel_config_t chan_cfg = {
        .active_thresh = {2000},
        .charge_speed = TOUCH_CHARGE_SPEED_7,
        .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
    };
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        if (bsp_button_config[i].type == BSP_BUTTON_TYPE_TOUCHPAD) {
            uint8_t chan = bsp_button_config[i].cfg.custom.key;
            BSP_ERROR_CHECK_RETURN_ERR(touch_sensor_new_channel(bsp_touchpad_sens_handle,
                                       chan, &chan_cfg, &bsp_touchpad_chan_handles[chan]));
        }
    }

    /* Default filter */
    touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();
    BSP_ERROR_CHECK_RETURN_ERR(touch_sensor_config_filter(bsp_touchpad_sens_handle, &filter_cfg));

    /* Enable the touch sensor to do the initial scanning, so that to initialize the channel data */
    BSP_ERROR_CHECK_RETURN_ERR(touch_sensor_enable(bsp_touchpad_sens_handle));

    /* Scan the enabled touch channels for several times, to make sure the initial channel data is stable */
    for (int i = 0; i < BSP_TOUCHPAD_RETRIES; i++) {
        ESP_ERROR_CHECK(touch_sensor_trigger_oneshot_scanning(bsp_touchpad_sens_handle, 2000));
    }

    /* Disable the touch channel to rollback the state */
    BSP_ERROR_CHECK_RETURN_ERR(touch_sensor_disable(bsp_touchpad_sens_handle));

    /* Initial read */
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        if (bsp_button_config[i].type == BSP_BUTTON_TYPE_TOUCHPAD) {
            uint8_t chan = bsp_button_config[i].cfg.custom.key;
            uint32_t data[TOUCH_SAMPLE_CFG_NUM];
            BSP_ERROR_CHECK_RETURN_ERR(touch_channel_read_data(bsp_touchpad_chan_handles[chan],
                                       TOUCH_CHAN_DATA_TYPE_RAW, data));
            bsp_touchpad_initial_values[chan] = data[0];
        }
    }

    /* Enable and start continuous scanning */
    BSP_ERROR_CHECK_RETURN_ERR(touch_sensor_enable(bsp_touchpad_sens_handle));
    BSP_ERROR_CHECK_RETURN_ERR(touch_sensor_start_continuous_scanning(bsp_touchpad_sens_handle));

    bsp_touchpad_sens_initialized = true;
    ESP_LOGI(TAG, "Touch sensor initialized.");
    return ESP_OK;
}

static esp_err_t  bsp_touchpad_custom_deinit(button_driver_t *button_driver)
{
    (void)button_driver;
    if (!bsp_touchpad_sens_initialized) {
        return ESP_OK;
    }
    bsp_touchpad_sens_initialized = false;

    touch_sensor_stop_continuous_scanning(bsp_touchpad_sens_handle);
    touch_sensor_disable(bsp_touchpad_sens_handle);
    for (int i = 0; i < TOUCH_MAX_CHAN_ID; i++) {
        if (bsp_touchpad_chan_handles[i] != NULL) {
            touch_sensor_del_channel(bsp_touchpad_chan_handles[i]);
            bsp_touchpad_chan_handles[i] = NULL;
        }
    }
    touch_sensor_del_controller(bsp_touchpad_sens_handle);
    bsp_touchpad_sens_handle = NULL;
    return ESP_OK;
}

static uint8_t bsp_touchpad_custom_get_key_value(button_driver_t *button_driver)
{
    /* Init, if not */
    bsp_touchpad_custom_init();

    bsp_button_type_touchpad_t *custom_btn = __containerof(button_driver, bsp_button_type_touchpad_t, base);
    int touchpad_ch = custom_btn->key;

    if (touchpad_ch < 0 || touchpad_ch > TOUCH_MAX_CHAN_ID || bsp_touchpad_chan_handles[touchpad_ch] == NULL) {
        return 0;  /* released */
    }

    uint32_t data[TOUCH_SAMPLE_CFG_NUM];
    if (touch_channel_read_data(bsp_touchpad_chan_handles[touchpad_ch], TOUCH_CHAN_DATA_TYPE_RAW, data) != ESP_OK) {
        return 0;
    }
    /* Read RAW value and compare with initial read */
    bool pressed = ((int)(data[0] - bsp_touchpad_initial_values[touchpad_ch]) > bsp_touchpad_initial_values[touchpad_ch] *
                    BSP_TOUCHPAD_THRESH_RATIO);
    return pressed ? 1 : 0;
}

esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size)
{

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
        } else if (bsp_button_config[i].type == BSP_BUTTON_TYPE_TOUCHPAD) {
            ret |= iot_button_create(&btn_config, &bsp_button_config[i].cfg.custom.base, &btn_array[i]);
        } else {
            ESP_LOGW(TAG, "Unsupported button type!");
        }

        if (btn_cnt) {
            (*btn_cnt)++;
        }
    }
    return ret;
}
