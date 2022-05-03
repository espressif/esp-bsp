/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_touch.h"

static const char *TAG = "TT21100";

/* I2C timeout for read/write */
#define ESP_LCD_TOUCH_TT21100_I2C_TIMEOUT_MS 10
/* I2C address of the TT21100 controller */
#define ESP_LCD_TOUCH_TT21100_I2C_ADDRESS (0x24)

#define ESP_LCD_TOUCH_TT21100_MAX_DATA_LEN  (7+CONFIG_ESP_LCD_TOUCH_MAX_POINTS*10) /* 7 Header + (Points * 10 data bytes) */

/*******************************************************************************
* Function definitions
*******************************************************************************/
static esp_err_t esp_lcd_touch_tt21100_read_data(esp_lcd_touch_handle_t tp);
static bool esp_lcd_touch_tt21100_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
static esp_err_t esp_lcd_touch_tt21100_get_button_state(esp_lcd_touch_handle_t tp, uint8_t n, uint8_t *state);
#endif
static esp_err_t esp_lcd_touch_tt21100_del(esp_lcd_touch_handle_t tp);

/* I2C read */
static esp_err_t touch_tt21100_i2c_read(esp_lcd_touch_handle_t tp, uint8_t *data, uint8_t len);

/* TT21100 reset */
static esp_err_t touch_tt21100_reset(esp_lcd_touch_handle_t tp);

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t esp_lcd_touch_new_i2c_tt21100(const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch)
{
    esp_err_t ret = ESP_OK;

    assert(config != NULL);
    assert(out_touch != NULL);

    /* Prepare main structure */
    esp_lcd_touch_handle_t esp_lcd_touch_tt21100 = heap_caps_calloc(1, sizeof(esp_lcd_touch_t), MALLOC_CAP_DEFAULT);
    ESP_GOTO_ON_FALSE(esp_lcd_touch_tt21100, ESP_ERR_NO_MEM, err, TAG, "no mem for TT21100 controller");

    /* Only supported callbacks are set */
    esp_lcd_touch_tt21100->read_data = esp_lcd_touch_tt21100_read_data;
    esp_lcd_touch_tt21100->get_xy = esp_lcd_touch_tt21100_get_xy;
#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
    esp_lcd_touch_tt21100->get_button_state = esp_lcd_touch_tt21100_get_button_state;
#endif
    esp_lcd_touch_tt21100->del = esp_lcd_touch_tt21100_del;

    /* Mutex */
    esp_lcd_touch_tt21100->data.lock.owner = portMUX_FREE_VAL;

    /* Save config */
    memcpy(&esp_lcd_touch_tt21100->config, config, sizeof(esp_lcd_touch_config_t));

    /* Check I2C number */
    if (esp_lcd_touch_tt21100->config.device.i2c.port < 0 || esp_lcd_touch_tt21100->config.device.i2c.port >= SOC_I2C_NUM) {
        ESP_LOGE(TAG, "Bad I2C number!");
        ret = ESP_ERR_INVALID_ARG;
        goto err;
    }

    /* Prepare pin for touch interrupt */
    if (esp_lcd_touch_tt21100->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_tt21100->config.int_gpio_num)
        };
        ret = gpio_config(&int_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

    /* Prepare pin for touch controller reset */
    if (esp_lcd_touch_tt21100->config.rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_tt21100->config.rst_gpio_num)
        };
        ret = gpio_config(&rst_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

    /* Reset controller */
    ret = touch_tt21100_reset(esp_lcd_touch_tt21100);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "TT21100 reset failed");

err:
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (0x%x)! Touch controller TT21100 initialization failed!", ret);
        if (esp_lcd_touch_tt21100) {
            esp_lcd_touch_tt21100_del(esp_lcd_touch_tt21100);
        }
    }

    *out_touch = esp_lcd_touch_tt21100;

    return ret;
}

static esp_err_t esp_lcd_touch_tt21100_read_data(esp_lcd_touch_handle_t tp)
{
    typedef struct {
        uint8_t : 5;
        uint8_t touch_type: 3;
        uint8_t tip: 1;
        uint8_t event_id: 2;
        uint8_t touch_id: 5;
        uint16_t x;
        uint16_t y;
        uint8_t pressure;
        uint16_t major_axis_length;
        uint8_t orientation;
    } __attribute__((packed)) touch_record_struct_t;

    typedef struct {
        uint16_t data_len;
        uint8_t report_id;
        uint16_t time_stamp;
        uint8_t : 2;
        uint8_t large_object : 1;
        uint8_t record_num : 5;
        uint8_t report_counter: 2;
        uint8_t : 3;
        uint8_t noise_efect: 3;
        touch_record_struct_t touch_record[0];
    } __attribute__((packed)) touch_report_struct_t;

#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
    typedef struct {
        uint16_t length;        /*!< Always 14(0x000E) */
        uint8_t report_id;      /*!< Always 03h */
        uint16_t time_stamp;    /*!< Number in units of 100 us */
        uint8_t btn_val;        /*!< Only use bit[0..3] */
        uint16_t btn_signal[4];
    } __attribute__((packed)) button_record_struct_t;
#endif

    touch_report_struct_t *p_report_data = NULL;
    touch_record_struct_t *p_touch_data = NULL;

#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
    button_record_struct_t *p_btn_data = NULL;
#endif

    esp_err_t err;
    static uint16_t data_len;
    static uint8_t data[ESP_LCD_TOUCH_TT21100_MAX_DATA_LEN];
    uint8_t tp_num = 0;
    size_t i = 0;

    assert(tp != NULL);

    /* Get report data length */
    err = touch_tt21100_i2c_read(tp, (uint8_t *)&data_len, sizeof(data_len));
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    /* Read report data if length */
    if (data_len > 0 && data_len < sizeof(data)) {
        touch_tt21100_i2c_read(tp, data, data_len);
        ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

        taskENTER_CRITICAL(&tp->data.lock);

        if (data_len == 14) {
#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
            /* Button event */
            p_btn_data = (button_record_struct_t *) data;

            /* Buttons count */
            tp->data.buttons = (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS < 4 ? CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS : 4);

            for (i = 0; i < tp->data.buttons; i++) {
                tp->data.button[i].status = p_btn_data->btn_signal[i];
            }

            ESP_LOGD(TAG, "Len : %04Xh. ID : %02Xh. Time : %5u. Val : [%u] - [%04X][%04X][%04X][%04X]",
                     p_btn_data->length, p_btn_data->report_id, p_btn_data->time_stamp, p_btn_data->btn_val,
                     p_btn_data->btn_signal[0], p_btn_data->btn_signal[1], p_btn_data->btn_signal[2], p_btn_data->btn_signal[3]);
#endif
        } else if (data_len >= 7) {
            /* Touch point event */
            p_report_data = (touch_report_struct_t *) data;
            tp_num = (data_len - sizeof(touch_report_struct_t)) / sizeof(touch_record_struct_t);

            /* Number of touched points */
            tp_num = (tp_num > CONFIG_ESP_LCD_TOUCH_MAX_POINTS ? CONFIG_ESP_LCD_TOUCH_MAX_POINTS : tp_num);
            tp->data.points = tp_num;

            /* Fill all coordinates */
            for (i = 0; i < tp_num; i++) {
                p_touch_data = &p_report_data->touch_record[i];

                tp->data.coords[i].x = p_touch_data->x;
                tp->data.coords[i].y = p_touch_data->y;
                tp->data.coords[i].strength = p_touch_data->pressure;

                ESP_LOGD(TAG, "(%zu) [%3u][%3u]", i, p_touch_data->x, p_touch_data->y);
            }
        }

        taskEXIT_CRITICAL(&tp->data.lock);
    }

    return ESP_OK;
}

static bool esp_lcd_touch_tt21100_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    assert(tp != NULL);
    assert(x != NULL);
    assert(y != NULL);
    assert(point_num != NULL);
    assert(max_point_num > 0);

    taskENTER_CRITICAL(&tp->data.lock);

    /* Count of points */
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);

    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength) {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    /* Invalidate */
    tp->data.points = 0;

    taskEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

#if (CONFIG_ESP_LCD_TOUCH_MAX_BUTTONS > 0)
static esp_err_t esp_lcd_touch_tt21100_get_button_state(esp_lcd_touch_handle_t tp, uint8_t n, uint8_t *state)
{
    esp_err_t err = ESP_OK;
    assert(tp != NULL);
    assert(state != NULL);

    *state = 0;

    taskENTER_CRITICAL(&tp->data.lock);

    if (n > tp->data.buttons) {
        err = ESP_ERR_INVALID_ARG;
    } else {
        *state = tp->data.button[n].status;
    }

    taskEXIT_CRITICAL(&tp->data.lock);

    return err;
}
#endif

static esp_err_t esp_lcd_touch_tt21100_del(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    /* Reset GPIO pin settings */
    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
    }

    /* Reset GPIO pin settings */
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }

    free(tp);

    return ESP_OK;
}

/*******************************************************************************
* Private API function
*******************************************************************************/

/* Reset controller */
static esp_err_t touch_tt21100_reset(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, !tp->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, tp->config.levels.reset), TAG, "GPIO set level error!");
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return ESP_OK;
}

static esp_err_t touch_tt21100_i2c_read(esp_lcd_touch_handle_t tp, uint8_t *data, uint8_t len)
{
    esp_lcd_touch_dev_i2c_t *tt21100_dev = NULL;

    assert(tp != NULL);
    assert(data != NULL);

    tt21100_dev = &tp->config.device.i2c;
    assert(tt21100_dev->port >= 0 && tt21100_dev->port < SOC_I2C_NUM);

    /* Read data */
    return i2c_master_read_from_device(tt21100_dev->port, ESP_LCD_TOUCH_TT21100_I2C_ADDRESS, data, len, ESP_LCD_TOUCH_TT21100_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}
