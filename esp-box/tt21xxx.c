/**
 * @file tt21xxx.c
 * @brief
 * @version 0.1
 * @date 2021-09-06
 *
 * @copyright Copyright 2021 Espressif Systems (Shanghai) Co. Ltd.
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *
 *               http://www.apache.org/licenses/LICENSE-2.0
 *
 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 */

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tt21xxx.h"

#define TT21xxx_CHIP_ADDR_DEFAULT   (0x24)
#define TT21xxx_REG_TP_NUM          (0x1)
#define TT21xxx_REG_X_POS           (0x2)
#define TT21xxx_REG_Y_POS           (0x3)

static const char *TAG = "tt21xxx";

typedef struct {
    i2c_port_t port;
    gpio_num_t rst_pin;
    gpio_num_t int_pin;
    uint8_t tp_num, btn_val;
    uint16_t x, y, btn_signal;
} tt21xxx_dev_t;

static esp_err_t tt21xxx_read(i2c_port_t port, void *data, size_t data_len)
{
    return i2c_master_read_from_device(port, TT21xxx_CHIP_ADDR_DEFAULT, data, data_len, pdMS_TO_TICKS(1000));
}

esp_err_t tt21xxx_create(tt21xxx_handle_t *tp, const i2c_port_t port, const tt21xxx_config_t *config)
{
    // Probe that TP is active on I2C bus
    uint8_t dummy[2];
    ESP_RETURN_ON_ERROR(tt21xxx_read(port, &dummy, sizeof(dummy)), TAG, "Device not answering on I2C bus");

    // Allocate memory and save configuration
    tt21xxx_dev_t *dev = calloc(1, sizeof(tt21xxx_dev_t));
    if (!dev) {
        return ESP_ERR_NO_MEM;
    }
    dev->rst_pin = config->rst_pin;
    dev->int_pin = config->int_pin;
    dev->port = port;

    // Setup Interrupt pin (if used)
    if (dev->int_pin != GPIO_NUM_NC) {
        const gpio_config_t io_conf_key = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = BIT64(dev->int_pin),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf_key));
    }

    // Setup Reset pin (if used)
    if (dev->rst_pin != GPIO_NUM_NC) {
        const gpio_config_t io_conf_key = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(dev->rst_pin),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io_conf_key));
    }

    // Wait for chip boot up
    uint16_t reg_val = 0;
    do {
        ESP_ERROR_CHECK(tt21xxx_read(dev->port, &reg_val, sizeof(reg_val)));
        vTaskDelay(pdMS_TO_TICKS(20));
    } while (0x0002 != reg_val);

    *tp = (tt21xxx_handle_t)dev;
    return ESP_OK;
}

void tt21xxx_delete(tt21xxx_handle_t tp)
{
    tt21xxx_dev_t *dev = (tt21xxx_dev_t *) tp;
    if (dev->rst_pin != GPIO_NUM_NC) {
        const gpio_config_t io_conf_key = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = BIT64(dev->rst_pin),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_ENABLE,
        };
        gpio_config(&io_conf_key);
    }
    free(dev);
}

esp_err_t tt21xxx_tp_read(tt21xxx_handle_t tp)
{
    tt21xxx_dev_t *dev = (tt21xxx_dev_t *) tp;
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

    typedef struct {
        uint16_t length;        /*!< Always 14(0x000E) */
        uint8_t report_id;      /*!< Always 03h */
        uint16_t time_stamp;    /*!< Number in units of 100 us */
        uint8_t btn_val;        /*!< Only use bit[0..3] */
        uint16_t btn_signal[4];
    } __attribute__((packed)) button_record_struct_t;

    touch_report_struct_t *p_report_data = NULL;
    touch_record_struct_t *p_touch_data = NULL;
    button_record_struct_t *p_btn_data = NULL;

    static uint16_t data_len;
    static uint8_t data[256];
    esp_err_t ret_val = ESP_OK;

    /* Get report data length */
    ret_val |= tt21xxx_read(dev->port, &data_len, sizeof(data_len));
    ESP_LOGD(TAG, "Data len : %u", data_len);

    /* Read report data if length */
    if (data_len < 0xff) {
        tt21xxx_read(dev->port, data, data_len);
        switch (data_len) {
        case 2:     /* No avaliable data*/
            break;
        case 7:
        case 17:
        case 27:
            p_report_data = (touch_report_struct_t *) data;
            p_touch_data = &p_report_data->touch_record[0];
            dev->x = p_touch_data->x;
            dev->y = p_touch_data->y;

            dev->tp_num = (data_len - sizeof(touch_report_struct_t)) / sizeof(touch_record_struct_t);
            for (size_t i = 0; i < dev->tp_num; i++) {
                p_touch_data = &p_report_data->touch_record[i];
                ESP_LOGD(TAG, "(%zu) [%3u][%3u]", i, p_touch_data->x, p_touch_data->y);
            }
            break;
        case 14:    /* Button event */
            p_btn_data = (button_record_struct_t *) data;
            dev->btn_val = p_btn_data->btn_val;
            dev->btn_signal = p_btn_data->btn_signal[0];
            ESP_LOGD(TAG, "Len : %04Xh. ID : %02Xh. Time : %5u. Val : [%u] - [%04X][%04X][%04X][%04X]",
                     p_btn_data->length, p_btn_data->report_id, p_btn_data->time_stamp, p_btn_data->btn_val,
                     p_btn_data->btn_signal[0], p_btn_data->btn_signal[1], p_btn_data->btn_signal[2], p_btn_data->btn_signal[3]);
            break;
        default:
            break;
        }
    } else {
        return ESP_FAIL;
    }

    return ret_val;
}
//@todo parameter checks
esp_err_t tt21xxx_get_touch_point(tt21xxx_handle_t tp, uint8_t *p_tp_num, uint16_t *p_x, uint16_t *p_y)
{
    tt21xxx_dev_t *dev = (tt21xxx_dev_t *) tp;
    *p_x = dev->x;
    *p_y = dev->y;
    *p_tp_num = dev->tp_num;

    return ESP_OK;
}

esp_err_t tt21xxx_get_btn_val(tt21xxx_handle_t tp, uint8_t *p_btn_val, uint16_t *p_btn_signal)
{
    tt21xxx_dev_t *dev = (tt21xxx_dev_t *) tp;
    *p_btn_val = dev->btn_val;
    *p_btn_signal = dev->btn_signal;

    return ESP_OK;
}

bool tt21xxx_data_avaliable(tt21xxx_handle_t tp)
{
    tt21xxx_dev_t *dev = (tt21xxx_dev_t *) tp;

    int level = gpio_get_level(dev->int_pin);
    if (level) {
        return false;
    }

    return true;
}
