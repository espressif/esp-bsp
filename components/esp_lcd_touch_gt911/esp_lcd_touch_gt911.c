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

static const char *TAG = "GT911";

/* I2C timeout for read/write */
#define ESP_LCD_TOUCH_GT911_I2C_TIMEOUT_MS 10
/* I2C address of the GT911 controller */
#define ESP_LCD_TOUCH_GT911_I2C_ADDRESS (0x5D)
/* GT911 registers */
#define ESP_LCD_TOUCH_GT911_READ_XY_REG (0x814E)
#define ESP_LCD_TOUCH_GT911_CONFIG_REG  (0x8047)
#define ESP_LCD_TOUCH_GT911_PRODUCT_ID_REG (0x8140)

/*******************************************************************************
* Function definitions
*******************************************************************************/
static esp_err_t esp_lcd_touch_gt911_read_data(esp_lcd_touch_handle_t tp);
static bool esp_lcd_touch_gt911_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
static esp_err_t esp_lcd_touch_gt911_del(esp_lcd_touch_handle_t tp);

/* I2C read/write */
static esp_err_t touch_gt911_i2c_read(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len);
static esp_err_t touch_gt911_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t data);

/* GT911 reset */
static esp_err_t touch_gt911_reset(esp_lcd_touch_handle_t tp);
/* Read status and config register */
static void touch_gt911_read_cfg(esp_lcd_touch_handle_t tp);

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t esp_lcd_touch_new_i2c_gt911(const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch)
{
    esp_err_t ret = ESP_OK;

    assert(config != NULL);
    assert(out_touch != NULL);

    /* Prepare main structure */
    esp_lcd_touch_handle_t esp_lcd_touch_gt911 = heap_caps_calloc(1, sizeof(esp_lcd_touch_t), MALLOC_CAP_DEFAULT);
    ESP_GOTO_ON_FALSE(esp_lcd_touch_gt911, ESP_ERR_NO_MEM, err, TAG, "no mem for GT911 controller");

    /* Only supported callbacks are set */
    esp_lcd_touch_gt911->read_data = esp_lcd_touch_gt911_read_data;
    esp_lcd_touch_gt911->get_xy = esp_lcd_touch_gt911_get_xy;
    esp_lcd_touch_gt911->del = esp_lcd_touch_gt911_del;

    /* Mutex */
    esp_lcd_touch_gt911->data.lock.owner = portMUX_FREE_VAL;

    /* Save config */
    memcpy(&esp_lcd_touch_gt911->config, config, sizeof(esp_lcd_touch_config_t));

    /* Check I2C number */
    if (esp_lcd_touch_gt911->config.device.i2c.port < 0 || esp_lcd_touch_gt911->config.device.i2c.port >= SOC_I2C_NUM) {
        ESP_LOGE(TAG, "Bad I2C number!");
        ret = ESP_ERR_INVALID_ARG;
        goto err;
    }

    /* Prepare pin for touch interrupt */
    if (esp_lcd_touch_gt911->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_gt911->config.int_gpio_num)
        };
        ret = gpio_config(&int_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

    /* Prepare pin for touch controller reset */
    if (esp_lcd_touch_gt911->config.rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_gpio_config = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(esp_lcd_touch_gt911->config.rst_gpio_num)
        };
        ret = gpio_config(&rst_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

    /* Reset controller */
    ret = touch_gt911_reset(esp_lcd_touch_gt911);
    ESP_GOTO_ON_ERROR(ret, err, TAG, "GT911 reset failed");

    /* Read status and config info */
    touch_gt911_read_cfg(esp_lcd_touch_gt911);

err:
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (0x%x)! Touch controller GT911 initialization failed!", ret);
        if (esp_lcd_touch_gt911) {
            esp_lcd_touch_gt911_del(esp_lcd_touch_gt911);
        }
    }

    *out_touch = esp_lcd_touch_gt911;

    return ret;
}

static esp_err_t esp_lcd_touch_gt911_read_data(esp_lcd_touch_handle_t tp)
{
    esp_err_t err;
    uint8_t buf[41];
    uint8_t touch_cnt = 0;
    uint8_t clear = 0;
    size_t i = 0;

    assert(tp != NULL);

    err = touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, buf, 1);
    ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

    /* Any touch data? */
    if ((buf[0] & 0x80) == 0x00) {
        touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, clear);
    } else {
        /* Count of touched points */
        touch_cnt = buf[0] & 0x0f;
        if (touch_cnt > 5 || touch_cnt == 0) {
            touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, clear);
            return ESP_OK;
        }

        /* Read all points */
        err = touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG + 1, &buf[1], touch_cnt * 8);
        ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

        /* Clear all */
        err = touch_gt911_i2c_write(tp, ESP_LCD_TOUCH_GT911_READ_XY_REG, clear);
        ESP_RETURN_ON_ERROR(err, TAG, "I2C read error!");

        taskENTER_CRITICAL(&tp->data.lock);

        /* Number of touched points */
        touch_cnt = (touch_cnt > CONFIG_ESP_LCD_TOUCH_MAX_POINTS ? CONFIG_ESP_LCD_TOUCH_MAX_POINTS : touch_cnt);
        tp->data.points = touch_cnt;

        /* Fill all coordinates */
        for (i = 0; i < touch_cnt; i++) {
            tp->data.coords[i].x = ((uint16_t)buf[(i * 8) + 3] << 8) + buf[(i * 8) + 2];
            tp->data.coords[i].y = (((uint16_t)buf[(i * 8) + 5] << 8) + buf[(i * 8) + 4]);
            tp->data.coords[i].strength = (((uint16_t)buf[(i * 8) + 7] << 8) + buf[(i * 8) + 6]);
        }

        taskEXIT_CRITICAL(&tp->data.lock);
    }

    return ESP_OK;
}

static bool esp_lcd_touch_gt911_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
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

static esp_err_t esp_lcd_touch_gt911_del(esp_lcd_touch_handle_t tp)
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
static esp_err_t touch_gt911_reset(esp_lcd_touch_handle_t tp)
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

static void touch_gt911_read_cfg(esp_lcd_touch_handle_t tp)
{
    uint8_t buf[4];

    assert(tp != NULL);

    touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_PRODUCT_ID_REG, (uint8_t *)&buf[0], 3);
    touch_gt911_i2c_read(tp, ESP_LCD_TOUCH_GT911_CONFIG_REG, (uint8_t *)&buf[3], 1);

    ESP_LOGI(TAG, "TouchPad_ID:0x%02x,0x%02x,0x%02x", buf[0], buf[1], buf[2]);
    ESP_LOGI(TAG, "TouchPad_Config_Version:%d", buf[3]);
}

static esp_err_t touch_gt911_i2c_read(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t *data, uint8_t len)
{
    esp_lcd_touch_dev_i2c_t *gt911_dev = NULL;
    uint8_t reg_buf[2] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF)};

    assert(tp != NULL);
    assert(data != NULL);

    gt911_dev = &tp->config.device.i2c;
    assert(gt911_dev->port >= 0 && gt911_dev->port < SOC_I2C_NUM);

    /* Write register number */
    ESP_RETURN_ON_ERROR(i2c_master_write_to_device(gt911_dev->port, ESP_LCD_TOUCH_GT911_I2C_ADDRESS, reg_buf, sizeof(reg_buf), ESP_LCD_TOUCH_GT911_I2C_TIMEOUT_MS / portTICK_PERIOD_MS), TAG, "I2C write error!");

    /* Read data */
    return i2c_master_read_from_device(gt911_dev->port, ESP_LCD_TOUCH_GT911_I2C_ADDRESS, data, len, ESP_LCD_TOUCH_GT911_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t touch_gt911_i2c_write(esp_lcd_touch_handle_t tp, uint16_t reg, uint8_t data)
{
    esp_lcd_touch_dev_i2c_t *gt911_dev = NULL;
    uint8_t reg_buf[3] = {(uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF), data};

    assert(tp != NULL);

    gt911_dev = &tp->config.device.i2c;
    assert(gt911_dev->port >= 0 && gt911_dev->port < SOC_I2C_NUM);

    return i2c_master_write_to_device(gt911_dev->port, ESP_LCD_TOUCH_GT911_I2C_ADDRESS, reg_buf, sizeof(reg_buf), ESP_LCD_TOUCH_GT911_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}
