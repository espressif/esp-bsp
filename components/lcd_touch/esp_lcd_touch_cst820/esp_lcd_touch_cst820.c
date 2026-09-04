/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"

#define CST820_TOUCH_REPORT_REG         (0x00)
#define CST820_TOUCH_REPORT_SIZE        (15)
#define CST820_TOUCH_COUNT_OFFSET       (2)
#define CST820_FIRST_POINT_OFFSET       (3)
#define CST820_CHIP_ID_REG              (0xA7)
#define CST820_MAX_TOUCH_POINTS         (2)
#define CST820_TOUCH_RECORD_SIZE        (6)
#define CST820_TOUCH_COUNT_MASK         (0x0F)
#define CST820_COORDINATE_HIGH_MASK     (0x0F)
#define CST820_TRACK_ID_SHIFT           (4)

static const char *TAG = "cst820";

static esp_err_t esp_lcd_touch_cst820_read_data(esp_lcd_touch_handle_t tp);
static bool esp_lcd_touch_cst820_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y,
                                        uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
static esp_err_t esp_lcd_touch_cst820_get_track_id(esp_lcd_touch_handle_t tp, uint8_t *track_id,
        uint8_t max_point_num);
static esp_err_t esp_lcd_touch_cst820_del(esp_lcd_touch_handle_t tp);

static esp_err_t touch_cst820_i2c_read(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t *data, size_t len);
static esp_err_t touch_cst820_i2c_write(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t data);
static esp_err_t touch_cst820_reset(esp_lcd_touch_handle_t tp);
static esp_err_t touch_cst820_wake_by_reset(esp_lcd_touch_handle_t tp);
#ifndef CONFIG_ESP_LCD_TOUCH_CST820_DISABLE_READ_ID
static esp_err_t touch_cst820_read_id(esp_lcd_touch_handle_t tp);
#endif
static void touch_cst820_clear_points(esp_lcd_touch_handle_t tp);

esp_err_t esp_lcd_touch_new_i2c_cst820(const esp_lcd_panel_io_handle_t io,
                                       const esp_lcd_touch_config_t *config,
                                       esp_lcd_touch_handle_t *tp)
{
    ESP_RETURN_ON_FALSE(io != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller IO handle can't be NULL");
    ESP_RETURN_ON_FALSE(config != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Pointer to the touch controller configuration can't be NULL");
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Pointer to the touch controller handle can't be NULL");

    esp_err_t ret = ESP_OK;
    esp_lcd_touch_handle_t cst820 = calloc(1, sizeof(esp_lcd_touch_t));
    ESP_GOTO_ON_FALSE(cst820 != NULL, ESP_ERR_NO_MEM, err, TAG, "Touch handle allocation failed");

    cst820->io = io;
    cst820->read_data = esp_lcd_touch_cst820_read_data;
    cst820->get_xy = esp_lcd_touch_cst820_get_xy;
    cst820->get_track_id = esp_lcd_touch_cst820_get_track_id;
    cst820->del = esp_lcd_touch_cst820_del;
    cst820->data.lock.owner = portMUX_FREE_VAL;
    memcpy(&cst820->config, config, sizeof(esp_lcd_touch_config_t));

    if (cst820->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .pin_bit_mask = BIT64(cst820->config.int_gpio_num),
            .mode = GPIO_MODE_INPUT,
            .intr_type = cst820->config.levels.interrupt ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&int_gpio_config), err, TAG, "Interrupt GPIO configuration failed");

        if (cst820->config.interrupt_callback != NULL) {
            ESP_GOTO_ON_ERROR(esp_lcd_touch_register_interrupt_callback(cst820,
                              cst820->config.interrupt_callback), err, TAG,
                              "Interrupt callback registration failed");
        }
    }

    if (cst820->config.rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t rst_gpio_config = {
            .pin_bit_mask = BIT64(cst820->config.rst_gpio_num),
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&rst_gpio_config), err, TAG, "Reset GPIO configuration failed");
    }

    ESP_GOTO_ON_ERROR(touch_cst820_reset(cst820), err, TAG, "Controller reset failed");

#ifdef CONFIG_ESP_LCD_TOUCH_CST820_DISABLE_READ_ID
    ESP_LOGI(TAG, "Controller ID read disabled");
#else
    ESP_GOTO_ON_ERROR(touch_cst820_read_id(cst820), err, TAG, "Controller ID read failed");
#endif

    *tp = cst820;
    return ESP_OK;

err:
    if (cst820 != NULL) {
        esp_lcd_touch_cst820_del(cst820);
    }
    return ret;
}

static esp_err_t esp_lcd_touch_cst820_read_data(esp_lcd_touch_handle_t tp)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    /*
     * CST820 supplier firmware exposes one 15-byte report at 0x00. Byte 2 is
     * the point count and the two coordinate slots begin at bytes 3 and 9.
     * Read that native report directly; do not probe or alias CST816 layouts.
     */
    uint8_t data[CST820_TOUCH_REPORT_SIZE] = {0};
    ESP_RETURN_ON_ERROR(touch_cst820_i2c_read(tp, CST820_TOUCH_REPORT_REG,
                        data, sizeof(data)), TAG, "Touch report read failed");
    uint8_t point_num = data[CST820_TOUCH_COUNT_OFFSET] & CST820_TOUCH_COUNT_MASK;

    if (point_num == 0) {
        touch_cst820_clear_points(tp);
        return ESP_OK;
    }
    if (point_num > CST820_MAX_TOUCH_POINTS) {
        touch_cst820_clear_points(tp);
        ESP_LOGW(TAG, "Ignoring invalid touch count: %u", point_num);
        return ESP_OK;
    }

    point_num = point_num > CONFIG_ESP_LCD_TOUCH_MAX_POINTS ? CONFIG_ESP_LCD_TOUCH_MAX_POINTS : point_num;

    portENTER_CRITICAL(&tp->data.lock);
    tp->data.points = point_num;
    for (size_t i = 0; i < point_num; i++) {
        const size_t offset = CST820_FIRST_POINT_OFFSET + i * CST820_TOUCH_RECORD_SIZE;
        tp->data.coords[i].x = ((uint16_t)(data[offset] & CST820_COORDINATE_HIGH_MASK) << 8) |
                               data[offset + 1];
        tp->data.coords[i].y = ((uint16_t)(data[offset + 2] & CST820_COORDINATE_HIGH_MASK) << 8) |
                               data[offset + 3];
        tp->data.coords[i].track_id = data[offset + 2] >> CST820_TRACK_ID_SHIFT;
        tp->data.coords[i].strength = 0;
    }
    portEXIT_CRITICAL(&tp->data.lock);

    return ESP_OK;
}

static bool esp_lcd_touch_cst820_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y,
                                        uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    ESP_RETURN_ON_FALSE(tp != NULL, false, TAG, "Touch controller handle can't be NULL");
    ESP_RETURN_ON_FALSE(x != NULL, false, TAG, "Pointer to the X coordinate array can't be NULL");
    ESP_RETURN_ON_FALSE(y != NULL, false, TAG, "Pointer to the Y coordinate array can't be NULL");
    ESP_RETURN_ON_FALSE(point_num != NULL, false, TAG, "Pointer to the touch count can't be NULL");
    ESP_RETURN_ON_FALSE(max_point_num > 0, false, TAG, "Coordinate array must contain at least one element");

    portENTER_CRITICAL(&tp->data.lock);
    *point_num = tp->data.points > max_point_num ? max_point_num : tp->data.points;
    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;
        if (strength != NULL) {
            strength[i] = tp->data.coords[i].strength;
        }
    }
    tp->data.points = 0;
    portEXIT_CRITICAL(&tp->data.lock);

    return *point_num > 0;
}

static esp_err_t esp_lcd_touch_cst820_get_track_id(esp_lcd_touch_handle_t tp, uint8_t *track_id,
        uint8_t max_point_num)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");
    ESP_RETURN_ON_FALSE(track_id != NULL, ESP_ERR_INVALID_ARG, TAG,
                        "Pointer to the track ID array can't be NULL");
    ESP_RETURN_ON_FALSE(max_point_num > 0, ESP_ERR_INVALID_ARG, TAG,
                        "Track ID array must contain at least one element");

    /* tp->data.coords is sized by CONFIG_ESP_LCD_TOUCH_MAX_POINTS; clamp the
     * caller-supplied array size so it cannot read past the internal buffer. */
    const uint8_t point_num = max_point_num > CONFIG_ESP_LCD_TOUCH_MAX_POINTS ?
                              CONFIG_ESP_LCD_TOUCH_MAX_POINTS : max_point_num;

    portENTER_CRITICAL(&tp->data.lock);
    for (size_t i = 0; i < point_num; i++) {
        track_id[i] = tp->data.coords[i].track_id;
    }
    portEXIT_CRITICAL(&tp->data.lock);

    return ESP_OK;
}


esp_err_t esp_lcd_touch_cst820_enter_monitor_mode(esp_lcd_touch_handle_t tp)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    /*
     * Standby tier (~10 uA): the controller keeps scanning at a low
     * frequency and returns to dynamic mode on touch, pulsing INT so the
     * host can be woken by the panel. No undocumented register write is sent
     * here on purpose: per the datasheet the controller drops into standby
     * automatically 2 s after the last touch, and that path is the one that
     * is guaranteed to keep INT wake alive.
     */
    touch_cst820_clear_points(tp);

    ESP_LOGI(TAG, "Monitor mode armed: controller reaches standby <=2 s after the last touch, INT wakes the host");
    return ESP_OK;
}

esp_err_t esp_lcd_touch_cst820_exit_monitor_mode(esp_lcd_touch_handle_t tp)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    /* A touch already exits standby on its own; this forces dynamic mode
     * without waiting for one, using the datasheet reset path. */
    return touch_cst820_wake_by_reset(tp);
}

esp_err_t esp_lcd_touch_cst820_read_reg(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t *data, size_t len)
{
    return touch_cst820_i2c_read(tp, reg, data, len);
}

esp_err_t esp_lcd_touch_cst820_write_reg(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t data)
{
    return touch_cst820_i2c_write(tp, reg, data);
}

static esp_err_t esp_lcd_touch_cst820_del(esp_lcd_touch_handle_t tp)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        if (tp->config.interrupt_callback != NULL) {
            gpio_isr_handler_remove(tp->config.int_gpio_num);
        }
        gpio_reset_pin(tp->config.int_gpio_num);
    }
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }

    free(tp);
    return ESP_OK;
}

static esp_err_t touch_cst820_reset(esp_lcd_touch_handle_t tp)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    if (tp->config.rst_gpio_num == GPIO_NUM_NC) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, tp->config.levels.reset),
                        TAG, "Failed to assert reset");
    /* Reset pulse width: a busy-wait keeps it independent of the FreeRTOS
     * tick rate (pdMS_TO_TICKS(5) degenerates to 0 at the default 100 Hz). */
    esp_rom_delay_us(5000);
    ESP_RETURN_ON_ERROR(gpio_set_level(tp->config.rst_gpio_num, !tp->config.levels.reset),
                        TAG, "Failed to release reset");
    /* Tron (100 ms), rounded up with one tick of phase margin. */
    vTaskDelay(pdMS_TO_TICKS(100) + 1);

    return ESP_OK;
}


static esp_err_t touch_cst820_wake_by_reset(esp_lcd_touch_handle_t tp)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    /* The datasheet names reset as the way out of standby; this forces the
     * controller back to dynamic mode without waiting for a touch. */
    ESP_RETURN_ON_FALSE(tp->config.rst_gpio_num != GPIO_NUM_NC, ESP_ERR_NOT_SUPPORTED, TAG,
                        "Reset GPIO is required to wake the controller");
    ESP_RETURN_ON_ERROR(touch_cst820_reset(tp), TAG, "Wake-up reset failed");
    touch_cst820_clear_points(tp);
#ifndef CONFIG_ESP_LCD_TOUCH_CST820_DISABLE_READ_ID
    /* Confirm the controller is back on the bus before reporting success. */
    ESP_RETURN_ON_ERROR(touch_cst820_read_id(tp), TAG, "Controller ID read after wake-up failed");
#endif

    return ESP_OK;
}

#ifndef CONFIG_ESP_LCD_TOUCH_CST820_DISABLE_READ_ID
static esp_err_t touch_cst820_read_id(esp_lcd_touch_handle_t tp)
{
    uint8_t id = 0;
    ESP_RETURN_ON_ERROR(touch_cst820_i2c_read(tp, CST820_CHIP_ID_REG, &id, sizeof(id)),
                        TAG, "ID register read failed");
    ESP_LOGI(TAG, "Controller ID: 0x%02X", id);
    return ESP_OK;
}
#endif

static esp_err_t touch_cst820_i2c_read(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");
    ESP_RETURN_ON_FALSE(data != NULL, ESP_ERR_INVALID_ARG, TAG, "Read buffer can't be NULL");
    ESP_RETURN_ON_FALSE(len > 0, ESP_ERR_INVALID_SIZE, TAG, "Read size must be greater than zero");

    return esp_lcd_panel_io_rx_param(tp->io, reg, data, len);
}

static esp_err_t touch_cst820_i2c_write(esp_lcd_touch_handle_t tp, uint8_t reg, uint8_t data)
{
    ESP_RETURN_ON_FALSE(tp != NULL, ESP_ERR_INVALID_ARG, TAG, "Touch controller handle can't be NULL");

    return esp_lcd_panel_io_tx_param(tp->io, reg, (uint8_t[]) {
        data
    }, 1);
}

static void touch_cst820_clear_points(esp_lcd_touch_handle_t tp)
{
    portENTER_CRITICAL(&tp->data.lock);
    tp->data.points = 0;
    portEXIT_CRITICAL(&tp->data.lock);
}
