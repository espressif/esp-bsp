/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_lcd_touch.h"
#include "esp_lvgl_port.h"
#include "esp_timer.h"
#include "sdkconfig.h"

static const char *TAG = "LVGL";

/*******************************************************************************
* Types definitions
*******************************************************************************/

typedef struct {
    esp_lcd_touch_handle_t  handle;     /* LCD touch IO handle */
    lv_indev_t              *indev;     /* LVGL input device driver */
    struct {
        float x;
        float y;
    } scale;                            /* Touch scale */
} lvgl_port_touch_ctx_t;

/*******************************************************************************
* Function definitions
*******************************************************************************/

static void lvgl_port_touchpad_read(lv_indev_t *indev_drv, lv_indev_data_t *data);
static void lvgl_port_touch_interrupt_callback(esp_lcd_touch_handle_t tp);

/*******************************************************************************
* Public API functions
*******************************************************************************/

lv_indev_t *lvgl_port_add_touch(const lvgl_port_touch_cfg_t *touch_cfg)
{
    esp_err_t ret = ESP_OK;
    lv_indev_t *indev = NULL;
    assert(touch_cfg != NULL);
    assert(touch_cfg->disp != NULL);
    assert(touch_cfg->handle != NULL);

    /* Touch context */
    lvgl_port_touch_ctx_t *touch_ctx = malloc(sizeof(lvgl_port_touch_ctx_t));
    if (touch_ctx == NULL) {
        ESP_LOGE(TAG, "Not enough memory for touch context allocation!");
        return NULL;
    }
    touch_ctx->handle = touch_cfg->handle;
    touch_ctx->scale.x = (touch_cfg->scale.x ? touch_cfg->scale.x : 1);
    touch_ctx->scale.y = (touch_cfg->scale.y ? touch_cfg->scale.y : 1);

    if (touch_ctx->handle->config.int_gpio_num != GPIO_NUM_NC) {
        /* Register touch interrupt callback */
        ret = esp_lcd_touch_register_interrupt_callback_with_data(touch_ctx->handle, lvgl_port_touch_interrupt_callback, touch_ctx);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "Error in register touch interrupt.");
    }

    lvgl_port_lock(0);
    /* Register a touchpad input device */
    indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    /* Event mode can be set only, when touch interrupt enabled */
    if (touch_ctx->handle->config.int_gpio_num != GPIO_NUM_NC) {
        lv_indev_set_mode(indev, LV_INDEV_MODE_EVENT);
    }
    lv_indev_set_read_cb(indev, lvgl_port_touchpad_read);
    lv_indev_set_disp(indev, touch_cfg->disp);
    lv_indev_set_driver_data(indev, touch_ctx);
    touch_ctx->indev = indev;
    lvgl_port_unlock();

err:
    if (ret != ESP_OK) {
        if (touch_ctx) {
            free(touch_ctx);
        }
    }

    return indev;
}

esp_err_t lvgl_port_remove_touch(lv_indev_t *touch)
{
    assert(touch);
    lvgl_port_touch_ctx_t *touch_ctx = (lvgl_port_touch_ctx_t *)lv_indev_get_driver_data(touch);

    lvgl_port_lock(0);
    /* Remove input device driver */
    lv_indev_delete(touch);
    lvgl_port_unlock();

    if (touch_ctx->handle->config.int_gpio_num != GPIO_NUM_NC) {
        /* Unregister touch interrupt callback */
        esp_lcd_touch_register_interrupt_callback(touch_ctx->handle, NULL);
    }

    if (touch_ctx) {
        free(touch_ctx);
    }

    return ESP_OK;
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static void lvgl_port_touchpad_read(lv_indev_t *indev_drv, lv_indev_data_t *data)
{
    assert(indev_drv);
    lvgl_port_touch_ctx_t *touch_ctx = (lvgl_port_touch_ctx_t *)lv_indev_get_driver_data(indev_drv);
    assert(touch_ctx);
    assert(touch_ctx->handle);

    uint8_t touch_cnt = 0;
    esp_lcd_touch_point_data_t touch_data[CONFIG_ESP_LCD_TOUCH_MAX_POINTS] = {0};

    /* Read data from touch controller into memory */
    ESP_ERROR_CHECK(esp_lcd_touch_read_data(touch_ctx->handle));

    /* Read data from touch controller */
    ESP_ERROR_CHECK(esp_lcd_touch_get_data(touch_ctx->handle, touch_data, &touch_cnt, CONFIG_ESP_LCD_TOUCH_MAX_POINTS));

#if (CONFIG_ESP_LCD_TOUCH_MAX_POINTS > 1 && CONFIG_LV_USE_GESTURE_RECOGNITION)
    // Number of touch points which need to be constantly updated inside gesture recognizers
#define GESTURE_TOUCH_POINTS 2
#if GESTURE_TOUCH_POINTS > CONFIG_ESP_LCD_TOUCH_MAX_POINTS
#error "Number of touch point for gesture exceeds maximum number of aquired touch points"
#endif

    /* Initialize LVGL touch data for each activated touch point */
    lv_indev_touch_data_t touches[GESTURE_TOUCH_POINTS] = {0};

    for (int i = 0; i < touch_cnt && i < GESTURE_TOUCH_POINTS; i++) {
        touches[i].state = LV_INDEV_STATE_PRESSED;
        touches[i].point.x = touch_ctx->scale.x * touch_data[i].x;
        touches[i].point.y = touch_ctx->scale.y * touch_data[i].y;
        touches[i].id = touch_data[i].track_id;
        touches[i].timestamp = esp_timer_get_time() / 1000;
    }

    /* Pass touch data to LVGL gesture recognizers */
    lv_indev_gesture_recognizers_update(indev_drv, touches, GESTURE_TOUCH_POINTS);
    lv_indev_gesture_recognizers_set_data(indev_drv, data);

#endif

    if (touch_cnt > 0) {
        data->point.x = touch_ctx->scale.x * touch_data[0].x;
        data->point.y = touch_ctx->scale.y * touch_data[0].y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void IRAM_ATTR lvgl_port_touch_interrupt_callback(esp_lcd_touch_handle_t tp)
{
    lvgl_port_touch_ctx_t *touch_ctx = (lvgl_port_touch_ctx_t *) tp->config.user_data;

    /* Wake LVGL task, if needed */
    lvgl_port_task_wake(LVGL_PORT_EVENT_TOUCH, touch_ctx->indev);
}
