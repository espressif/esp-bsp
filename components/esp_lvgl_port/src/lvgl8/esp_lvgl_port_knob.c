/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_lvgl_port.h"

static const char *TAG = "LVGL";

/*******************************************************************************
* Types definitions
*******************************************************************************/

typedef struct {
    knob_handle_t   knob_handle; /* Encoder knob handlers */
    button_handle_t btn_handle; /* Encoder button handlers */
    lv_indev_drv_t  indev_drv;  /* LVGL input device driver */
    bool btn_enter;     /* Encoder button enter state */
    int32_t diff;       /* Encoder diff */
} lvgl_port_encoder_ctx_t;

/*******************************************************************************
* Function definitions
*******************************************************************************/

static void lvgl_port_encoder_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data);
static void lvgl_port_encoder_btn_down_handler(void *arg, void *arg2);
static void lvgl_port_encoder_btn_up_handler(void *arg, void *arg2);
static void lvgl_port_encoder_left_handler(void *arg, void *arg2);
static void lvgl_port_encoder_right_handler(void *arg, void *arg2);
static int32_t lvgl_port_calculate_diff(knob_handle_t knob, knob_event_t event);

/*******************************************************************************
* Public API functions
*******************************************************************************/

lv_indev_t *lvgl_port_add_encoder(const lvgl_port_encoder_cfg_t *encoder_cfg)
{
    esp_err_t ret = ESP_OK;
    lv_indev_t *indev = NULL;
    assert(encoder_cfg != NULL);
    assert(encoder_cfg->disp != NULL);

    /* Encoder context */
    lvgl_port_encoder_ctx_t *encoder_ctx = malloc(sizeof(lvgl_port_encoder_ctx_t));
    if (encoder_ctx == NULL) {
        ESP_LOGE(TAG, "Not enough memory for encoder context allocation!");
        return NULL;
    }

    /* Encoder_a/b */
    if (encoder_cfg->encoder_a_b != NULL) {
        encoder_ctx->knob_handle = iot_knob_create(encoder_cfg->encoder_a_b);
        ESP_GOTO_ON_FALSE(encoder_ctx->knob_handle, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for knob create!");
    }

    ESP_ERROR_CHECK(iot_knob_register_cb(encoder_ctx->knob_handle, KNOB_LEFT, lvgl_port_encoder_left_handler, encoder_ctx));
    ESP_ERROR_CHECK(iot_knob_register_cb(encoder_ctx->knob_handle, KNOB_RIGHT, lvgl_port_encoder_right_handler, encoder_ctx));

    /* Encoder Enter */
    if (encoder_cfg->encoder_enter != NULL) {
        encoder_ctx->btn_handle = iot_button_create(encoder_cfg->encoder_enter);
        ESP_GOTO_ON_FALSE(encoder_ctx->btn_handle, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for button create!");
    }

    ESP_ERROR_CHECK(iot_button_register_cb(encoder_ctx->btn_handle, BUTTON_PRESS_DOWN, lvgl_port_encoder_btn_down_handler, encoder_ctx));
    ESP_ERROR_CHECK(iot_button_register_cb(encoder_ctx->btn_handle, BUTTON_PRESS_UP, lvgl_port_encoder_btn_up_handler, encoder_ctx));

    encoder_ctx->btn_enter = false;
    encoder_ctx->diff = 0;

    /* Register a encoder input device */
    lv_indev_drv_init(&encoder_ctx->indev_drv);
    encoder_ctx->indev_drv.type = LV_INDEV_TYPE_ENCODER;
    encoder_ctx->indev_drv.disp = encoder_cfg->disp;
    encoder_ctx->indev_drv.read_cb = lvgl_port_encoder_read;
    encoder_ctx->indev_drv.user_data = encoder_ctx;
    indev = lv_indev_drv_register(&encoder_ctx->indev_drv);

err:
    if (ret != ESP_OK) {
        if (encoder_ctx->knob_handle != NULL) {
            iot_knob_delete(encoder_ctx->knob_handle);
        }

        if (encoder_ctx->btn_handle != NULL) {
            iot_button_delete(encoder_ctx->btn_handle);
        }

        if (encoder_ctx != NULL) {
            free(encoder_ctx);
        }
    }
    return indev;
}

esp_err_t lvgl_port_remove_encoder(lv_indev_t *encoder)
{
    assert(encoder);
    lv_indev_drv_t *indev_drv = encoder->driver;
    assert(indev_drv);
    lvgl_port_encoder_ctx_t *encoder_ctx = (lvgl_port_encoder_ctx_t *)indev_drv->user_data;

    if (encoder_ctx->knob_handle != NULL) {
        iot_knob_delete(encoder_ctx->knob_handle);
    }

    if (encoder_ctx->btn_handle != NULL) {
        iot_button_delete(encoder_ctx->btn_handle);
    }

    if (encoder_ctx != NULL) {
        free(encoder_ctx);
    }

    return ESP_OK;
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static void lvgl_port_encoder_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    assert(indev_drv);
    lvgl_port_encoder_ctx_t *ctx = (lvgl_port_encoder_ctx_t *)indev_drv->user_data;
    assert(ctx);

    data->enc_diff = ctx->diff;
    data->state = (true == ctx->btn_enter) ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    ctx->diff = 0;
}

static void lvgl_port_encoder_btn_down_handler(void *arg, void *arg2)
{
    lvgl_port_encoder_ctx_t *ctx = (lvgl_port_encoder_ctx_t *) arg2;
    button_handle_t button = (button_handle_t)arg;
    if (ctx && button) {
        /* ENTER */
        if (button == ctx->btn_handle) {
            ctx->btn_enter = true;
        }
    }
}

static void lvgl_port_encoder_btn_up_handler(void *arg, void *arg2)
{
    lvgl_port_encoder_ctx_t *ctx = (lvgl_port_encoder_ctx_t *) arg2;
    button_handle_t button = (button_handle_t)arg;
    if (ctx && button) {
        /* ENTER */
        if (button == ctx->btn_handle) {
            ctx->btn_enter = false;
        }
    }
}

static void lvgl_port_encoder_left_handler(void *arg, void *arg2)
{
    lvgl_port_encoder_ctx_t *ctx = (lvgl_port_encoder_ctx_t *) arg2;
    knob_handle_t knob = (knob_handle_t)arg;
    if (ctx && knob) {
        /* LEFT */
        if (knob == ctx->knob_handle) {
            ctx->diff += lvgl_port_calculate_diff(knob, KNOB_LEFT);
        }
    }
}

static void lvgl_port_encoder_right_handler(void *arg, void *arg2)
{
    lvgl_port_encoder_ctx_t *ctx = (lvgl_port_encoder_ctx_t *) arg2;
    knob_handle_t knob = (knob_handle_t)arg;
    if (ctx && knob) {
        /* RIGHT */
        if (knob == ctx->knob_handle) {
            ctx->diff += lvgl_port_calculate_diff(knob, KNOB_RIGHT);
        }
    }
}

static int32_t lvgl_port_calculate_diff(knob_handle_t knob, knob_event_t event)
{
    static int32_t last_v = 0;

    int32_t diff = 0;
    int32_t invd = iot_knob_get_count_value(knob);

    if (last_v ^ invd) {

        diff = (int32_t)((uint32_t)invd - (uint32_t)last_v);
        diff += (event == KNOB_RIGHT && invd < last_v) ? CONFIG_KNOB_HIGH_LIMIT :
                (event == KNOB_LEFT && invd > last_v) ? CONFIG_KNOB_LOW_LIMIT : 0;
        last_v = invd;
    }

    return diff;
}
