/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"

#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
#include "esp_lcd_mipi_dsi.h"
#endif
#if CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "esp_lcd_panel_rgb.h"
#endif

#if (ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 4, 4)) || (ESP_IDF_VERSION == ESP_IDF_VERSION_VAL(5, 0, 0))
#define LVGL_PORT_HANDLE_FLUSH_READY 0
#else
#define LVGL_PORT_HANDLE_FLUSH_READY 1
#endif

static const char *TAG = "LVGL";

/*******************************************************************************
* Types definitions
*******************************************************************************/

typedef struct {
    esp_lcd_panel_io_handle_t io_handle;      /* LCD panel IO handle */
    esp_lcd_panel_handle_t    panel_handle;   /* LCD panel handle */
    esp_lcd_panel_handle_t    control_handle; /* LCD panel control handle */
    lvgl_port_rotation_cfg_t  rotation;       /* Default values of the screen rotation */
    lv_color16_t              *draw_buffs[2]; /* Display draw buffers */
    lv_display_t              *disp_drv;      /* LVGL display driver */
    struct {
        unsigned int monochrome: 1;  /* True, if display is monochrome and using 1bit for 1px */
        unsigned int swap_bytes: 1;  /* Swap bytes in RGB656 (16-bit) before send to LCD driver */
        unsigned int full_refresh: 1;   /*!< 1: Always make the whole screen redrawn */
        unsigned int direct_mode: 1;    /*!< 1: Use screen-sized buffers and draw to absolute coordinates */
        unsigned int flush_byself: 1;
    } flags;
} lvgl_port_display_ctx_t;

/*******************************************************************************
* Function definitions
*******************************************************************************/

#if LVGL_PORT_HANDLE_FLUSH_READY
static bool lvgl_port_flush_io_ready_callback(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
static bool lvgl_port_flush_panel_ready_callback(esp_lcd_panel_handle_t panel_io, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx);
#endif
#if CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static bool lvgl_port_flush_vsync_ready_callback(esp_lcd_panel_handle_t panel_io, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx);
#endif
#endif
static void lvgl_port_flush_callback(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map);
static void lvgl_port_disp_size_update_callback(lv_event_t *e);

/*******************************************************************************
* Public API functions
*******************************************************************************/

lv_display_t *lvgl_port_add_disp(const lvgl_port_display_cfg_t *disp_cfg)
{
    esp_err_t ret = ESP_OK;
    lv_display_t *disp = NULL;
    lv_color16_t *buf1 = NULL;
    lv_color16_t *buf2 = NULL;
    assert(disp_cfg != NULL);
    assert(disp_cfg->panel_handle != NULL);
    assert(disp_cfg->buffer_size > 0);
    assert(disp_cfg->hres > 0);
    assert(disp_cfg->vres > 0);

    /* Display context */
    lvgl_port_display_ctx_t *disp_ctx = malloc(sizeof(lvgl_port_display_ctx_t));
    ESP_GOTO_ON_FALSE(disp_ctx, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for display context allocation!");
    memset(disp_ctx, 0, sizeof(lvgl_port_display_ctx_t));
    disp_ctx->io_handle = disp_cfg->io_handle;
    disp_ctx->panel_handle = disp_cfg->panel_handle;
    disp_ctx->control_handle = disp_cfg->control_handle;
    disp_ctx->rotation.swap_xy = disp_cfg->rotation.swap_xy;
    disp_ctx->rotation.mirror_x = disp_cfg->rotation.mirror_x;
    disp_ctx->rotation.mirror_y = disp_cfg->rotation.mirror_y;
    disp_ctx->flags.swap_bytes = disp_cfg->flags.swap_bytes;

    if (disp_cfg->user_buf1 || disp_cfg->user_buf2) {
        buf1 = disp_cfg->user_buf1;
        buf2 = disp_cfg->user_buf2;
    } else {
        uint32_t buff_caps = MALLOC_CAP_DEFAULT;
        if (disp_cfg->flags.buff_dma && disp_cfg->flags.buff_spiram) {
            ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "Alloc DMA capable buffer in SPIRAM is not supported!");
        } else if (disp_cfg->flags.buff_dma) {
            buff_caps = MALLOC_CAP_DMA;
        } else if (disp_cfg->flags.buff_spiram) {
            buff_caps = MALLOC_CAP_SPIRAM;
        }

        /* alloc draw buffers used by LVGL */
        /* it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized */
        buf1 = heap_caps_malloc(disp_cfg->buffer_size * sizeof(lv_color16_t), buff_caps);
        ESP_GOTO_ON_FALSE(buf1, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for LVGL buffer (buf1) allocation!");
        if (disp_cfg->double_buffer) {
            buf2 = heap_caps_malloc(disp_cfg->buffer_size * sizeof(lv_color16_t), buff_caps);
            ESP_GOTO_ON_FALSE(buf2, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for LVGL buffer (buf2) allocation!");
        }
    }

    disp_ctx->draw_buffs[0] = buf1;
    disp_ctx->draw_buffs[1] = buf2;

    lvgl_port_lock(0);
    disp = lv_display_create(disp_cfg->hres, disp_cfg->vres);

    /* Monochrome display settings */
    if (disp_cfg->monochrome) {
        /* When using monochromatic display, there must be used full bufer! */
        ESP_GOTO_ON_FALSE((disp_cfg->hres * disp_cfg->vres == disp_cfg->buffer_size), ESP_ERR_INVALID_ARG, err, TAG, "Monochromatic display must using full buffer!");

        disp_ctx->flags.monochrome = 1;

        //lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
        lv_display_set_buffers(disp, buf1, buf2, disp_cfg->buffer_size * sizeof(lv_color16_t), LV_DISPLAY_RENDER_MODE_FULL);
    } else if (disp_cfg->flags.direct_mode) {
        /* When using direct_mode, there must be used full bufer! */
        ESP_GOTO_ON_FALSE((disp_cfg->hres * disp_cfg->vres == disp_cfg->buffer_size), ESP_ERR_INVALID_ARG, err, TAG, "Direct_mode must using full buffer!");

        disp_ctx->flags.direct_mode = disp_cfg->flags.direct_mode;

        lv_display_set_buffers(disp, buf1, buf2, disp_cfg->buffer_size * sizeof(lv_color16_t), LV_DISPLAY_RENDER_MODE_DIRECT);
    } else if (disp_cfg->flags.full_refresh) {
        /* When using full_refresh, there must be used full bufer! */
        ESP_GOTO_ON_FALSE((disp_cfg->hres * disp_cfg->vres == disp_cfg->buffer_size), ESP_ERR_INVALID_ARG, err, TAG, "Full_fresh must using full buffer!");

        disp_ctx->flags.full_refresh = disp_cfg->flags.full_refresh;

        lv_display_set_buffers(disp, buf1, buf2, disp_cfg->buffer_size * sizeof(lv_color16_t), LV_DISPLAY_RENDER_MODE_FULL);
    } else {
        lv_display_set_buffers(disp, buf1, buf2, disp_cfg->buffer_size * sizeof(lv_color16_t), LV_DISPLAY_RENDER_MODE_PARTIAL);
    }

    lv_display_set_flush_cb(disp, lvgl_port_flush_callback);
    lv_display_add_event_cb(disp, lvgl_port_disp_size_update_callback, LV_EVENT_RESOLUTION_CHANGED, disp_ctx);

    lv_display_set_user_data(disp, disp_ctx);
    disp_ctx->disp_drv = disp;

#if LVGL_PORT_HANDLE_FLUSH_READY
    if (disp_cfg->mipi_dsi) {
#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
        /* Register done callback (MIPI-DSI) */
        const esp_lcd_dpi_panel_event_callbacks_t cbs = {
            .on_color_trans_done = lvgl_port_flush_panel_ready_callback,
        };
        esp_lcd_dpi_panel_register_event_callbacks(disp_ctx->panel_handle, &cbs, disp_ctx->disp_drv);
#else
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "MIPI-DSI is supported only on ESP32P4 and from IDF 5.3!");
#endif
    } else if (disp_cfg->RGB) {
        disp_ctx->flags.flush_byself = true;
#if (CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0))
        /* Register done callback */
        const esp_lcd_rgb_panel_event_callbacks_t vsync_cbs = {
            .on_vsync = lvgl_port_flush_vsync_ready_callback,
        };

        const esp_lcd_rgb_panel_event_callbacks_t bb_cbs = {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 2)
            .on_bounce_frame_finish = lvgl_port_flush_vsync_ready_callback,
#endif
        };

        if (disp_cfg->flags.bb_mode && (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 2))) {
            ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(disp_ctx->panel_handle, &bb_cbs, &disp_ctx->disp_drv));
        } else {
            ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(disp_ctx->panel_handle, &vsync_cbs, &disp_ctx->disp_drv));
        }
#else
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "RGB is supported only on ESP32S3 and from IDF 5.0!");
#endif
    } else {
        /* Register done callback */
        const esp_lcd_panel_io_callbacks_t cbs = {
            .on_color_trans_done = lvgl_port_flush_io_ready_callback,
        };
        assert(disp_ctx->io_handle != NULL);
        esp_lcd_panel_io_register_event_callbacks(disp_ctx->io_handle, &cbs, disp_ctx->disp_drv);
    }
#endif

    lvgl_port_unlock();

err:
    if (ret != ESP_OK) {
        if (buf1) {
            free(buf1);
        }
        if (buf2) {
            free(buf2);
        }
        if (disp_ctx) {
            free(disp_ctx);
        }
    }

    return disp;
}

esp_err_t lvgl_port_remove_disp(lv_display_t *disp)
{
    assert(disp);
    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)lv_display_get_user_data(disp);

    lvgl_port_lock(0);
    lv_disp_remove(disp);
    lvgl_port_unlock();

    if (disp_ctx->draw_buffs[0]) {
        free(disp_ctx->draw_buffs[0]);
    }

    if (disp_ctx->draw_buffs[1]) {
        free(disp_ctx->draw_buffs[1]);
    }

    free(disp_ctx);

    return ESP_OK;
}

void lvgl_port_flush_ready(lv_display_t *disp)
{
    assert(disp);
    lv_disp_flush_ready(disp);
}

/*******************************************************************************
* Private functions
*******************************************************************************/

#if LVGL_PORT_HANDLE_FLUSH_READY
static bool lvgl_port_flush_io_ready_callback(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp_drv = (lv_display_t *)user_ctx;
    assert(disp_drv != NULL);
    lv_disp_flush_ready(disp_drv);
    return false;
}
#if (CONFIG_IDF_TARGET_ESP32P4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
static bool lvgl_port_flush_panel_ready_callback(esp_lcd_panel_handle_t panel_io, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp_drv = (lv_display_t *)user_ctx;
    assert(disp_drv != NULL);
    lv_disp_flush_ready(disp_drv);
    return false;
}
#endif
#if CONFIG_IDF_TARGET_ESP32S3 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static bool lvgl_port_flush_vsync_ready_callback(esp_lcd_panel_handle_t panel_io, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    BaseType_t need_yield = pdFALSE;

    lv_display_t *disp_drv = (lv_display_t *)user_ctx;
    assert(disp_drv != NULL);
    // esp_rom_printf(DRAM_STR("end:%d\r\n"), esp_log_timestamp());
    need_yield = lvgl_port_task_notify();

    return (need_yield == pdTRUE);
}
#endif
#endif

static void _lvgl_port_draw_sw_rgb565_swap(uint8_t *color_map, int x1, int y1, int x2, int y2)
{
    uint16_t *from = (uint16_t *)color_map;

    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            uint16_t color = from[y * LV_HOR_RES + x];

            uint8_t *data = (uint8_t*)&color;
            uint8_t temp = data[0];
            data[0] = data[1];
            data[1] = temp;
            from[y * LV_HOR_RES + x] = color;
        }
    }
}

static void _lvgl_port_transform_monochrome(lv_display_t *display, const lv_area_t *area, uint8_t *color_map)
{
    uint8_t *buf = color_map;
    lv_color16_t *color = (lv_color16_t *)color_map;
    uint16_t hor_res = lv_display_get_physical_horizontal_resolution(display);
    uint16_t ver_res = lv_display_get_physical_vertical_resolution(display);
    uint16_t res = hor_res;
    bool swap_xy = (lv_display_get_rotation(display) == LV_DISPLAY_ROTATION_90 || lv_display_get_rotation(display) == LV_DISPLAY_ROTATION_270);

    int x1 = area->x1;
    int x2 = area->x2;
    int y1 = area->y1;
    int y2 = area->y2;

    int out_x, out_y;
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            bool chroma_color = (color[hor_res * y + x].blue > 16);

            if (swap_xy) {
                out_x = y;
                out_y = x;
                res = ver_res;
            } else {
                out_x = x;
                out_y = y;
                res = hor_res;
            }

            /* Write to the buffer as required for the display.
            * It writes only 1-bit for monochrome displays mapped vertically.*/
            buf = color_map + res * (out_y >> 3) + (out_x);
            if (chroma_color) {
                (*buf) &= ~(1 << (out_y % 8));
            } else {
                (*buf) |= (1 << (out_y % 8));
            }
        }
    }
}

static void lvgl_port_flush_callback(lv_display_t *drv, const lv_area_t *area, uint8_t *color_map)
{
    assert(drv != NULL);
    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)lv_display_get_user_data(drv);
    assert(disp_ctx != NULL);

    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

    //TODO: try to use SPI_SWAP_DATA_RX from https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32s3/api-reference/peripherals/spi_master.html#c.SPI_SWAP_DATA_TX
    if (disp_ctx->flags.swap_bytes) {
        if ((disp_ctx->flags.direct_mode) || (disp_ctx->flags.full_refresh)) {
            _lvgl_port_draw_sw_rgb565_swap(color_map, offsetx1, offsety1, offsetx2, offsety2);
        } else {
            size_t len = lv_area_get_size(area);
            lv_draw_sw_rgb565_swap(color_map, len);
        }
    }

    /* Transfor data in buffer for monochromatic screen */
    if (disp_ctx->flags.monochrome) {
        _lvgl_port_transform_monochrome(drv, area, color_map);
    }

    if ((disp_ctx->flags.direct_mode || disp_ctx->flags.full_refresh)) {
        if (lv_disp_flush_is_last(drv)) {
            /* If the interface is I80 or SPI, this step cannot be used for drawing. */
            esp_lcd_panel_draw_bitmap(disp_ctx->panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
            /* Waiting for the last frame buffer to complete transmission */
            ulTaskNotifyValueClear(NULL, ULONG_MAX);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    } else {
        esp_lcd_panel_draw_bitmap(disp_ctx->panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    }

    if (disp_ctx->flags.flush_byself) {
        lv_disp_flush_ready(drv);
    }
}

static void lvgl_port_disp_size_update_callback(lv_event_t *e)
{
    assert(e);
    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)e->user_data;
    assert(disp_ctx != NULL);
    esp_lcd_panel_handle_t control_handle = (disp_ctx->control_handle ? disp_ctx->control_handle : disp_ctx->panel_handle);

    /* Solve rotation screen and touch */
    switch (lv_display_get_rotation(disp_ctx->disp_drv)) {
    case LV_DISPLAY_ROTATION_0:
        /* Rotate LCD display */
        esp_lcd_panel_swap_xy(control_handle, disp_ctx->rotation.swap_xy);
        esp_lcd_panel_mirror(control_handle, disp_ctx->rotation.mirror_x, disp_ctx->rotation.mirror_y);
        break;
    case LV_DISPLAY_ROTATION_90:
        /* Rotate LCD display */
        esp_lcd_panel_swap_xy(control_handle, !disp_ctx->rotation.swap_xy);
        if (disp_ctx->rotation.swap_xy) {
            esp_lcd_panel_mirror(control_handle, !disp_ctx->rotation.mirror_x, disp_ctx->rotation.mirror_y);
        } else {
            esp_lcd_panel_mirror(control_handle, disp_ctx->rotation.mirror_x, !disp_ctx->rotation.mirror_y);
        }
        break;
    case LV_DISPLAY_ROTATION_180:
        /* Rotate LCD display */
        esp_lcd_panel_swap_xy(control_handle, disp_ctx->rotation.swap_xy);
        esp_lcd_panel_mirror(control_handle, !disp_ctx->rotation.mirror_x, !disp_ctx->rotation.mirror_y);
        break;
    case LV_DISPLAY_ROTATION_270:
        /* Rotate LCD display */
        esp_lcd_panel_swap_xy(control_handle, !disp_ctx->rotation.swap_xy);
        if (disp_ctx->rotation.swap_xy) {
            esp_lcd_panel_mirror(control_handle, disp_ctx->rotation.mirror_x, !disp_ctx->rotation.mirror_y);
        } else {
            esp_lcd_panel_mirror(control_handle, !disp_ctx->rotation.mirror_x, disp_ctx->rotation.mirror_y);
        }
        break;
    }
}
