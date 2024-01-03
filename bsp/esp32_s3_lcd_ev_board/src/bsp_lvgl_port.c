/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_timer.h"
#include "esp_lcd_touch.h"
#include "lvgl.h"

#include "bsp_err_check.h"
#include "bsp/display.h"
#include "bsp/esp32_s3_lcd_ev_board.h"

static const char *TAG = "bsp_lvgl_port";

#if CONFIG_BSP_DISPLAY_LVGL_ROTATION_DEGREE != 0
static void *get_next_frame_buffer(esp_lcd_panel_handle_t panel_handle)
{
    static void *next_fb = NULL;
    static void *fb[2] = { NULL };
    if (next_fb == NULL) {
        BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &fb[0], &fb[1]));
        next_fb = fb[1];
    } else {
        next_fb = (next_fb == fb[0]) ? fb[1] : fb[0];
    }
    return next_fb;
}

IRAM_ATTR static void rotate_copy_pixel(const uint16_t *from, uint16_t *to, uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t w, uint16_t h, lv_disp_rot_t rotate)
{
    int from_index = 0;

    int to_bytes_per_line;
    int to_index = 0;
    int to_index_const = 0;

    switch (rotate) {
    case LV_DISP_ROT_90:
        to_index_const = (w - x_start - 1) * h;
        for (int from_y = y_start; from_y < y_end + 1; from_y++) {
            from_index = from_y * w + x_start;
            to_index = to_index_const + from_y;
            for (int from_x = x_start; from_x < x_end + 1; from_x++) {
                *(to + to_index) = *(from + from_index);
                from_index += 1;
                to_index -= h;
            }
        }
        break;
    case LV_DISP_ROT_180:
        to_index_const = h * w - x_start - 1;
        for (int from_y = y_start; from_y < y_end + 1; from_y++) {
            from_index = from_y * w + x_start;
            to_index = to_index_const - from_y * w;
            for (int from_x = x_start; from_x < x_end + 1; from_x++) {
                *(to + to_index) = *(from + from_index);
                from_index += 1;
                to_index -= 1;
            }
        }
        break;
    case LV_DISP_ROT_270:
        to_index_const = (x_start + 1) * h - 1;
        for (int from_y = y_start; from_y < y_end + 1; from_y++) {
            from_index = from_y * w + x_start;
            to_index = to_index_const - from_y;
            for (int from_x = x_start; from_x < x_end + 1; from_x++) {
                *(to + to_index) = *(from + from_index);
                from_index += 1;
                to_index += h;
            }
        }
        break;
    default:
        break;
    }
}
#endif /* CONFIG_BSP_DISPLAY_LVGL_ROTATION_DEGREE */

#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
#if CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
typedef struct {
    uint16_t inv_p;
    uint8_t inv_area_joined[LV_INV_BUF_SIZE];
    lv_area_t inv_areas[LV_INV_BUF_SIZE];
} lv_port_dirty_area_t;

static lv_port_dirty_area_t dirty_area;

static void flush_dirty_save(lv_port_dirty_area_t *dirty_area)
{
    lv_disp_t *disp = _lv_refr_get_disp_refreshing();
    dirty_area->inv_p = disp->inv_p;
    for (int i = 0; i < disp->inv_p; i++) {
        dirty_area->inv_area_joined[i] = disp->inv_area_joined[i];
        dirty_area->inv_areas[i] = disp->inv_areas[i];
    }
}

typedef enum {
    FLUSH_STATUS_PART,
    FLUSH_STATUS_FULL
} lv_port_flush_status_t;

typedef enum {
    FLUSH_PROBE_PART_COPY,
    FLUSH_PROBE_SKIP_COPY,
    FLUSH_PROBE_FULL_COPY,
} lv_port_flush_probe_t;

/**
 * @brief Probe dirty area to copy
 *
 * @note This function is used to avoid tearing effect, and only work with LVGL direct-mode.
 *
 */
static lv_port_flush_probe_t flush_copy_probe(lv_disp_drv_t *drv)
{
    static lv_port_flush_status_t prev_status = FLUSH_PROBE_PART_COPY;
    lv_port_flush_status_t cur_status;
    uint8_t probe_result;
    lv_disp_t *disp_refr = _lv_refr_get_disp_refreshing();

    uint32_t flush_ver = 0;
    uint32_t flush_hor = 0;
    for (int i = 0; i < disp_refr->inv_p; i++) {
        if (disp_refr->inv_area_joined[i] == 0) {
            flush_ver = (disp_refr->inv_areas[i].y2 + 1 - disp_refr->inv_areas[i].y1);
            flush_hor = (disp_refr->inv_areas[i].x2 + 1 - disp_refr->inv_areas[i].x1);
            break;
        }
    }
    /* Check if the current full screen refreshes */
    cur_status = ((flush_ver == drv->ver_res) && (flush_hor == drv->hor_res)) ? (FLUSH_STATUS_FULL) : (FLUSH_STATUS_PART);

    if (prev_status == FLUSH_STATUS_FULL) {
        if ((cur_status == FLUSH_STATUS_PART)) {
            probe_result = FLUSH_PROBE_FULL_COPY;
        } else {
            probe_result = FLUSH_PROBE_SKIP_COPY;
        }
    } else {
        probe_result = FLUSH_PROBE_PART_COPY;
    }
    prev_status = cur_status;

    return probe_result;
}

#if CONFIG_BSP_DISPLAY_LVGL_ROTATION_DEGREE != 0
static inline void *flush_get_next_buf(void *panel_handle)
{
    return get_next_frame_buffer(panel_handle);
}

/**
 * @brief Copy dirty area
 *
 * @note This function is used to avoid tearing effect, and only work with LVGL direct-mode.
 *
 */
static void flush_dirty_copy(void *dst, void *src, lv_port_dirty_area_t *dirty_area)
{
    lv_coord_t x_start, x_end, y_start, y_end;
    for (int i = 0; i < dirty_area->inv_p; i++) {
        /* Refresh the unjoined areas*/
        if (dirty_area->inv_area_joined[i] == 0) {
            x_start = dirty_area->inv_areas[i].x1;
            x_end = dirty_area->inv_areas[i].x2;
            y_start = dirty_area->inv_areas[i].y1;
            y_end = dirty_area->inv_areas[i].y2;

            rotate_copy_pixel(src, dst, x_start, y_start, x_end, y_end, LV_HOR_RES, LV_VER_RES, CONFIG_BSP_DISPLAY_LVGL_ROTATION_DEGREE);
        }
    }
}

static void flush_callback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)drv->user_data;
    esp_lcd_panel_handle_t panel_handle = disp_ctx->panel_handle;
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;
    void *next_fb = NULL;
    lv_port_flush_probe_t probe_result = FLUSH_PROBE_PART_COPY;

    /* Action after last area refresh */
    if (lv_disp_flush_is_last(drv)) {
        /* Check if the `full_refresh` flag has been triggered */
        if (drv->full_refresh) {
            /* Reset flag */
            drv->full_refresh = 0;

            // Roate and copy data from the whole screen LVGL's buffer to the next frame buffer
            next_fb = flush_get_next_buf(panel_handle);
            rotate_copy_pixel((uint16_t *)color_map, next_fb, offsetx1, offsety1, offsetx2, offsety2, LV_HOR_RES, LV_VER_RES, CONFIG_BSP_DISPLAY_LVGL_ROTATION_DEGREE);

            /* Switch the current RGB frame buffer to `next_fb` */
            esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, next_fb);

            /* Waiting for the current frame buffer to complete transmission */
            ulTaskNotifyValueClear(NULL, ULONG_MAX);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            /* Synchronously update the dirty area for another frame buffer */
            flush_dirty_copy(flush_get_next_buf(panel_handle), color_map, &dirty_area);
            flush_get_next_buf(panel_handle);
        } else {
            /* Probe the copy method for the current dirty area */
            probe_result = flush_copy_probe(drv);

            if (probe_result == FLUSH_PROBE_FULL_COPY) {
                /* Save current dirty area for next frame buffer */
                flush_dirty_save(&dirty_area);

                /* Set LVGL full-refresh flag and set flush ready in advance */
                drv->full_refresh = 1;
                lv_disp_flush_ready(drv);

                /* Force to refresh whole screen, and will invoke `flush_callback` recursively */
                lv_refr_now(_lv_refr_get_disp_refreshing());
            } else {
                /* Update current dirty area for next frame buffer */
                next_fb = flush_get_next_buf(panel_handle);
                flush_dirty_save(&dirty_area);
                flush_dirty_copy(next_fb, color_map, &dirty_area);

                /* Switch the current RGB frame buffer to `next_fb` */
                esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, next_fb);

                /* Waiting for the current frame buffer to complete transmission */
                ulTaskNotifyValueClear(NULL, ULONG_MAX);
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                if (probe_result == FLUSH_PROBE_PART_COPY) {
                    /* Synchronously update the dirty area for another frame buffer */
                    flush_dirty_save(&dirty_area);
                    flush_dirty_copy(flush_get_next_buf(panel_handle), color_map, &dirty_area);
                    flush_get_next_buf(panel_handle);
                }
            }
        }
    }

    lv_disp_flush_ready(drv);
}
#else
static inline void *flush_get_next_buf(void *buf)
{
    lv_disp_t *disp = _lv_refr_get_disp_refreshing();
    lv_disp_draw_buf_t *draw_buf = disp->driver->draw_buf;
    return (buf == draw_buf->buf1) ? draw_buf->buf2 : draw_buf->buf1;
}

/**
 * @brief Copy dirty area
 *
 * @note This function is used to avoid tearing effect, and only work with LVGL direct-mode.
 *
 */
static void flush_dirty_copy(void *dst, void *src, lv_port_dirty_area_t *dirty_area)
{
    lv_coord_t x_start, x_end, y_start, y_end;
    uint32_t copy_bytes_per_line;
    uint32_t h_res = LV_HOR_RES;
    uint32_t bytes_per_line = h_res * 2;
    uint8_t *from, *to;
    for (int i = 0; i < dirty_area->inv_p; i++) {
        /* Refresh the unjoined areas*/
        if (dirty_area->inv_area_joined[i] == 0) {
            x_start = dirty_area->inv_areas[i].x1;
            x_end = dirty_area->inv_areas[i].x2 + 1;
            y_start = dirty_area->inv_areas[i].y1;
            y_end = dirty_area->inv_areas[i].y2 + 1;

            copy_bytes_per_line = (x_end - x_start) * 2;
            from = src + (y_start * h_res + x_start) * 2;
            to = dst + (y_start * h_res + x_start) * 2;
            for (int y = y_start; y < y_end; y++) {
                memcpy(to, from, copy_bytes_per_line);
                from += bytes_per_line;
                to += bytes_per_line;
            }
        }
    }
}

static void flush_callback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)drv->user_data;
    esp_lcd_panel_handle_t panel_handle = disp_ctx->panel_handle;
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

    lv_port_flush_probe_t probe_result;
    /* Action after last area refresh */
    if (lv_disp_flush_is_last(drv)) {
        /* Check if the `full_refresh` flag has been triggered */
        if (drv->full_refresh) {
            /* Reset flag */
            drv->full_refresh = 0;

            /* Switch the current RGB frame buffer to `color_map` */
            esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);

            /* Waiting for the last frame buffer to complete transmission */
            ulTaskNotifyValueClear(NULL, ULONG_MAX);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            /* Synchronously update the dirty area for another frame buffer */
            flush_dirty_copy(flush_get_next_buf(color_map), color_map, &dirty_area);
            drv->draw_buf->buf_act = (color_map == drv->draw_buf->buf1) ? drv->draw_buf->buf2 : drv->draw_buf->buf1;
        } else {
            /* Probe the copy method for the current dirty area */
            probe_result = flush_copy_probe(drv);

            if (probe_result == FLUSH_PROBE_FULL_COPY) {
                /* Save current dirty area for next frame buffer */
                flush_dirty_save(&dirty_area);

                /* Set LVGL full-refresh flag and set flush ready in advance */
                drv->full_refresh = 1;
                lv_disp_flush_ready(drv);

                /* Force to refresh whole screen, and will invoke `flush_callback` recursively */
                lv_refr_now(_lv_refr_get_disp_refreshing());
            } else {
                /* Switch the current RGB frame buffer to `color_map` */
                esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);

                /* Waiting for the last frame buffer to complete transmission */
                ulTaskNotifyValueClear(NULL, ULONG_MAX);
                ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

                if (probe_result == FLUSH_PROBE_PART_COPY) {
                    /* Synchronously update the dirty area for another frame buffer */
                    flush_dirty_save(&dirty_area);
                    flush_dirty_copy(flush_get_next_buf(color_map), color_map, &dirty_area);
                }
            }
        }
    }

    lv_disp_flush_ready(drv);
}
#endif /* CONFIG_BSP_DISPLAY_LVGL_ROTATION_DEGREE */

#elif CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH && CONFIG_BSP_LCD_RGB_BUFFER_NUMS == 2

static void flush_callback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)drv->user_data;
    esp_lcd_panel_handle_t panel_handle = disp_ctx->panel_handle;
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

    /* Switch the current RGB frame buffer to `color_map` */
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);

    /* Waiting for the last frame buffer to complete transmission */
    ulTaskNotifyValueClear(NULL, ULONG_MAX);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    lv_disp_flush_ready(drv);
}

#elif CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH && CONFIG_BSP_LCD_RGB_BUFFER_NUMS == 3

#if CONFIG_BSP_DISPLAY_LVGL_ROTATION_DEGREE == 0
static void *lvgl_port_rgb_last_buf = NULL;
static void *lvgl_port_rgb_next_buf = NULL;
static void *lvgl_port_flush_next_buf = NULL;
#endif

void flush_callback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)drv->user_data;
    esp_lcd_panel_handle_t panel_handle = disp_ctx->panel_handle;
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

#if CONFIG_BSP_DISPLAY_LVGL_ROTATION_DEGREE != 0
    void *next_fb = get_next_frame_buffer(panel_handle);

    /* Rotate and copy dirty area from the current LVGL's buffer to the next RGB frame buffer */
    rotate_copy_pixel((uint16_t *)color_map, next_fb, offsetx1, offsety1, offsetx2, offsety2, LV_HOR_RES, LV_VER_RES, CONFIG_BSP_DISPLAY_LVGL_ROTATION_DEGREE);

    /* Switch the current RGB frame buffer to `next_fb` */
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, next_fb);
#else
    drv->draw_buf->buf1 = color_map;
    drv->draw_buf->buf2 = lvgl_port_flush_next_buf;
    lvgl_port_flush_next_buf = color_map;

    /* Switch the current RGB frame buffer to `color_map` */
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);

    lvgl_port_rgb_next_buf = color_map;
#endif

    lv_disp_flush_ready(drv);
}
#endif

static BaseType_t lcd_trans_done(esp_lcd_panel_handle_t panel, void *user_ctx)
{
    BaseType_t need_yield = pdFALSE;

    lvgl_port_display_ctx_t *disp_ctx = ((lv_disp_drv_t *)user_ctx)->user_data;
    TaskHandle_t lvgl_task_handle = disp_ctx->lvgl_task_handle;

#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH && (CONFIG_BSP_LCD_RGB_BUFFER_NUMS == 3) && (CONFIG_BSP_DISPLAY_LVGL_ROTATION_DEGREE == 0)
    if (lvgl_port_rgb_next_buf != lvgl_port_rgb_last_buf) {
        lvgl_port_flush_next_buf = lvgl_port_rgb_last_buf;
        lvgl_port_rgb_last_buf = lvgl_port_rgb_next_buf;
    }
#else
    // Notify that the current RGB frame buffer has been transmitted
    xTaskNotifyFromISR(lvgl_task_handle, ULONG_MAX, eNoAction, &need_yield);
#endif
    return (need_yield == pdTRUE);
}

#else

void flush_callback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)drv->user_data;
    esp_lcd_panel_handle_t panel_handle = disp_ctx->panel_handle;
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

    /* Just copy data from the color map to the RGB frame buffer */
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);

    lv_disp_flush_ready(drv);
}

#endif /* CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR */

static void update_callback(lv_disp_drv_t *drv)
{
    assert(drv != NULL);
    lvgl_port_display_ctx_t *disp_ctx = (lvgl_port_display_ctx_t *)drv->user_data;
    assert(disp_ctx != NULL);
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) disp_ctx->panel_handle;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    case LV_DISP_ROT_90:
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_180:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_270:
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    }
}

lv_disp_t *bsp_display_lcd_init()
{
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;           // LCD panel handle

    bsp_display_config_t disp_config = { 0 };

    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&disp_config, &panel_handle, &io_handle));

    // alloc draw buffers used by LVGL
    void *buf1 = NULL;
    void *buf2 = NULL;
    int buffer_size = 0;

    ESP_LOGD(TAG, "Malloc memory for LVGL buffer");
#ifndef CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
    // Normmaly, for RGB LCD, we just use one buffer for LVGL rendering
    buffer_size = BSP_LCD_H_RES * LVGL_BUFFER_HEIGHT;
    buf1 = heap_caps_malloc(buffer_size * sizeof(lv_color_t), LVGL_BUFFER_MALLOC);
    BSP_NULL_CHECK(buf1, NULL);
    ESP_LOGI(TAG, "LVGL buffer size: %dKB", buffer_size * sizeof(lv_color_t) / 1024);
#else
    // To avoid the tearing effect, we should use at least two frame buffers: one for LVGL rendering and another for RGB output
    buffer_size = BSP_LCD_H_RES * BSP_LCD_V_RES;
#if (CONFIG_BSP_LCD_RGB_BUFFER_NUMS == 3) && (CONFIG_BSP_DISPLAY_LVGL_ROTATION_DEGREE == 0) && CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
    // With the usage of three buffers and full-refresh, we always have one buffer available for rendering, eliminating the need to wait for the RGB's sync signal
    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 3, &lvgl_port_rgb_last_buf, &buf1, &buf2));
    lvgl_port_rgb_next_buf = lvgl_port_rgb_last_buf;
    lvgl_port_flush_next_buf = buf2;
#elif (CONFIG_BSP_LCD_RGB_BUFFER_NUMS == 3) && (CONFIG_BSP_DISPLAY_LVGL_ROTATION_DEGREE != 0)
    // Here we are using three frame buffers, one for LVGL rendering, and the other two for RGB driver (one of them is used for rotation)
    void *fbs[3];
    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 3, &fbs[0], &fbs[1], &fbs[2]));
    buf1 = fbs[2];
#else
    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
#endif
#endif /* CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR */

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = buffer_size,

#if CONFIG_BSP_DISPLAY_LVGL_ROTATION_90 || CONFIG_BSP_DISPLAY_LVGL_ROTATION_270
        .hres = BSP_LCD_V_RES,
        .vres = BSP_LCD_H_RES,
#else
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
#endif

        .flags = {
            .interface_RGB = true,
        },

        .user_lv_flush_cb = flush_callback,
        .user_lv_update_cb = update_callback,
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
        .user_lcd_transdone_cb = lcd_trans_done,
#endif
        .user_buf1 = buf1,
        .user_buf2 = buf2,

#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
        .refresh_mode.full_refresh = 1,
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
        .refresh_mode.direct_mode = 1,
#endif

#if CONFIG_BSP_LCD_RGB_REFRESH_MANUALLY
        .trans_mode = {
            .manual_mode = 1,
            .flags = {
                .ref_period = CONFIG_BSP_LCD_RGB_REFRESH_TASK_PERIOD,
                .ref_priority = CONFIG_BSP_LCD_RGB_REFRESH_TASK_PRIORITY,
            }
        },
#elif CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_MODE
        .trans_mode = {
            .bb_mode = 1,
        }
#elif CONFIG_BSP_LCD_RGB_REFRESH_AUTO
        .trans_mode = {
            .audo_mode = 1,
        }
#endif
    };

    return lvgl_port_add_disp(&disp_cfg);
}
