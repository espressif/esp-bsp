/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "driver/ppa.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "lvgl.h"
#include "esp_memory_utils.h"
#include "esp_private/esp_cache_private.h"
#include "esp_lvgl_port_dma2d.h"

static const char *TAG = "ppa_lvgl";

#define ALIGN_UP(num, align)      (((num) + ((align) - 1)) & ~((align) - 1))
#define ALIGN_DOWN(num, align)    ((num) & (~((align) - 1)))
#define PPA_LOG_CYCLE             (21)

// Need define in others file
extern void bsp_get_lvgl_buffer(void **buffer1, void **buffer2);

// Need to free when lvgl deinit
static uint8_t *fg_pic_buf = NULL;

typedef struct {
    uint32_t opa;
    void *dest_buf;
    uint32_t dest_w;
    uint32_t dest_h;
    uint32_t dest_stride;
    const void *src_buf;
    uint32_t src_stride;
    const lv_opa_t *mask_buf;
    uint32_t mask_stride;
} asm_dsc_t;

static size_t get_cache_line_size(const void *addr)
{
    esp_err_t ret = ESP_FAIL;
    size_t cache_line_size = 0;
    uint32_t heap_caps = esp_ptr_external_ram(addr) ? MALLOC_CAP_SPIRAM : MALLOC_CAP_INTERNAL;
    ret = esp_cache_get_alignment(heap_caps, &cache_line_size);
    return ret == ESP_OK ? cache_line_size : 0;
}

static lv_result_t bsp_get_lvgl_info(uint32_t *pic_w, uint32_t *pic_h, uint32_t *stride)
{
    lv_display_t *disp = lv_display_get_default();
    if (disp == NULL) {
        return LV_RESULT_INVALID;
    }
    *pic_w = lv_display_get_horizontal_resolution(disp);
    *pic_h = lv_display_get_vertical_resolution(disp);
    lv_color_format_t cf = lv_display_get_color_format(disp);
    *stride = lv_draw_buf_width_to_stride(*pic_w, cf);
    return LV_RESULT_OK;
}

static lv_result_t find_start_addr_and_offset(const uint8_t *src_buf, uint8_t **pic_addr, uint32_t *offsetx, uint32_t *offsety)
{
    uint8_t *buf1, *buf2, *addr;
    uint32_t pic_w, pic_h, stride;
    if (bsp_get_lvgl_info(&pic_w, &pic_h, &stride) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }
    uint32_t pic_size = stride * pic_h;
    bsp_get_lvgl_buffer((void **)&buf1, (void **)&buf2);
    if (buf1 == NULL || buf2 == NULL) {
        return LV_RESULT_INVALID;
    }
    uint32_t cache_line_size = get_cache_line_size(src_buf);
    if (src_buf >= buf1 && src_buf < buf1 + pic_size) {
        addr = buf1;
    } else if (src_buf >= buf2 && src_buf < buf2 + pic_size) {
        addr = buf2;
    } else if ((uint32_t)src_buf == ALIGN_UP((uint32_t)src_buf, cache_line_size)) { // If fg buffer is aligned, this is ok
        *pic_addr = (uint8_t *)src_buf;
        *offsetx = 0;
        *offsety = 0;
        return LV_RESULT_OK;
    } else {
        return LV_RESULT_INVALID;
    }
    cache_line_size = get_cache_line_size(addr);
    if ((uint32_t)addr != ALIGN_UP((uint32_t)addr, cache_line_size)) {
        return LV_RESULT_INVALID;
    }

    int pixel_size = stride / pic_w;
    uint32_t pixel_num = (src_buf - addr) / pixel_size;
    *pic_addr = addr;
    *offsetx = pixel_num % pic_w;
    *offsety = pixel_num / pic_w;
    return LV_RESULT_OK;
}

static lv_result_t _check_arg(uint32_t dest_w, uint32_t dest_h, const uint8_t *mask_buf)
{
    int color_format = lv_display_get_color_format(NULL);
    if (!((color_format == LV_COLOR_FORMAT_RGB565) || (color_format == LV_COLOR_FORMAT_RGB888)) ) {
        return LV_RESULT_INVALID;
    }

    uint32_t width = lv_display_get_horizontal_resolution(NULL);
    if (dest_w * dest_h < width * 20 || mask_buf != NULL) {
        return LV_RESULT_INVALID;
    }
    return LV_RESULT_OK;
}

static lv_result_t ppa_blend_get_color_mode_by_px_size(int px_size, uint32_t *mode)
{
    ppa_blend_color_mode_t color_mode = PPA_BLEND_COLOR_MODE_RGB565;
    if (px_size == 2) {
        color_mode = PPA_BLEND_COLOR_MODE_RGB565;
    } else if (px_size == 3) {
        color_mode = PPA_BLEND_COLOR_MODE_RGB888;
    } else if (px_size == 4) {
        color_mode = PPA_BLEND_COLOR_MODE_ARGB8888;
    } else {
        return LV_RESULT_INVALID;
    }
    *mode = color_mode;
    return LV_RESULT_OK;
}

static lv_result_t ppa_get_default_color_mode(uint32_t *color_mode)
{
    lv_color_format_t cf = lv_display_get_color_format(NULL);
    if (cf == LV_COLOR_FORMAT_RGB565) {
        *color_mode = PPA_FILL_COLOR_MODE_RGB565;
    } else if (cf == LV_COLOR_FORMAT_RGB888) {
        *color_mode = PPA_FILL_COLOR_MODE_RGB888;
    } else if (cf == LV_COLOR_FORMAT_ARGB8888) {
        *color_mode = PPA_FILL_COLOR_MODE_ARGB8888;
    } else {
        return LV_RESULT_INVALID;
    }
    return LV_RESULT_OK;
}

int esp_dma2d_convert_for_lvgl(asm_dsc_t *dsc, const uint8_t dest_px_size, uint32_t src_px_size)
{
    if (_check_arg(dsc->dest_w, dsc->dest_h, dsc->mask_buf) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    if (dest_px_size < 2 || dest_px_size > 3 || src_px_size < 2 || src_px_size > 3) {
        return LV_RESULT_INVALID;
    }

    dma2d_pic_config_t src = {
        .format = {
            .color_space = COLOR_SPACE_RGB,
            .pixel_format = src_px_size == 2 ? COLOR_PIXEL_RGB565 : COLOR_PIXEL_RGB888,
        },
        .pic_buf = (uint8_t *) dsc->src_buf,
        .pic_w = dsc->src_stride / src_px_size,
        .pic_h = dsc->dest_h,
        .offset_x = 0,
        .offset_y = 0,
        .block_w = dsc->dest_w,
        .block_h = dsc->dest_h,
    };

    dma2d_pic_config_t dest = {
        .format = {
            .color_space = COLOR_SPACE_RGB,
            .pixel_format = dest_px_size == 2 ? COLOR_PIXEL_RGB565 : COLOR_PIXEL_RGB888,
        },
        .block_w = dsc->dest_w,
        .block_h = dsc->dest_h,
    };

    uint32_t stride;
    if (bsp_get_lvgl_info(&dest.pic_w, &dest.pic_h, &stride) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    if (find_start_addr_and_offset(dsc->dest_buf, &dest.pic_buf, &dest.offset_x, &dest.offset_y) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    if (dma2d_color_convert(&src, &dest) == ESP_FAIL) {
        return LV_RESULT_INVALID;
    }

    return LV_RESULT_OK;
}

int esp_ppa_fill_opa_for_lvgl(asm_dsc_t *dsc, uint32_t dest_px_size)
{
    if (_check_arg(dsc->dest_w, dsc->dest_h, dsc->mask_buf) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    int64_t start = esp_timer_get_time();

    uint32_t dest_w = dsc->dest_w;
    uint32_t dest_h = dsc->dest_h;
    uint32_t pic_w, pic_h, stride;
    if (bsp_get_lvgl_info(&pic_w, &pic_h, &stride) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    uint8_t *bg_pic = NULL;
    uint32_t bg_offx = 0;
    uint32_t bg_offy = 0;
    uint32_t bg_color_mode = PPA_BLEND_COLOR_MODE_RGB888;
    if (ppa_get_default_color_mode(&bg_color_mode) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    static uint32_t fg_max_size = 0;
    uint32_t out_buf_size_align =  dest_w * dest_h; // fg color format is A8
    size_t cache_line_size = 0;
    esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &cache_line_size);
    out_buf_size_align = ALIGN_UP(out_buf_size_align, cache_line_size);
    if (fg_max_size < out_buf_size_align) {
        fg_max_size = out_buf_size_align;
        if (fg_pic_buf != NULL) {
            free(fg_pic_buf);
        }
        fg_pic_buf = heap_caps_aligned_calloc(cache_line_size, fg_max_size, sizeof(uint8_t), MALLOC_CAP_SPIRAM);
    }

    ppa_blend_color_mode_t fg_color_mode = PPA_BLEND_COLOR_MODE_A8;
    ppa_alpha_update_mode_t fg_updata_mode = PPA_ALPHA_FIX_VALUE;

    lv_color_t lv_color = {0};
    memcpy(&lv_color, dsc->src_buf, sizeof(lv_color_t));
    color_pixel_rgb888_data_t fg_rgb_data = {
        .r = lv_color.red,
        .g = lv_color.green,
        .b = lv_color.blue,
    };

    if (find_start_addr_and_offset(dsc->dest_buf, &bg_pic, &bg_offx, &bg_offy) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    ppa_client_handle_t ppa_client_handle;
    ppa_client_config_t ppa_client_config = {
        .oper_type = PPA_OPERATION_BLEND,
        .max_pending_trans_num = 1,
    };
    esp_err_t err = ppa_register_client(&ppa_client_config, &ppa_client_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ppa register client err: %d\n", err);
        goto _err;
    }

    uint32_t outbuf_size = ALIGN_UP(stride * pic_h, cache_line_size);

    ppa_blend_oper_config_t oper_config = {
        .in_bg.buffer = bg_pic,
        .in_bg.pic_w = pic_w,
        .in_bg.pic_h = pic_h,
        .in_bg.block_w = dest_w,
        .in_bg.block_h = dest_h,
        .in_bg.block_offset_x = bg_offx,
        .in_bg.block_offset_y = bg_offy,
        .in_bg.blend_cm = bg_color_mode,
        .bg_rgb_swap = 0,
        .bg_byte_swap = 0,
        .bg_alpha_update_mode = PPA_ALPHA_NO_CHANGE,

        .in_fg.buffer = fg_pic_buf,
        .in_fg.pic_w = dest_w,
        .in_fg.pic_h = dest_h,
        .in_fg.block_w = dest_w,
        .in_fg.block_h = dest_h,
        .in_fg.block_offset_x = 0,
        .in_fg.block_offset_y = 0,
        .in_fg.blend_cm = fg_color_mode,
        .fg_rgb_swap = 0,
        .fg_byte_swap = 0,
        .fg_alpha_update_mode = fg_updata_mode,
        .fg_alpha_fix_val = dsc->opa,
        .fg_fix_rgb_val = fg_rgb_data,

        .out.buffer = bg_pic,
        .out.buffer_size = outbuf_size,
        .out.pic_w = pic_w,
        .out.pic_h = pic_h,
        .out.block_offset_x = bg_offx,
        .out.block_offset_y = bg_offy,
        .out.blend_cm = bg_color_mode,

        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    err = ppa_do_blend(ppa_client_handle, &oper_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ppa do blend err: %d\n", err);
        goto _err;
    }
    err = ppa_unregister_client(ppa_client_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ppa unregister client err: %d\n", err);
        goto _err;
    }
    int duration = esp_timer_get_time() - start;
    uint32_t data_size = dest_w * dest_h * (dest_px_size + 1);
    float speed = data_size * 0.95367 / duration;
    static uint32_t count = 0;
    if (count++ % PPA_LOG_CYCLE == 0) {
        ESP_LOGD(TAG, "%s, %d ms, (x:%ld, y:%ld)[w:%ld, h:%ld][spx:%d, dpx:%ld], %.1f MB/s",
                 __FUNCTION__, duration / 1000, bg_offx, bg_offy, dest_w, dest_h, 1, dest_px_size, speed);
    }

    return LV_RESULT_OK;
_err:
    return LV_RESULT_INVALID;
}

int esp_ppa_blend_area_for_lvgl(asm_dsc_t *dsc, uint32_t dest_px_size, uint8_t src_px_size)
{
    if (_check_arg(dsc->dest_w, dsc->dest_h, dsc->mask_buf) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    uint32_t pic_w, pic_h, stride;
    if (bsp_get_lvgl_info(&pic_w, &pic_h, &stride) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    int64_t start = esp_timer_get_time();

    uint8_t *bg_pic = NULL;
    uint32_t bg_offx = 0;
    uint32_t bg_offy = 0;
    uint32_t bg_color_mode = PPA_BLEND_COLOR_MODE_RGB565;
    if (ppa_get_default_color_mode(&bg_color_mode) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    uint8_t *fg_pic = NULL;
    uint32_t fg_offx = 0;
    uint32_t fg_offy = 0;
    uint32_t fg_color_mode = PPA_BLEND_COLOR_MODE_RGB565;
    if (ppa_blend_get_color_mode_by_px_size(src_px_size, &fg_color_mode) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }
    ppa_alpha_update_mode_t fg_updata_mode = PPA_ALPHA_NO_CHANGE;

    uint32_t dest_w = dsc->dest_w;
    uint32_t dest_h = dsc->dest_h;

    if (find_start_addr_and_offset(dsc->dest_buf, &bg_pic, &bg_offx, &bg_offy) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    uint16_t cache_line_size = get_cache_line_size(bg_pic);

    if (find_start_addr_and_offset(dsc->src_buf, &fg_pic, &fg_offx, &fg_offy) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    ppa_client_handle_t ppa_client_handle;
    ppa_client_config_t ppa_client_config = {
        .oper_type = PPA_OPERATION_BLEND,
        .max_pending_trans_num = 1,
    };
    esp_err_t err = ppa_register_client(&ppa_client_config, &ppa_client_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ppa register client err: %d\n", err);
        goto _err;
    }

    uint32_t outbuf_size = ALIGN_UP(stride * pic_h, cache_line_size);

    ppa_blend_oper_config_t oper_config = {
        .in_bg.buffer = bg_pic,
        .in_bg.pic_w = pic_w,
        .in_bg.pic_h = pic_h,
        .in_bg.block_w = dest_w,
        .in_bg.block_h = dest_h,
        .in_bg.block_offset_x = bg_offx,
        .in_bg.block_offset_y = bg_offy,
        .in_bg.blend_cm = bg_color_mode,
        .bg_rgb_swap = 0,
        .bg_byte_swap = 0,
        .bg_alpha_update_mode = PPA_ALPHA_NO_CHANGE,

        .in_fg.buffer = fg_pic,
        .in_fg.pic_w = dest_w,
        .in_fg.pic_h = dest_h,
        .in_fg.block_w = dest_w,
        .in_fg.block_h = dest_h,
        .in_fg.block_offset_x = 0,
        .in_fg.block_offset_y = 0,
        .in_fg.blend_cm = PPA_BLEND_COLOR_MODE_ARGB8888,
        .fg_rgb_swap = 0,
        .fg_byte_swap = 0,
        .fg_alpha_update_mode = fg_updata_mode,

        .out.buffer = bg_pic,
        .out.buffer_size = outbuf_size,
        .out.pic_w = pic_w,
        .out.pic_h = pic_h,
        .out.block_offset_x = bg_offx,
        .out.block_offset_y = bg_offy,
        .out.blend_cm = bg_color_mode,

        .mode = PPA_TRANS_MODE_BLOCKING,
    };
    err = ppa_do_blend(ppa_client_handle, &oper_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ppa do blend err: %d\n", err);
        goto _err;
    }
    err = ppa_unregister_client(ppa_client_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ppa unregister client err: %d\n", err);
        goto _err;
    }
    int duration = esp_timer_get_time() - start;
    uint32_t data_size = dest_w * dest_h * (dest_px_size * 2 + src_px_size);
    float speed = data_size * 0.95367 / duration;
    static uint32_t count = 0;
    if (count++ % PPA_LOG_CYCLE == 0) {
        ESP_LOGD(TAG, "%s, %d ms, (x:%ld, y:%ld)[w:%ld, h:%ld][spx:%d, dpx:%ld], %.1f MB/s",
                 __FUNCTION__, duration / 1000, bg_offx, bg_offy, dest_w, dest_h, src_px_size, dest_px_size, speed);
    }

    return LV_RESULT_OK;
_err:
    return LV_RESULT_INVALID;
}

static inline uint32_t convert_color_to_uint32(lv_color_t color, uint8_t opa)
{
    return (uint32_t)((uint32_t)opa << 24) + ((uint32_t)color.red << 16) + ((uint32_t)color.green << 8) + (color.blue);
}

int esp_ppa_fill_for_lvgl(asm_dsc_t *dsc, uint32_t dest_px_size)
{
    if (_check_arg(dsc->dest_w, dsc->dest_h, dsc->mask_buf) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    uint32_t pic_w, pic_h, stride;
    if (bsp_get_lvgl_info(&pic_w, &pic_h, &stride) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    uint8_t *pic_addr = NULL;
    uint32_t offsetx = 0;
    uint32_t offsety = 0;
    uint32_t dest_w = dsc->dest_w;
    uint32_t dest_h = dsc->dest_h;

    if (find_start_addr_and_offset(dsc->dest_buf, &pic_addr, &offsetx, &offsety) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    int64_t start = esp_timer_get_time();

    uint32_t color_mode = PPA_FILL_COLOR_MODE_RGB565;
    if (ppa_get_default_color_mode(&color_mode) == LV_RESULT_INVALID) {
        return LV_RESULT_INVALID;
    }

    lv_color_t lv_color = {0};
    memcpy(&lv_color, dsc->src_buf, sizeof(lv_color_t));
    uint32_t color = convert_color_to_uint32(lv_color, dsc->opa);

    ppa_client_handle_t ppa_client_handle = NULL;

    int cache_line_size = get_cache_line_size(pic_addr);
    uint32_t out_buf_size_align = ALIGN_UP(stride * pic_h, cache_line_size);
    ppa_client_config_t ppa_client_config = {
        .oper_type = PPA_OPERATION_FILL,
        .max_pending_trans_num = 1,
    };
    esp_err_t err = ppa_register_client(&ppa_client_config, &ppa_client_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ppa register client err: %d\n", err);
    }

    ppa_fill_oper_config_t fill_trans_config = {
        .fill_block_w = dest_w,
        .fill_block_h = dest_h,
        .fill_argb_color = {
            .val = color,
        },
        .mode = PPA_TRANS_MODE_BLOCKING,

        .out = {
            .buffer = pic_addr,
            .buffer_size = out_buf_size_align,
            .pic_w = pic_w,
            .pic_h = pic_h,
            .block_offset_x = offsetx,
            .block_offset_y = offsety,
            .fill_cm = color_mode,
        }
    };
    err = ppa_do_fill(ppa_client_handle, &fill_trans_config);
    int duration = esp_timer_get_time() - start;
    err = ppa_unregister_client(ppa_client_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ppa unregister client err: %d\n", err);
    }
    uint32_t data_size = dsc->dest_w * dsc->dest_h * dest_px_size;
    float speed = data_size * 0.95367 / duration;
    static uint32_t count = 0;
    if (count++ % PPA_LOG_CYCLE == 0) {
        ESP_LOGD(TAG, "%s, %d ms, (x:%ld, y:%ld)[w:%ld, h:%ld][spx:%d, dpx:%ld], %ld KB, %.1f MB/s",
                 __FUNCTION__, duration / 1000, offsetx, offsety, dest_w, dest_h, 0, dest_px_size, data_size / 1024, speed);
    }
    if (err != ESP_OK) {
        return LV_RESULT_INVALID;
    }
    return LV_RESULT_OK;
}

void esp_ppa_mem_release(void)
{
    if (fg_pic_buf) {
        free(fg_pic_buf);
        fg_pic_buf = NULL;
    }
}
