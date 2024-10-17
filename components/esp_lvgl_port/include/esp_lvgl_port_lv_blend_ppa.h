/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

typedef struct {
    uint32_t opa;
    void * dest_buf;
    uint32_t dest_w;
    uint32_t dest_h;
    uint32_t dest_stride;
    const void * src_buf;
    uint32_t src_stride;
    const lv_opa_t * mask_buf;
    uint32_t mask_stride;
} asm_dsc_t;

// ppa for lvgl
extern int esp_ppa_fill_for_lvgl(asm_dsc_t *dsc, uint32_t dest_px_size);
extern int esp_ppa_fill_opa_for_lvgl(asm_dsc_t *dsc, uint32_t dest_px_size);
extern int esp_ppa_blend_area_for_lvgl(asm_dsc_t * dsc, uint32_t dest_px_size, uint8_t src_px_size);
extern int esp_dma2d_convert_for_lvgl(asm_dsc_t * dsc, const uint8_t dest_px_size, uint32_t src_px_size);

/*********************
 *      DEFINES
 *********************/

#ifndef LV_DRAW_SW_COLOR_BLEND_TO_RGB565
#define LV_DRAW_SW_COLOR_BLEND_TO_RGB565(dsc) \
    _lv_color_blend_to_rgb565_esp2d(dsc)
#endif

#ifndef LV_DRAW_SW_COLOR_BLEND_TO_RGB565_WITH_OPA
#define LV_DRAW_SW_COLOR_BLEND_TO_RGB565_WITH_OPA(dsc) \
    _lv_color_blend_to_rgb565_with_opa_esp2d(dsc)
#endif


#ifndef LV_DRAW_SW_RGB565_BLEND_NORMAL_TO_RGB565
#define LV_DRAW_SW_RGB565_BLEND_NORMAL_TO_RGB565(dsc)  \
    _lv_rgb565_blend_normal_to_rgb565_esp2d(dsc)
#endif

#ifndef LV_DRAW_SW_RGB565_BLEND_NORMAL_TO_RGB565_WITH_OPA
#define LV_DRAW_SW_RGB565_BLEND_NORMAL_TO_RGB565_WITH_OPA(dsc)  \
    _lv_rgb565_blend_normal_to_rgb565_with_opa_esp2d(dsc)
#endif

#ifndef LV_DRAW_SW_RGB888_BLEND_NORMAL_TO_RGB565
#define LV_DRAW_SW_RGB888_BLEND_NORMAL_TO_RGB565(dsc, src_px_size)  \
    _lv_rgb888_blend_normal_to_rgb565_esp2d(dsc, src_px_size)
#endif

#ifndef LV_DRAW_SW_RGB888_BLEND_NORMAL_TO_RGB565_WITH_OPA
#define LV_DRAW_SW_RGB888_BLEND_NORMAL_TO_RGB565_WITH_OPA(dsc, src_px_size)  \
    _lv_rgb888_blend_normal_to_rgb565_with_opa_esp2d(dsc, src_px_size)
#endif

#ifndef LV_DRAW_SW_ARGB8888_BLEND_NORMAL_TO_RGB565
#define LV_DRAW_SW_ARGB8888_BLEND_NORMAL_TO_RGB565(dsc)  \
    _lv_argb8888_blend_normal_to_rgb565_esp2d(dsc)
#endif

#ifndef LV_DRAW_SW_ARGB8888_BLEND_NORMAL_TO_RGB565_WITH_OPA
#define LV_DRAW_SW_ARGB8888_BLEND_NORMAL_TO_RGB565_WITH_OPA(dsc)  \
    _lv_argb8888_blend_normal_to_rgb565_with_opa_esp2d(dsc)
#endif

#ifndef LV_DRAW_SW_COLOR_BLEND_TO_RGB888
#define LV_DRAW_SW_COLOR_BLEND_TO_RGB888(dsc, dst_px_size) \
    _lv_color_blend_to_rgb888_esp2d(dsc, dst_px_size)
#endif

#ifndef LV_DRAW_SW_COLOR_BLEND_TO_RGB888_WITH_OPA
#define LV_DRAW_SW_COLOR_BLEND_TO_RGB888_WITH_OPA(dsc, dst_px_size) \
    _lv_color_blend_to_rgb888_with_opa_esp2d(dsc, dst_px_size)
#endif

#ifndef LV_DRAW_SW_RGB565_BLEND_NORMAL_TO_RGB888
#define LV_DRAW_SW_RGB565_BLEND_NORMAL_TO_RGB888(dsc, dst_px_size)  \
    _lv_rgb565_blend_normal_to_rgb888_esp2d(dsc, dst_px_size)
#endif

#ifndef LV_DRAW_SW_RGB565_BLEND_NORMAL_TO_RGB888_WITH_OPA
#define LV_DRAW_SW_RGB565_BLEND_NORMAL_TO_RGB888_WITH_OPA(dsc, dst_px_size)  \
    _lv_rgb565_blend_normal_to_rgb888_with_opa_esp2d(dsc, dst_px_size)
#endif

#ifndef LV_DRAW_SW_RGB888_BLEND_NORMAL_TO_RGB888
#define LV_DRAW_SW_RGB888_BLEND_NORMAL_TO_RGB888(dsc, dst_px_size, src_px_size)  \
    _lv_rgb888_blend_normal_to_rgb888_esp2d(dsc, dst_px_size, src_px_size)
#endif

#ifndef LV_DRAW_SW_RGB888_BLEND_NORMAL_TO_RGB888_WITH_OPA
#define LV_DRAW_SW_RGB888_BLEND_NORMAL_TO_RGB888_WITH_OPA(dsc, dst_px_size, src_px_size)  \
    _lv_rgb888_blend_normal_to_rgb888_with_opa_esp2d(dsc, dst_px_size, src_px_size)
#endif

#ifndef LV_DRAW_SW_COLOR_BLEND_TO_ARGB8888
#define LV_DRAW_SW_COLOR_BLEND_TO_ARGB8888(dsc) \
    _lv_color_blend_to_argb8888_esp2d(dsc)
#endif

#ifndef LV_DRAW_SW_ARGB8888_BLEND_NORMAL_TO_RGB888
#define LV_DRAW_SW_ARGB8888_BLEND_NORMAL_TO_RGB888(dsc, dst_px_size)  \
    _lv_argb8888_blend_normal_to_rgb888_esp2d(dsc, dst_px_size)
#endif

#ifndef LV_DRAW_SW_ARGB8888_BLEND_NORMAL_TO_RGB888_WITH_OPA
#define LV_DRAW_SW_ARGB8888_BLEND_NORMAL_TO_RGB888_WITH_OPA(dsc, dst_px_size)  \
    _lv_argb8888_blend_normal_to_rgb888_with_opa_esp2d(dsc, dst_px_size)
#endif

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

typedef struct {
    void * dest_buf;
    int32_t dest_w;
    int32_t dest_h;
    int32_t dest_stride;
    const lv_opa_t * mask_buf;
    int32_t mask_stride;
    lv_color_t color;
    lv_opa_t opa;
    lv_area_t relative_area;
} blend_fill_dsc_t;

typedef struct {
    void * dest_buf;
    int32_t dest_w;
    int32_t dest_h;
    int32_t dest_stride;
    const lv_opa_t * mask_buf;
    int32_t mask_stride;
    const void * src_buf;
    int32_t src_stride;
    lv_color_format_t src_color_format;
    lv_opa_t opa;
    lv_blend_mode_t blend_mode;
    lv_area_t relative_area;    /**< The blend area relative to the layer's buffer area. */
    lv_area_t src_area;             /**< The original src area. */
} blend_image_dsc_t;

// Simple fill: ppa fill
static inline lv_result_t _lv_color_blend_to_rgb565_esp2d(blend_fill_dsc_t * dsc)
{
    asm_dsc_t asm_dsc = {
        .dest_buf = dsc->dest_buf,
        .dest_w = dsc->dest_w,
        .dest_h = dsc->dest_h,
        .dest_stride = dsc->dest_stride,
        .src_buf = &dsc->color,
        .opa = dsc->opa,
    };
    return esp_ppa_fill_for_lvgl(&asm_dsc, 2);
}

static inline lv_result_t _lv_color_blend_to_rgb888_esp2d(blend_fill_dsc_t * dsc, uint32_t dst_px_size)
{
    asm_dsc_t asm_dsc = {
        .dest_buf = dsc->dest_buf,
        .dest_w = dsc->dest_w,
        .dest_h = dsc->dest_h,
        .dest_stride = dsc->dest_stride,
        .src_buf = &dsc->color,
        .opa = dsc->opa,
    };
    return esp_ppa_fill_for_lvgl(&asm_dsc, dst_px_size);
}

static inline lv_result_t _lv_color_blend_to_argb8888_esp2d(blend_fill_dsc_t * dsc)
{
    asm_dsc_t asm_dsc = {
        .dest_buf = dsc->dest_buf,
        .dest_w = dsc->dest_w,
        .dest_h = dsc->dest_h,
        .dest_stride = dsc->dest_stride,
        .src_buf = &dsc->color,
        .opa = dsc->opa,
    };
    return esp_ppa_fill_for_lvgl(&asm_dsc, 4);
}

// Blend with opa
static inline lv_result_t _lv_color_blend_to_rgb565_with_opa_esp2d(blend_fill_dsc_t * dsc)
{
    asm_dsc_t asm_dsc = {
        .dest_buf = dsc->dest_buf,
        .dest_w = dsc->dest_w,
        .dest_h = dsc->dest_h,
        .dest_stride = dsc->dest_stride,
        .src_buf = &dsc->color,
        .opa = dsc->opa,
    };
    return esp_ppa_fill_opa_for_lvgl(&asm_dsc, 2);
}

static inline lv_result_t _lv_color_blend_to_rgb888_with_opa_esp2d(blend_fill_dsc_t * dsc,
                                                                   uint32_t dst_px_size)
{
    asm_dsc_t asm_dsc = {
        .dest_buf = dsc->dest_buf,
        .dest_w = dsc->dest_w,
        .dest_h = dsc->dest_h,
        .dest_stride = dsc->dest_stride,
        .src_buf = &dsc->color,
        .opa = dsc->opa,
    };
    return esp_ppa_fill_opa_for_lvgl(&asm_dsc, dst_px_size);
}

// Color convert (including dam2d memcpy)
static inline lv_result_t _lv_rgb565_blend_normal_to_rgb565_esp2d(blend_image_dsc_t * dsc)
{
    asm_dsc_t asm_dsc = {
        .dest_buf = dsc->dest_buf,
        .dest_w = dsc->dest_w,
        .dest_h = dsc->dest_h,
        .dest_stride = dsc->dest_stride,
        .src_buf = dsc->src_buf,
        .src_stride = dsc->src_stride,
        .opa = dsc->opa,
    };
    return esp_dma2d_convert_for_lvgl(&asm_dsc, 2, 2);
}

static inline lv_result_t _lv_rgb565_blend_normal_to_rgb888_esp2d(blend_image_dsc_t * dsc,
                                                                  uint32_t dst_px_size)
{
    asm_dsc_t asm_dsc = {
        .dest_buf = dsc->dest_buf,
        .dest_w = dsc->dest_w,
        .dest_h = dsc->dest_h,
        .dest_stride = dsc->dest_stride,
        .src_buf = dsc->src_buf,
        .src_stride = dsc->src_stride,
        .opa = dsc->opa,
    };
    return esp_dma2d_convert_for_lvgl(&asm_dsc, dst_px_size, 2);
}

static inline lv_result_t _lv_rgb888_blend_normal_to_rgb565_esp2d(blend_image_dsc_t * dsc,
                                                                  uint32_t src_px_size)
{
    asm_dsc_t asm_dsc = {
        .dest_buf = dsc->dest_buf,
        .dest_w = dsc->dest_w,
        .dest_h = dsc->dest_h,
        .dest_stride = dsc->dest_stride,
        .src_buf = dsc->src_buf,
        .src_stride = dsc->src_stride,
        .opa = dsc->opa,
    };
    return esp_dma2d_convert_for_lvgl(&asm_dsc, src_px_size, 2);
}

static inline lv_result_t _lv_rgb888_blend_normal_to_rgb888_esp2d(blend_image_dsc_t * dsc,
                                                                  uint32_t dst_px_size,
                                                                  uint32_t src_px_size)
{
    asm_dsc_t asm_dsc = {
        .dest_buf = dsc->dest_buf,
        .dest_w = dsc->dest_w,
        .dest_h = dsc->dest_h,
        .dest_stride = dsc->dest_stride,
        .src_buf = dsc->src_buf,
        .src_stride = dsc->src_stride,
        .opa = dsc->opa,
    };
    return esp_dma2d_convert_for_lvgl(&asm_dsc, dst_px_size, src_px_size);
}

// Blend normal
static inline lv_result_t _lv_argb8888_blend_normal_to_rgb565_esp2d(blend_image_dsc_t * dsc)
{
    asm_dsc_t asm_dsc = {
        .dest_buf = dsc->dest_buf,
        .dest_w = dsc->dest_w,
        .dest_h = dsc->dest_h,
        .dest_stride = dsc->dest_stride,
        .src_buf = dsc->src_buf,
        .src_stride = dsc->src_stride,
        .opa = dsc->opa,
    };
    return esp_ppa_blend_area_for_lvgl(&asm_dsc, 2, 4);
}

static inline lv_result_t _lv_argb8888_blend_normal_to_rgb888_esp2d(blend_image_dsc_t * dsc,
                                                                    uint32_t dst_px_size)
{
    asm_dsc_t asm_dsc = {
        .dest_buf = dsc->dest_buf,
        .dest_w = dsc->dest_w,
        .dest_h = dsc->dest_h,
        .dest_stride = dsc->dest_stride,
        .src_buf = dsc->src_buf,
        .src_stride = dsc->src_stride,
        .opa = dsc->opa,
    };
    return esp_ppa_blend_area_for_lvgl(&asm_dsc, dst_px_size, 4);
}


static inline lv_result_t _lv_rgb565_blend_normal_to_rgb565_with_opa_esp2d(blend_image_dsc_t * dsc)
{
    return LV_RESULT_INVALID;
}

static inline lv_result_t _lv_rgb888_blend_normal_to_rgb565_with_opa_esp2d(blend_image_dsc_t * dsc,
                                                                           uint32_t src_px_size)
{
    return LV_RESULT_INVALID;
}

static inline lv_result_t _lv_argb8888_blend_normal_to_rgb565_with_opa_esp2d(blend_image_dsc_t * dsc)
{
    return LV_RESULT_INVALID;
}

static inline lv_result_t _lv_rgb565_blend_normal_to_rgb888_with_opa_esp2d(blend_image_dsc_t * dsc,
                                                                           uint32_t dst_px_size)
{
    return LV_RESULT_INVALID;
}

static inline lv_result_t _lv_rgb888_blend_normal_to_rgb888_with_opa_esp2d(blend_image_dsc_t * dsc,
                                                                           uint32_t dst_px_size, uint32_t src_px_size)
{
    return LV_RESULT_INVALID;
}

static inline lv_result_t _lv_argb8888_blend_normal_to_rgb888_with_opa_esp2d(blend_image_dsc_t * dsc,
                                                                             uint32_t dst_px_size)
{
    return LV_RESULT_INVALID;
}

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif
