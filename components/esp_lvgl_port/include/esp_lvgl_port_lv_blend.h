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

#if !CONFIG_LV_DRAW_SW_ASM_CUSTOM
#warning "esp_lvgl_port_lv_blend.h included, but CONFIG_LV_DRAW_SW_ASM_CUSTOM not set. Assembly rendering not used"
#else

#include "lv_version.h"

/*********************
 *      DEFINES
 *********************/

#ifndef LV_DRAW_SW_COLOR_BLEND_TO_ARGB8888
#define LV_DRAW_SW_COLOR_BLEND_TO_ARGB8888(dsc) \
    _lv_color_blend_to_argb8888_esp(dsc)
#endif

#ifndef LV_DRAW_SW_COLOR_BLEND_TO_RGB565
#define LV_DRAW_SW_COLOR_BLEND_TO_RGB565(dsc) \
    _lv_color_blend_to_rgb565_esp(dsc)
#endif


/**********************
 *      TYPEDEFS
 **********************/

typedef struct {
    uint32_t opa;
    void *dst_buf;
    uint32_t dst_w;
    uint32_t dst_h;
    uint32_t dst_stride;
    const void *src_buf;
    uint32_t src_stride;
    const lv_opa_t *mask_buf;
    uint32_t mask_stride;
} asm_dsc_t;

#if ((LVGL_VERSION_MAJOR == 9) && (LVGL_VERSION_MINOR == 1))

typedef struct {
    void * dest_buf;
    int32_t dest_w;
    int32_t dest_h;
    int32_t dest_stride;
    const lv_opa_t * mask_buf;
    int32_t mask_stride;
    lv_color_t color;
    lv_opa_t opa;
} bsp_blend_fill_dsc_t;

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
} bsp_blend_image_dsc_t;

#elif ((LVGL_VERSION_MAJOR == 9) && (LVGL_VERSION_MINOR == 2))

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
} bsp_blend_fill_dsc_t;

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
} bsp_blend_image_dsc_t;

#endif

/**********************
 * GLOBAL PROTOTYPES
 **********************/

extern int lv_color_blend_to_argb8888_esp(asm_dsc_t *asm_dsc);

static inline lv_result_t _lv_color_blend_to_argb8888_esp(void *dsc)
{
    bsp_blend_fill_dsc_t * fill_dsc = (bsp_blend_fill_dsc_t *) dsc;
    asm_dsc_t asm_dsc = {
        .dst_buf = fill_dsc->dest_buf,
        .dst_w = fill_dsc->dest_w,
        .dst_h = fill_dsc->dest_h,
        .dst_stride = fill_dsc->dest_stride,
        .src_buf = &fill_dsc->color,
    };

    return lv_color_blend_to_argb8888_esp(&asm_dsc);
}

extern int lv_color_blend_to_rgb565_esp(asm_dsc_t *asm_dsc);

static inline lv_result_t _lv_color_blend_to_rgb565_esp(void *dsc)
{
    bsp_blend_fill_dsc_t * fill_dsc = (bsp_blend_fill_dsc_t *) dsc;
    asm_dsc_t asm_dsc = {
        .dst_buf = fill_dsc->dest_buf,
        .dst_w = fill_dsc->dest_w,
        .dst_h = fill_dsc->dest_h,
        .dst_stride = fill_dsc->dest_stride,
        .src_buf = &fill_dsc->color,
    };

    return lv_color_blend_to_rgb565_esp(&asm_dsc);
}

#endif // CONFIG_LV_DRAW_SW_ASM_CUSTOM

#ifdef __cplusplus
} /*extern "C"*/
#endif
