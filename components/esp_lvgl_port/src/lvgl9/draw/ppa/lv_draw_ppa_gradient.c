/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PPA gradient fill draw
 */

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_PPA_GRADIENT

/*******************************************************************************
* Function definitions
*******************************************************************************/

static uint32_t mix_color_u32(lv_color_t a, lv_color_t b, uint32_t mix_q8);
static uint32_t grad_color_u32(const lv_grad_dsc_t *grad, uint8_t pos_frac, lv_opa_t fill_opa);
static void enqueue_strip(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
                          const lv_area_t *strip, const lv_area_t *buf_area, uint32_t color);

/*******************************************************************************
* Public API functions
*******************************************************************************/

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_gradient(lv_draw_task_t *t, const lv_draw_fill_dsc_t *dsc,
        const lv_area_t *coords)
{
    lv_draw_ppa_unit_t *u = (lv_draw_ppa_unit_t *)t->draw_unit;
    lv_layer_t *layer = t->target_layer;
    lv_draw_buf_t *draw_buf = layer->draw_buf;

    lv_area_t rel_coords;
    lv_area_copy(&rel_coords, coords);

    lv_area_t rel_clip;
    lv_area_copy(&rel_clip, &t->clip_area);

    lv_area_t blend_area;
    if (!lv_area_intersect(&blend_area, &rel_coords, &rel_clip)) {
        return;
    }

    int32_t total_w = lv_area_get_width(&rel_coords);
    int32_t total_h = lv_area_get_height(&rel_coords);
    if (total_w <= 0 || total_h <= 0) {
        return;
    }

    bool vertical = (dsc->grad.dir == LV_GRAD_DIR_VER);

    int32_t span = vertical ? total_h : total_w;
    int32_t steps = LV_PPA_GRADIENT_STEPS;
    if (steps > span) {
        steps = span;
    }
    if (steps <= 0) {
        return;
    }

    for (int32_t i = 0; i < steps; i++) {
        uint32_t mix_q8 = (uint32_t)((i * 2 + 1) * 256 / (steps * 2));
        if (mix_q8 > 256) {
            mix_q8 = 256;
        }
        uint8_t pos_frac = (uint8_t)((mix_q8 * 255u) / 256u);
        uint32_t color;
        if (dsc->grad.stops_count == 2) {
            color = mix_color_u32(dsc->grad.stops[0].color, dsc->grad.stops[1].color, mix_q8);
            color = (color & 0x00FFFFFFu) | ((uint32_t)dsc->opa << 24);
        } else {
            color = grad_color_u32(&dsc->grad, pos_frac, dsc->opa);
        }

        lv_area_t strip;
        if (vertical) {
            int32_t y1 = rel_coords.y1 + (i * total_h) / steps;
            int32_t y2 = rel_coords.y1 + ((i + 1) * total_h) / steps - 1;
            if (y2 < y1) {
                continue;
            }
            lv_area_set(&strip, rel_coords.x1, y1, rel_coords.x2, y2);
        } else {
            int32_t x1 = rel_coords.x1 + (i * total_w) / steps;
            int32_t x2 = rel_coords.x1 + ((i + 1) * total_w) / steps - 1;
            if (x2 < x1) {
                continue;
            }
            lv_area_set(&strip, x1, rel_coords.y1, x2, rel_coords.y2);
        }

        lv_area_t clipped;
        if (!lv_area_intersect(&clipped, &strip, &blend_area)) {
            continue;
        }
        enqueue_strip(u, draw_buf, &clipped, &layer->buf_area, color);
    }
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static uint32_t LV_ATTRIBUTE_FAST_MEM mix_color_u32(lv_color_t a, lv_color_t b, uint32_t mix_q8)
{
    uint32_t inv = 256u - mix_q8;
    uint32_t r = (a.red   * inv + b.red   * mix_q8) >> 8;
    uint32_t g = (a.green * inv + b.green * mix_q8) >> 8;
    uint32_t blue = (a.blue * inv + b.blue * mix_q8) >> 8;
    return (0xFFu << 24) | (r << 16) | (g << 8) | blue;
}

static uint32_t LV_ATTRIBUTE_FAST_MEM grad_color_u32(const lv_grad_dsc_t *grad, uint8_t pos_frac,
        lv_opa_t fill_opa)
{
    uint8_t n = grad->stops_count;
    if (n < 2) {
        return 0;
    }

    if (pos_frac <= grad->stops[0].frac) {
        return lv_draw_ppa_fill_color_u32(grad->stops[0].color,
                                          (lv_opa_t)(((uint16_t)grad->stops[0].opa * fill_opa) >> 8));
    }

    for (uint8_t i = 0; i < n - 1; i++) {
        if (pos_frac <= grad->stops[i + 1].frac) {
            uint8_t f0 = grad->stops[i].frac;
            uint8_t f1 = grad->stops[i + 1].frac;
            if (f1 <= f0) {
                return lv_draw_ppa_fill_color_u32(grad->stops[i + 1].color,
                                                  (lv_opa_t)(((uint16_t)grad->stops[i + 1].opa * fill_opa) >> 8));
            }
            uint32_t mix_q8 = (uint32_t)(pos_frac - f0) * 256u / (uint32_t)(f1 - f0);
            if (mix_q8 > 256) {
                mix_q8 = 256;
            }
            lv_color_t c = grad->stops[i].color;
            lv_color_t d = grad->stops[i + 1].color;
            uint32_t rgb = mix_color_u32(c, d, mix_q8);
            lv_opa_t opa = (lv_opa_t)(((uint16_t)grad->stops[i].opa * (256u - mix_q8)
                                       + (uint16_t)grad->stops[i + 1].opa * mix_q8) >> 8);
            opa = (lv_opa_t)(((uint16_t)opa * fill_opa) >> 8);
            return (rgb & 0x00FFFFFFu) | ((uint32_t)opa << 24);
        }
    }

    const uint8_t last_i = (uint8_t)LV_MIN((int)grad->stops_count - 1, LV_GRADIENT_MAX_STOPS - 1);
    return lv_draw_ppa_fill_color_u32(grad->stops[last_i].color,
                                      (lv_opa_t)(((uint16_t)grad->stops[last_i].opa * fill_opa) >> 8));
}

static void LV_ATTRIBUTE_FAST_MEM enqueue_strip(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
        const lv_area_t *strip, const lv_area_t *buf_area, uint32_t color)
{
    lv_area_t rel;
    lv_area_copy(&rel, strip);
    lv_area_move(&rel, -buf_area->x1, -buf_area->y1);
    lv_draw_ppa_solid_op(u, draw_buf, &rel, color);
}

#endif /* LV_USE_PPA_GRADIENT */
