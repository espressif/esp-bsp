/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PPA line draw
 */

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_PPA_LINE

/*******************************************************************************
* Function definitions
*******************************************************************************/

static void enqueue_segment(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
                            const lv_area_t *strip, const lv_area_t *clip,
                            const lv_area_t *buf_area, uint32_t color);
static void draw_round_cap(lv_draw_task_t *t, int32_t cx, int32_t cy, int32_t width,
                           lv_color_t color, lv_opa_t opa);

/*******************************************************************************
* Public API functions
*******************************************************************************/

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_line(lv_draw_task_t *t, const lv_draw_line_dsc_t *dsc)
{
    if (dsc->opa <= (lv_opa_t)LV_OPA_MIN) {
        return;
    }
    if (dsc->width <= 0) {
        return;
    }

    lv_draw_ppa_unit_t *u = (lv_draw_ppa_unit_t *)t->draw_unit;
    lv_layer_t *layer = t->target_layer;
    lv_draw_buf_t *draw_buf = layer->draw_buf;

    int32_t p1x = (int32_t)dsc->p1.x;
    int32_t p1y = (int32_t)dsc->p1.y;
    int32_t p2x = (int32_t)dsc->p2.x;
    int32_t p2y = (int32_t)dsc->p2.y;

    int32_t half = dsc->width / 2;
    int32_t extra = dsc->width - half - 1;
    if (extra < 0) {
        extra = 0;
    }

    lv_area_t strip;
    if (p1y == p2y) {
        int32_t x1 = LV_MIN(p1x, p2x);
        int32_t x2 = LV_MAX(p1x, p2x);
        strip.x1 = x1;
        strip.x2 = x2;
        strip.y1 = p1y - half;
        strip.y2 = p1y + extra;
    } else if (p1x == p2x) {
        int32_t y1 = LV_MIN(p1y, p2y);
        int32_t y2 = LV_MAX(p1y, p2y);
        strip.x1 = p1x - half;
        strip.x2 = p1x + extra;
        strip.y1 = y1;
        strip.y2 = y2;
    } else {
        return;
    }

    if (strip.x2 < strip.x1 || strip.y2 < strip.y1) {
        return;
    }

    uint32_t color = lv_draw_ppa_fill_color_u32(dsc->color, dsc->opa);
    enqueue_segment(u, draw_buf, &strip, &t->clip_area, &layer->buf_area, color);

    if (dsc->round_start) {
        draw_round_cap(t, p1x, p1y, dsc->width, dsc->color, dsc->opa);
    }
    if (dsc->round_end) {
        draw_round_cap(t, p2x, p2y, dsc->width, dsc->color, dsc->opa);
    }
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static void LV_ATTRIBUTE_FAST_MEM enqueue_segment(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
        const lv_area_t *strip, const lv_area_t *clip,
        const lv_area_t *buf_area, uint32_t color)
{
    lv_area_t fill_area;
    if (!lv_area_intersect(&fill_area, strip, clip)) {
        return;
    }
    lv_area_move(&fill_area, -buf_area->x1, -buf_area->y1);

    lv_draw_ppa_solid_op(u, draw_buf, &fill_area, color);
}

#if LV_USE_PPA_ROUND_FILL

static void LV_ATTRIBUTE_FAST_MEM draw_round_cap(lv_draw_task_t *t, int32_t cx, int32_t cy, int32_t width,
        lv_color_t color, lv_opa_t opa)
{
    if (width <= 0) {
        return;
    }

    int32_t r = width / 2;
    lv_area_t cap = {
        .x1 = cx - r,
        .y1 = cy - r,
        .x2 = cx + r - 1 + (width & 1),
        .y2 = cy + r - 1 + (width & 1),
    };

    lv_draw_fill_dsc_t fill_dsc;
    lv_draw_fill_dsc_init(&fill_dsc);
    fill_dsc.color = color;
    fill_dsc.opa = opa;
    fill_dsc.radius = LV_RADIUS_CIRCLE;
    lv_draw_ppa_round_fill(t, &fill_dsc, &cap);
}

#else

static void LV_ATTRIBUTE_FAST_MEM draw_round_cap(lv_draw_task_t *t, int32_t cx, int32_t cy, int32_t width,
        lv_color_t color, lv_opa_t opa)
{
    lv_draw_ppa_unit_t *u = (lv_draw_ppa_unit_t *)t->draw_unit;
    lv_layer_t *layer = t->target_layer;
    int32_t r = width / 2;
    lv_area_t cap = {
        .x1 = cx - r,
        .y1 = cy - r,
        .x2 = cx + r,
        .y2 = cy + r,
    };
    lv_area_t clipped;
    if (!lv_area_intersect(&clipped, &cap, &t->clip_area)) {
        return;
    }
    lv_area_move(&clipped, -layer->buf_area.x1, -layer->buf_area.y1);
    lv_draw_ppa_solid_op(u, layer->draw_buf, &clipped, lv_draw_ppa_fill_color_u32(color, opa));
}

#endif /* LV_USE_PPA_ROUND_FILL */

#endif /* LV_USE_PPA_LINE */
