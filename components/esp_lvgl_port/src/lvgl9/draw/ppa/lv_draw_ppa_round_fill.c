/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lv_draw_ppa_round_fill.c
 *
 */

/*********************
 *      INCLUDES
 *********************/

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_PPA_ROUND_FILL

/**********************
 *  STATIC PROTOTYPES
 **********************/

static int32_t int_sqrt(int32_t v);
static void enqueue_strip(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
                          const lv_area_t *strip, const lv_area_t *clip,
                          const lv_area_t *buf_area, uint32_t color);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/* Decompose a rounded rectangle fill into a body band plus 2*radius scanlines
 * that approximate the four rounded corners. Each scanline writes a single
 * horizontal strip joining the left and right rounded contours, so the per-op
 * cost grows linearly with the radius (not quadratically). The decomposition
 * is binary: corner pixels are either fully inside or fully outside, no
 * coverage. The Kconfig threshold (LV_PPA_ROUND_FILL_MIN_RADIUS) keeps small
 * radii on SW where coverage matters more and PPA's per-op overhead bites.
 *
 * The geometry uses an integer sqrt so the file does not pull in libm. The
 * scanline x extent is `radius - sqrt(radius^2 - (yc - y)^2)`, applied
 * symmetrically to the top/bottom corners. A single body fill covers the
 * straight middle band. */
void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_round_fill(lv_draw_task_t *t, const lv_draw_fill_dsc_t *dsc,
        const lv_area_t *coords)
{
    if (dsc->opa <= (lv_opa_t)LV_OPA_MIN) {
        return;
    }
    if (dsc->radius <= 0) {
        return;
    }

    lv_draw_ppa_unit_t *u = (lv_draw_ppa_unit_t *)t->draw_unit;
    lv_layer_t *layer = t->target_layer;
    lv_draw_buf_t *draw_buf = layer->draw_buf;

    int32_t coords_w = lv_area_get_width(coords);
    int32_t coords_h = lv_area_get_height(coords);
    int32_t short_side = LV_MIN(coords_w, coords_h);
    int32_t r = dsc->radius;
    /* Cap the radius so the corner contours never cross the rectangle centre
     * (LVGL applies the same clamp in lv_draw_sw_border). */
    if (r > short_side / 2) {
        r = short_side / 2;
    }
    if (r <= 0) {
        return;
    }

    uint32_t color = lv_draw_ppa_fill_color_u32(dsc->color, dsc->opa);

    /* Body: large rectangular band between the top and bottom corner regions. */
    {
        lv_area_t body = {
            .x1 = coords->x1,
            .y1 = coords->y1 + r,
            .x2 = coords->x2,
            .y2 = coords->y2 - r,
        };
        if (body.y2 >= body.y1) {
            enqueue_strip(u, draw_buf, &body, &t->clip_area, &layer->buf_area, color);
        }
    }

    /* Top and bottom corner scanlines: for each row in [0, r) compute the
     * inner x offset and emit a single horizontal strip from x1+offset to
     * x2-offset. The same offset is mirrored for the bottom band. */
    int32_t r_sq = r * r;
    for (int32_t i = 0; i < r; i++) {
        /* Distance from the row to the corner centre, in [1, r]. */
        int32_t dy = r - i;
        int32_t inner_offset = r - int_sqrt(r_sq - (dy - 1) * (dy - 1));
        if (inner_offset < 0) {
            inner_offset = 0;
        }
        if (inner_offset > r) {
            inner_offset = r;
        }

        /* Top band scanline. */
        lv_area_t top = {
            .x1 = coords->x1 + inner_offset,
            .y1 = coords->y1 + i,
            .x2 = coords->x2 - inner_offset,
            .y2 = coords->y1 + i,
        };
        if (top.x2 >= top.x1) {
            enqueue_strip(u, draw_buf, &top, &t->clip_area, &layer->buf_area, color);
        }

        /* Bottom band scanline (mirrored row). */
        lv_area_t bot = {
            .x1 = coords->x1 + inner_offset,
            .y1 = coords->y2 - i,
            .x2 = coords->x2 - inner_offset,
            .y2 = coords->y2 - i,
        };
        if (bot.y2 < bot.y1) {
            continue;
        }
        if (bot.y2 != top.y2 && bot.x2 >= bot.x1) {
            enqueue_strip(u, draw_buf, &bot, &t->clip_area, &layer->buf_area, color);
        }
    }
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/* Newton's method, integer-only. Good to ~one ULP for the radii LVGL uses
 * (well under 1e6), keeps the worker in IRAM-friendly territory. */
static int32_t LV_ATTRIBUTE_FAST_MEM int_sqrt(int32_t v)
{
    if (v <= 0) {
        return 0;
    }
    int32_t x = v;
    int32_t y = (x + 1) / 2;
    while (y < x) {
        x = y;
        y = (x + v / x) / 2;
    }
    return x;
}

static void LV_ATTRIBUTE_FAST_MEM enqueue_strip(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
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

#endif /* LV_USE_PPA_ROUND_FILL */
