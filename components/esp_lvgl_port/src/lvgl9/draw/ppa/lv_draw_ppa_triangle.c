/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PPA triangle draw
 */

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_PPA_TRIANGLE

/*******************************************************************************
* Function definitions
*******************************************************************************/

static bool detect_right_triangle(const lv_draw_triangle_dsc_t *dsc,
                                  lv_point_t *out_corner, lv_point_t *out_hx, lv_point_t *out_hy);
static void enqueue_scanline(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
                             const lv_area_t *line, const lv_area_t *clip,
                             const lv_area_t *buf_area, uint32_t color);

/*******************************************************************************
* Public API functions
*******************************************************************************/

/* Hardware path for axis-aligned right triangles, i.e. triangles where two of
 * the three vertices share the same x or the same y so one of the edges is
 * already a screen axis. The third edge becomes a linear ramp in x as a
 * function of y, which is exactly what a sequence of horizontal PPA fills can
 * cover (one fill per scanline). General triangles need true edge equations
 * and stay on the SW renderer; gradients/recolor on the triangle also fall
 * back to SW because the engine cannot interpolate colour across the
 * scanlines we emit.
 *
 * The decomposition is binary (no anti-aliasing): each scanline either
 * belongs to the triangle or it doesn't. UI use cases for triangles are
 * mostly chevrons/markers/indicator arrows where the missing AA is hard to
 * notice; the SW path is still available when the difference matters. */
void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_triangle(lv_draw_task_t *t, const lv_draw_triangle_dsc_t *dsc)
{
    if (dsc->opa <= (lv_opa_t)LV_OPA_MIN) {
        return;
    }
    if (dsc->grad.dir != LV_GRAD_DIR_NONE) {
        return;
    }

    lv_point_t corner;       /* right-angle vertex */
    lv_point_t hx, hy;        /* horizontal and vertical vertices (relative to corner) */
    if (!detect_right_triangle(dsc, &corner, &hx, &hy)) {
        return;
    }

    int32_t y_top = LV_MIN(corner.y, hy.y);
    int32_t y_bot = LV_MAX(corner.y, hy.y);
    int32_t height = y_bot - y_top;
    if (height <= 0) {
        return;
    }

    /* Hypotenuse runs from (hx.x, corner.y) to (corner.x, hy.y). For each
     * scanline, interpolate the x extent linearly between the two endpoints
     * using fixed-point arithmetic so the file does not depend on libm. */
    int32_t x_at_corner = corner.x;     /* x at the corner.y end of the hypotenuse */
    int32_t x_at_hx     = hx.x;         /* x at the hy.y end of the hypotenuse */
    int32_t base_y      = corner.y;
    int32_t apex_y      = hy.y;

    lv_draw_ppa_unit_t *u = (lv_draw_ppa_unit_t *)t->draw_unit;
    lv_layer_t *layer = t->target_layer;
    lv_draw_buf_t *draw_buf = layer->draw_buf;
    uint32_t color = lv_draw_ppa_fill_color_u32(dsc->color, dsc->opa);

    for (int32_t y = y_top; y <= y_bot; y++) {
        /* progress in [0..1] from base_y towards apex_y. The hypotenuse's x
         * walks linearly from x_at_corner to x_at_hx along that progress. */
        int32_t num = y - base_y;
        int32_t denom = apex_y - base_y;
        if (denom == 0) {
            continue;
        }
        /* Sign-preserving rounded division so both upper- and lower-pointing
         * triangles converge to the apex pixel at the right scanline. */
        int32_t step_q8 = (num * 256) / denom;
        if (step_q8 < 0) {
            step_q8 = -step_q8;
        }
        if (step_q8 > 256) {
            step_q8 = 256;
        }

        int32_t hyp_x = x_at_corner + ((x_at_hx - x_at_corner) * step_q8 + 128) / 256;
        int32_t x1 = LV_MIN(corner.x, hyp_x);
        int32_t x2 = LV_MAX(corner.x, hyp_x);

        lv_area_t line = { .x1 = x1, .y1 = y, .x2 = x2, .y2 = y };
        enqueue_scanline(u, draw_buf, &line, &t->clip_area, &layer->buf_area, color);
    }
}

/*******************************************************************************
* Private functions
*******************************************************************************/

/* Identify the right-angle vertex of an axis-aligned triangle. We look for a
 * vertex that shares y with one neighbour and x with the other; if no such
 * vertex exists the triangle is generic and the worker bails out. */
static bool detect_right_triangle(const lv_draw_triangle_dsc_t *dsc,
                                  lv_point_t *out_corner, lv_point_t *out_hx, lv_point_t *out_hy)
{
    lv_point_t p[3];
    for (int i = 0; i < 3; i++) {
        p[i].x = (int32_t)dsc->p[i].x;
        p[i].y = (int32_t)dsc->p[i].y;
    }

    for (int i = 0; i < 3; i++) {
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;
        bool ij_same_y = (p[i].y == p[j].y);
        bool ik_same_x = (p[i].x == p[k].x);
        if (ij_same_y && ik_same_x) {
            *out_corner = p[i];
            *out_hx     = p[j];
            *out_hy     = p[k];
            return true;
        }
        bool ij_same_x = (p[i].x == p[j].x);
        bool ik_same_y = (p[i].y == p[k].y);
        if (ij_same_x && ik_same_y) {
            *out_corner = p[i];
            *out_hx     = p[k];
            *out_hy     = p[j];
            return true;
        }
    }
    return false;
}

static void LV_ATTRIBUTE_FAST_MEM enqueue_scanline(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
        const lv_area_t *line, const lv_area_t *clip,
        const lv_area_t *buf_area, uint32_t color)
{
    lv_area_t fill_area;
    if (!lv_area_intersect(&fill_area, line, clip)) {
        return;
    }
    lv_area_move(&fill_area, -buf_area->x1, -buf_area->y1);
    if (fill_area.x2 < fill_area.x1 || fill_area.y2 < fill_area.y1) {
        return;
    }

    lv_draw_ppa_solid_op(u, draw_buf, &fill_area, color);
}

#endif /* LV_USE_PPA_TRIANGLE */
