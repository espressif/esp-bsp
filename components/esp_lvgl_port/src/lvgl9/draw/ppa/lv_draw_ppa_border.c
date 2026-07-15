/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lv_draw_ppa_border.c
 *
 */

/*********************
 *      INCLUDES
 *********************/

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_PPA_BORDER

/**********************
 *  STATIC PROTOTYPES
 **********************/

static int32_t int_sqrt(int32_t v);
static int32_t corner_inset(int32_t rout, int32_t dy);
static void draw_border_sharp(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf, const lv_draw_border_dsc_t *dsc,
                              const lv_area_t *rel_coords, const lv_area_t *rel_clip, uint32_t fill_color);
static void draw_border_rounded(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf, const lv_draw_border_dsc_t *dsc,
                                const lv_area_t *rel_coords, const lv_area_t *rel_clip, int32_t rout,
                                uint32_t fill_color);
static void enqueue_strip(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
                          const lv_area_t *strip, const lv_area_t *clip,
                          uint32_t fill_color);

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_border(lv_draw_task_t *t, const lv_draw_border_dsc_t *dsc,
        const lv_area_t *coords)
{
    if (dsc->opa <= LV_OPA_MIN || dsc->width <= 0 || dsc->side == LV_BORDER_SIDE_NONE) {
        return;
    }

    lv_draw_ppa_unit_t *u = (lv_draw_ppa_unit_t *)t->draw_unit;
    lv_layer_t *layer = t->target_layer;
    lv_draw_buf_t *draw_buf = layer->draw_buf;

    lv_area_t rel_coords;
    lv_area_copy(&rel_coords, coords);
    lv_area_move(&rel_coords, -layer->buf_area.x1, -layer->buf_area.y1);

    lv_area_t rel_clip;
    lv_area_copy(&rel_clip, &t->clip_area);
    lv_area_move(&rel_clip, -layer->buf_area.x1, -layer->buf_area.y1);

    uint32_t fill_color = lv_draw_ppa_fill_color_u32(dsc->color, dsc->opa);

    if (dsc->radius <= 0) {
        draw_border_sharp(u, draw_buf, dsc, &rel_coords, &rel_clip, fill_color);
        return;
    }

    int32_t coords_w = lv_area_get_width(&rel_coords);
    int32_t coords_h = lv_area_get_height(&rel_coords);
    int32_t short_side = LV_MIN(coords_w, coords_h);
    int32_t rout = dsc->radius;
    if (rout == LV_RADIUS_CIRCLE || rout > short_side / 2) {
        rout = short_side / 2;
    }
    if (rout <= 0) {
        draw_border_sharp(u, draw_buf, dsc, &rel_coords, &rel_clip, fill_color);
        return;
    }

    draw_border_rounded(u, draw_buf, dsc, &rel_coords, &rel_clip, rout, fill_color);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

static void LV_ATTRIBUTE_FAST_MEM draw_border_sharp(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
        const lv_draw_border_dsc_t *dsc,
        const lv_area_t *rel_coords, const lv_area_t *rel_clip,
        uint32_t fill_color)
{
    int32_t width = dsc->width;

    if (dsc->side & LV_BORDER_SIDE_TOP) {
        lv_area_t s = {
            .x1 = rel_coords->x1,
            .y1 = rel_coords->y1,
            .x2 = rel_coords->x2,
            .y2 = LV_MIN(rel_coords->y2, rel_coords->y1 + width - 1),
        };
        enqueue_strip(u, draw_buf, &s, rel_clip, fill_color);
    }

    if (dsc->side & LV_BORDER_SIDE_BOTTOM) {
        lv_area_t s = {
            .x1 = rel_coords->x1,
            .y1 = LV_MAX(rel_coords->y1, rel_coords->y2 - width + 1),
            .x2 = rel_coords->x2,
            .y2 = rel_coords->y2,
        };
        enqueue_strip(u, draw_buf, &s, rel_clip, fill_color);
    }

    int32_t side_y1 = (dsc->side & LV_BORDER_SIDE_TOP)    ? rel_coords->y1 + width : rel_coords->y1;
    int32_t side_y2 = (dsc->side & LV_BORDER_SIDE_BOTTOM) ? rel_coords->y2 - width : rel_coords->y2;
    if (side_y2 >= side_y1) {
        if (dsc->side & LV_BORDER_SIDE_LEFT) {
            lv_area_t s = {
                .x1 = rel_coords->x1,
                .y1 = side_y1,
                .x2 = LV_MIN(rel_coords->x2, rel_coords->x1 + width - 1),
                .y2 = side_y2,
            };
            enqueue_strip(u, draw_buf, &s, rel_clip, fill_color);
        }
        if (dsc->side & LV_BORDER_SIDE_RIGHT) {
            lv_area_t s = {
                .x1 = LV_MAX(rel_coords->x1, rel_coords->x2 - width + 1),
                .y1 = side_y1,
                .x2 = rel_coords->x2,
                .y2 = side_y2,
            };
            enqueue_strip(u, draw_buf, &s, rel_clip, fill_color);
        }
    }
}

static void LV_ATTRIBUTE_FAST_MEM draw_border_rounded(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
        const lv_draw_border_dsc_t *dsc,
        const lv_area_t *rel_coords, const lv_area_t *rel_clip,
        int32_t rout, uint32_t fill_color)
{
    int32_t width = dsc->width;

    if (dsc->side & LV_BORDER_SIDE_TOP) {
        for (int32_t i = 0; i < width; i++) {
            int32_t dy = rout - i;
            int32_t off = corner_inset(rout, dy);
            lv_area_t s = {
                .x1 = rel_coords->x1 + off,
                .y1 = rel_coords->y1 + i,
                .x2 = rel_coords->x2 - off,
                .y2 = rel_coords->y1 + i,
            };
            enqueue_strip(u, draw_buf, &s, rel_clip, fill_color);
        }
    }

    if (dsc->side & LV_BORDER_SIDE_BOTTOM) {
        for (int32_t i = 0; i < width; i++) {
            int32_t dy = rout - i;
            int32_t off = corner_inset(rout, dy);
            lv_area_t s = {
                .x1 = rel_coords->x1 + off,
                .y1 = rel_coords->y2 - i,
                .x2 = rel_coords->x2 - off,
                .y2 = rel_coords->y2 - i,
            };
            enqueue_strip(u, draw_buf, &s, rel_clip, fill_color);
        }
    }

    int32_t side_y1 = (dsc->side & LV_BORDER_SIDE_TOP) ? rel_coords->y1 + rout : rel_coords->y1;
    int32_t side_y2 = (dsc->side & LV_BORDER_SIDE_BOTTOM) ? rel_coords->y2 - rout : rel_coords->y2;
    if (side_y2 >= side_y1) {
        if (dsc->side & LV_BORDER_SIDE_LEFT) {
            for (int32_t i = 0; i < width; i++) {
                lv_area_t s = {
                    .x1 = rel_coords->x1 + i,
                    .y1 = side_y1,
                    .x2 = rel_coords->x1 + i,
                    .y2 = side_y2,
                };
                enqueue_strip(u, draw_buf, &s, rel_clip, fill_color);
            }
        }
        if (dsc->side & LV_BORDER_SIDE_RIGHT) {
            for (int32_t i = 0; i < width; i++) {
                lv_area_t s = {
                    .x1 = rel_coords->x2 - i,
                    .y1 = side_y1,
                    .x2 = rel_coords->x2 - i,
                    .y2 = side_y2,
                };
                enqueue_strip(u, draw_buf, &s, rel_clip, fill_color);
            }
        }
    }
}

static int32_t LV_ATTRIBUTE_FAST_MEM corner_inset(int32_t rout, int32_t dy)
{
    if (dy <= 0) {
        return rout;
    }
    if (dy > rout) {
        dy = rout;
    }
    int32_t r_sq = rout * rout;
    int32_t inner_offset = rout - int_sqrt(r_sq - (dy - 1) * (dy - 1));
    if (inner_offset < 0) {
        inner_offset = 0;
    }
    if (inner_offset > rout) {
        inner_offset = rout;
    }
    return inner_offset;
}

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
        uint32_t fill_color)
{
    lv_area_t fill_area;
    if (!lv_area_intersect(&fill_area, strip, clip)) {
        return;
    }

    lv_draw_ppa_solid_op(u, draw_buf, &fill_area, fill_color);
}

#endif /* LV_USE_PPA_BORDER */
