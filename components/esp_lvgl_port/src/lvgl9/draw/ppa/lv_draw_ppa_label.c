/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PPA label/glyph draw
 */

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"
#include "draw/lv_draw_label_private.h"

#if 0
/* PPA LABEL/LETTER: A8 mask blend with per-row span merge (lv_draw_ppa_glyph_blend).
 * Disabled — performance still poor vs SW; technique under evaluation. */

#if LV_USE_DRAW_SW
#include "draw/sw/lv_draw_sw.h"
#endif

static void draw_letter_cb(lv_draw_task_t *t, lv_draw_glyph_dsc_t *glyph_draw_dsc,
                           lv_draw_fill_dsc_t *fill_draw_dsc, const lv_area_t *fill_area);
static void draw_glyph_bitmap(lv_draw_task_t *t, lv_draw_glyph_dsc_t *glyph_draw_dsc);
static void draw_glyph_bitmap_sw_rotated(lv_draw_task_t *t, lv_draw_glyph_dsc_t *glyph_draw_dsc);
static void draw_glyph_image(lv_draw_task_t *t, lv_draw_glyph_dsc_t *glyph_draw_dsc);

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_letter(lv_draw_task_t *t, const lv_draw_letter_dsc_t *dsc,
        const lv_area_t *coords)
{
    if (dsc->opa <= (lv_opa_t)LV_OPA_MIN) {
        return;
    }

    lv_draw_glyph_dsc_t glyph_dsc;
    lv_draw_glyph_dsc_init(&glyph_dsc);
    glyph_dsc.opa = dsc->opa;
    glyph_dsc.color = dsc->color;
    glyph_dsc.rotation = dsc->rotation;
    glyph_dsc.pivot = dsc->pivot;

    lv_draw_unit_draw_letter(t, &glyph_dsc, &(lv_point_t) {
        .x = coords->x1, .y = coords->y1
    }, dsc->font, dsc->unicode, draw_letter_cb);

    if (glyph_dsc._draw_buf) {
        lv_draw_buf_destroy(glyph_dsc._draw_buf);
    }
}

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_label(lv_draw_task_t *t, const lv_draw_label_dsc_t *dsc,
        const lv_area_t *coords)
{
    if (dsc->opa <= (lv_opa_t)LV_OPA_MIN) {
        return;
    }
    lv_draw_label_iterate_characters(t, dsc, coords, draw_letter_cb);
}

static void LV_ATTRIBUTE_FAST_MEM draw_letter_cb(lv_draw_task_t *t, lv_draw_glyph_dsc_t *glyph_draw_dsc,
        lv_draw_fill_dsc_t *fill_draw_dsc, const lv_area_t *fill_area)
{
    if (glyph_draw_dsc) {
        switch (glyph_draw_dsc->format) {
        case LV_FONT_GLYPH_FORMAT_NONE:
#if LV_USE_FONT_PLACEHOLDER
            if (glyph_draw_dsc->bg_coords) {
                lv_draw_border_dsc_t border_dsc;
                lv_draw_border_dsc_init(&border_dsc);
                border_dsc.opa = glyph_draw_dsc->opa;
                border_dsc.color = glyph_draw_dsc->color;
                border_dsc.width = 1;
                lv_draw_ppa_border(t, &border_dsc, glyph_draw_dsc->bg_coords);
            }
#endif
            break;
        case LV_FONT_GLYPH_FORMAT_A1:
        case LV_FONT_GLYPH_FORMAT_A2:
        case LV_FONT_GLYPH_FORMAT_A3:
        case LV_FONT_GLYPH_FORMAT_A4:
        case LV_FONT_GLYPH_FORMAT_A8:
            if (glyph_draw_dsc->rotation % 3600 == 0) {
                draw_glyph_bitmap(t, glyph_draw_dsc);
            }
#if LV_USE_DRAW_SW
            else {
                draw_glyph_bitmap_sw_rotated(t, glyph_draw_dsc);
            }
#endif
            break;
        case LV_FONT_GLYPH_FORMAT_IMAGE:
            draw_glyph_image(t, glyph_draw_dsc);
            break;
        default:
            break;
        }
    }

    if (fill_draw_dsc && fill_area) {
        lv_draw_ppa_fill(t, fill_draw_dsc, fill_area);
    }
}

static void LV_ATTRIBUTE_FAST_MEM draw_glyph_bitmap(lv_draw_task_t *t, lv_draw_glyph_dsc_t *glyph_draw_dsc)
{
    lv_draw_ppa_unit_t *u = (lv_draw_ppa_unit_t *)t->draw_unit;
    lv_layer_t *layer = t->target_layer;
    lv_draw_buf_t *draw_buf = layer->draw_buf;
    const lv_area_t *mask_area = glyph_draw_dsc->letter_coords;

    const uint8_t *mask;
    uint32_t mask_stride;

    if (lv_font_has_static_bitmap(glyph_draw_dsc->g->resolved_font) &&
            glyph_draw_dsc->g->format == LV_FONT_GLYPH_FORMAT_A8) {
        glyph_draw_dsc->g->req_raw_bitmap = 1;
        mask = lv_font_get_glyph_static_bitmap(glyph_draw_dsc->g);
        if (mask == NULL) {
            return;
        }
        mask_stride = glyph_draw_dsc->g->stride;
        if (mask_stride == 0) {
            mask_stride = (uint32_t)lv_draw_buf_width_to_stride(lv_area_get_width(mask_area), LV_COLOR_FORMAT_A8);
        }
    } else {
        glyph_draw_dsc->glyph_data = lv_font_get_glyph_bitmap(glyph_draw_dsc->g, glyph_draw_dsc->_draw_buf);
        const lv_draw_buf_t *a8_buf = glyph_draw_dsc->glyph_data;
        if (a8_buf == NULL) {
            return;
        }
        mask = a8_buf->data;
        mask_stride = a8_buf->header.stride;
    }

    lv_draw_ppa_glyph_blend(u, draw_buf, mask_area, mask, mask_stride, glyph_draw_dsc->color,
                            glyph_draw_dsc->opa, &t->clip_area, &layer->buf_area);
}

#if LV_USE_DRAW_SW
static void LV_ATTRIBUTE_FAST_MEM draw_glyph_bitmap_sw_rotated(lv_draw_task_t *t,
        lv_draw_glyph_dsc_t *glyph_draw_dsc)
{
    glyph_draw_dsc->glyph_data = lv_font_get_glyph_bitmap(glyph_draw_dsc->g, glyph_draw_dsc->_draw_buf);
    const lv_draw_buf_t *draw_buf = glyph_draw_dsc->glyph_data;
    if (draw_buf == NULL) {
        return;
    }

    lv_draw_image_dsc_t img_dsc;
    lv_draw_image_dsc_init(&img_dsc);
    img_dsc.rotation = glyph_draw_dsc->rotation;
    img_dsc.scale_x = LV_SCALE_NONE;
    img_dsc.scale_y = LV_SCALE_NONE;
    img_dsc.opa = glyph_draw_dsc->opa;
    img_dsc.src = glyph_draw_dsc->glyph_data;
    img_dsc.recolor = glyph_draw_dsc->color;
    img_dsc.pivot = (lv_point_t) {
        .x = glyph_draw_dsc->pivot.x,
        .y = glyph_draw_dsc->g->box_h + glyph_draw_dsc->g->ofs_y
    };
    lv_draw_sw_image(t, &img_dsc, glyph_draw_dsc->letter_coords);
}
#endif

static void LV_ATTRIBUTE_FAST_MEM draw_glyph_image(lv_draw_task_t *t, lv_draw_glyph_dsc_t *glyph_draw_dsc)
{
#if LV_USE_PPA_IMG || LV_USE_PPA_TRANSFORM
    glyph_draw_dsc->glyph_data = lv_font_get_glyph_bitmap(glyph_draw_dsc->g, glyph_draw_dsc->_draw_buf);
    if (glyph_draw_dsc->glyph_data == NULL) {
        return;
    }

    lv_draw_image_dsc_t img_dsc;
    lv_draw_image_dsc_init(&img_dsc);
    img_dsc.base.layer = t->target_layer;
    img_dsc.rotation = glyph_draw_dsc->rotation;
    img_dsc.scale_x = LV_SCALE_NONE;
    img_dsc.scale_y = LV_SCALE_NONE;
    img_dsc.opa = glyph_draw_dsc->opa;
    img_dsc.src = glyph_draw_dsc->glyph_data;
    img_dsc.recolor = glyph_draw_dsc->color;
    img_dsc.pivot = (lv_point_t) {
        .x = glyph_draw_dsc->pivot.x,
        .y = glyph_draw_dsc->g->box_h + glyph_draw_dsc->g->ofs_y
    };

    const lv_draw_buf_t *src_buf = glyph_draw_dsc->glyph_data;
    img_dsc.header = src_buf->header;

    lv_draw_ppa_img(t, &img_dsc, glyph_draw_dsc->letter_coords);
#elif LV_USE_DRAW_SW
    glyph_draw_dsc->glyph_data = lv_font_get_glyph_bitmap(glyph_draw_dsc->g, glyph_draw_dsc->_draw_buf);
    if (glyph_draw_dsc->glyph_data == NULL) {
        return;
    }

    lv_draw_image_dsc_t img_dsc;
    lv_draw_image_dsc_init(&img_dsc);
    img_dsc.rotation = glyph_draw_dsc->rotation;
    img_dsc.scale_x = LV_SCALE_NONE;
    img_dsc.scale_y = LV_SCALE_NONE;
    img_dsc.opa = glyph_draw_dsc->opa;
    img_dsc.src = glyph_draw_dsc->glyph_data;
    img_dsc.recolor = glyph_draw_dsc->color;
    img_dsc.pivot = (lv_point_t) {
        .x = glyph_draw_dsc->pivot.x,
        .y = glyph_draw_dsc->g->box_h + glyph_draw_dsc->g->ofs_y
    };
    lv_draw_sw_image(t, &img_dsc, glyph_draw_dsc->letter_coords);
#else
    LV_UNUSED(t);
    LV_UNUSED(glyph_draw_dsc);
#endif
}

#endif
