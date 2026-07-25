/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PPA mask rect draw
 */

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_PPA_MASK_RECT

/*******************************************************************************
* Function definitions
*******************************************************************************/

static void enqueue_clear(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
                          const lv_area_t *area, const lv_area_t *buf_area);

/*******************************************************************************
* Public API functions
*******************************************************************************/

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_mask_rect(lv_draw_task_t *t, const lv_draw_mask_rect_dsc_t *dsc)
{
    if (dsc->radius != 0) {
        return;
    }

    lv_draw_ppa_unit_t *u = (lv_draw_ppa_unit_t *)t->draw_unit;
    lv_layer_t *layer = t->target_layer;
    lv_draw_buf_t *draw_buf = layer->draw_buf;
    const lv_area_t *buf_area = &layer->buf_area;

    lv_area_t draw_area;
    if (!lv_area_intersect(&draw_area, &dsc->area, &t->clip_area)) {
        return;
    }

    if (dsc->keep_outside) {
        enqueue_clear(u, draw_buf, &draw_area, buf_area);
        return;
    }

    lv_area_t strip;

    lv_area_set(&strip, t->clip_area.x1, t->clip_area.y1, t->clip_area.x2, dsc->area.y1 - 1);
    if (strip.y2 >= strip.y1) {
        enqueue_clear(u, draw_buf, &strip, buf_area);
    }

    lv_area_set(&strip, t->clip_area.x1, dsc->area.y2 + 1, t->clip_area.x2, t->clip_area.y2);
    if (strip.y2 >= strip.y1) {
        enqueue_clear(u, draw_buf, &strip, buf_area);
    }

    int32_t side_y1 = LV_MAX(t->clip_area.y1, dsc->area.y1);
    int32_t side_y2 = LV_MIN(t->clip_area.y2, dsc->area.y2);
    if (side_y2 < side_y1) {
        return;
    }

    lv_area_set(&strip, t->clip_area.x1, side_y1, dsc->area.x1 - 1, side_y2);
    if (strip.x2 >= strip.x1) {
        enqueue_clear(u, draw_buf, &strip, buf_area);
    }

    lv_area_set(&strip, dsc->area.x2 + 1, side_y1, t->clip_area.x2, side_y2);
    if (strip.x2 >= strip.x1) {
        enqueue_clear(u, draw_buf, &strip, buf_area);
    }
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static void LV_ATTRIBUTE_FAST_MEM enqueue_clear(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
        const lv_area_t *area, const lv_area_t *buf_area)
{
    lv_area_t rel;
    lv_area_copy(&rel, area);
    lv_area_move(&rel, -buf_area->x1, -buf_area->y1);
    if (rel.x2 < rel.x1 || rel.y2 < rel.y1) {
        return;
    }

    lv_draw_ppa_solid_op(u, draw_buf, &rel, 0);
}

#endif /* LV_USE_PPA_MASK_RECT */
