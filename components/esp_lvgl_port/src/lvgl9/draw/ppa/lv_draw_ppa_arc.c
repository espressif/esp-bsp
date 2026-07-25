/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PPA arc draw
 */

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_DRAW_SW
#include "draw/sw/lv_draw_sw.h"
#endif

#if LV_USE_PPA_ARC && LV_USE_PPA_BORDER

static bool arc_is_full_ring(const lv_draw_arc_dsc_t *dsc)
{
    return (dsc->start_angle + 360 == dsc->end_angle || dsc->start_angle == dsc->end_angle + 360);
}

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_arc(lv_draw_task_t *t, const lv_draw_arc_dsc_t *dsc,
        const lv_area_t *coords)
{
    if (dsc->opa <= (lv_opa_t)LV_OPA_MIN) {
        return;
    }
    if (dsc->width <= 0) {
        return;
    }
    if (dsc->start_angle == dsc->end_angle) {
        return;
    }

    if (dsc->img_src == NULL && arc_is_full_ring(dsc)) {
        int32_t width = dsc->width;
        if (width > (int32_t)dsc->radius) {
            width = (int32_t)dsc->radius;
        }

        lv_draw_border_dsc_t border_dsc;
        lv_draw_border_dsc_init(&border_dsc);
        border_dsc.opa = dsc->opa;
        border_dsc.color = dsc->color;
        border_dsc.width = width;
        border_dsc.radius = LV_RADIUS_CIRCLE;
        border_dsc.side = LV_BORDER_SIDE_FULL;
        lv_draw_ppa_border(t, &border_dsc, coords);
        return;
    }

#if LV_USE_DRAW_SW
    lv_draw_sw_arc(t, dsc, coords);
#else
    LV_LOG_WARN("PPA arc: partial arc needs LV_USE_DRAW_SW");
#endif
}

#endif /* LV_USE_PPA_ARC && LV_USE_PPA_BORDER */
