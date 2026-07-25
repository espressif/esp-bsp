/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PPA fill draw
 */

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_PPA

#include <string.h>

#define GLYPH_SPAN_MAX_RUNS 48

typedef struct {
    int16_t x1;
    int16_t x2;
    int16_t y1;
    int16_t y2;
} glyph_run_t;

static void solid_fill(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf, const lv_area_t *rel_area,
                       uint32_t argb_color)
{
    ppa_fill_oper_config_t cfg = {0};
    cfg.fill_argb_color.val = argb_color;
    cfg.out.block_offset_x  = rel_area->x1;
    cfg.out.block_offset_y  = rel_area->y1;
    cfg.out.fill_cm         = lv_color_format_to_ppa_fill(draw_buf->header.cf);
    cfg.fill_block_w        = (uint32_t)lv_area_get_width(rel_area);
    cfg.fill_block_h        = (uint32_t)lv_area_get_height(rel_area);
    cfg.out.buffer          = draw_buf->data;
    cfg.out.buffer_size     = draw_buf->data_size;
    cfg.out.pic_w           = draw_buf->header.w;
    cfg.out.pic_h           = draw_buf->header.h;
    cfg.mode                = LV_PPA_TRANS_MODE;
    cfg.user_data           = u;

    lv_draw_ppa_begin_op(u);
    esp_err_t ret = ppa_do_fill(u->fill_client, &cfg);
    if (ret != ESP_OK) {
        lv_draw_ppa_cancel_op(u);
        LV_LOG_ERROR("PPA fill failed: %d", ret);
    }
}

static void solid_blend_tile(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf, const lv_area_t *rel_area,
                             const color_pixel_argb8888_data_t *px, lv_opa_t opa)
{
    uint32_t block_w = (uint32_t)lv_area_get_width(rel_area);
    uint32_t block_h = (uint32_t)lv_area_get_height(rel_area);
    size_t a8_len = (size_t)block_w * block_h;
    if (a8_len == 0 || a8_len > u->a8_scratch_size) {
        return;
    }

    memset(u->a8_scratch, 0xFF, a8_len);

    uint8_t dest_bpp = lv_color_format_get_bpp(draw_buf->header.cf);
    if (dest_bpp == 0) {
        return;
    }
    uint32_t dest_pic_w = (draw_buf->header.stride * 8U) / dest_bpp;

    float opa_ratio = (float)opa / 255.0f;
    if (opa_ratio <= 0.0f) {
        opa_ratio = 1.0f / 256.0f;
    }
    if (opa_ratio >= 1.0f) {
        opa_ratio = 255.0f / 256.0f;
    }

    ppa_blend_oper_config_t cfg = {0};
    cfg.in_bg.buffer = draw_buf->data;
    cfg.in_bg.pic_w  = dest_pic_w;
    cfg.in_bg.pic_h  = draw_buf->header.h;
    cfg.in_bg.block_w = block_w;
    cfg.in_bg.block_h = block_h;
    cfg.in_bg.block_offset_x = (uint32_t)rel_area->x1;
    cfg.in_bg.block_offset_y = (uint32_t)rel_area->y1;
    cfg.in_bg.blend_cm = lv_color_format_to_ppa_blend(draw_buf->header.cf);
    cfg.bg_alpha_update_mode = PPA_ALPHA_NO_CHANGE;

    cfg.in_fg.buffer = u->a8_scratch;
    cfg.in_fg.pic_w  = block_w;
    cfg.in_fg.pic_h  = block_h;
    cfg.in_fg.block_w = block_w;
    cfg.in_fg.block_h = block_h;
    cfg.in_fg.block_offset_x = 0;
    cfg.in_fg.block_offset_y = 0;
    cfg.in_fg.blend_cm = PPA_BLEND_COLOR_MODE_A8;
    cfg.fg_alpha_update_mode = PPA_ALPHA_SCALE;
    cfg.fg_alpha_scale_ratio = opa_ratio;
    cfg.fg_fix_rgb_val.b = px->b;
    cfg.fg_fix_rgb_val.g = px->g;
    cfg.fg_fix_rgb_val.r = px->r;

    cfg.out.buffer = draw_buf->data;
    cfg.out.buffer_size = draw_buf->data_size;
    cfg.out.pic_w  = dest_pic_w;
    cfg.out.pic_h  = draw_buf->header.h;
    cfg.out.block_offset_x = (uint32_t)rel_area->x1;
    cfg.out.block_offset_y = (uint32_t)rel_area->y1;
    cfg.out.blend_cm = lv_color_format_to_ppa_blend(draw_buf->header.cf);
    cfg.bg_byte_swap = lv_color_format_needs_ppa_byte_swap(draw_buf->header.cf);
    cfg.mode = LV_PPA_TRANS_MODE;
    cfg.user_data = u;

    lv_draw_ppa_begin_op(u);
    esp_err_t ret = ppa_do_blend(u->blend_client, &cfg);
    if (ret != ESP_OK) {
        lv_draw_ppa_cancel_op(u);
        LV_LOG_ERROR("PPA solid blend failed: %d", ret);
    }
}

static void solid_blend_area(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf, const lv_area_t *rel_area,
                             const color_pixel_argb8888_data_t *px, lv_opa_t opa)
{
    if (u->a8_scratch == NULL) {
        solid_fill(u, draw_buf, rel_area, px->val);
        return;
    }

    int32_t y1 = rel_area->y1;
    int32_t y2 = rel_area->y2;
    int32_t x1 = rel_area->x1;
    int32_t x2 = rel_area->x2;

    for (int32_t y = y1; y <= y2;) {
        int32_t th = y2 - y + 1;
        if (th > LV_PPA_TILE_SIZE) {
            th = LV_PPA_TILE_SIZE;
        }

        for (int32_t x = x1; x <= x2;) {
            int32_t tw = x2 - x + 1;
            if (tw > LV_PPA_TILE_SIZE) {
                tw = LV_PPA_TILE_SIZE;
            }

            lv_area_t tile = {
                .x1 = x,
                .y1 = y,
                .x2 = x + tw - 1,
                .y2 = y + th - 1,
            };
            solid_blend_tile(u, draw_buf, &tile, px, opa);
            x += tw;
        }
        y += th;
    }
}

static void glyph_blend_tile(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf, const lv_area_t *rel_area,
                             const uint8_t *mask, uint32_t mask_stride, const color_pixel_argb8888_data_t *px,
                             lv_opa_t opa)
{
    uint32_t block_w = (uint32_t)lv_area_get_width(rel_area);
    uint32_t block_h = (uint32_t)lv_area_get_height(rel_area);
    if (block_w == 0 || block_h == 0) {
        return;
    }

    uint8_t dest_bpp = lv_color_format_get_bpp(draw_buf->header.cf);
    if (dest_bpp == 0) {
        return;
    }
    uint32_t dest_pic_w = (draw_buf->header.stride * 8U) / dest_bpp;

    float opa_ratio = (float)opa / 255.0f;
    if (opa_ratio <= 0.0f) {
        opa_ratio = 1.0f / 256.0f;
    }
    if (opa_ratio >= 1.0f) {
        opa_ratio = 255.0f / 256.0f;
    }

    ppa_blend_oper_config_t cfg = {0};
    cfg.in_bg.buffer = draw_buf->data;
    cfg.in_bg.pic_w = dest_pic_w;
    cfg.in_bg.pic_h = draw_buf->header.h;
    cfg.in_bg.block_w = block_w;
    cfg.in_bg.block_h = block_h;
    cfg.in_bg.block_offset_x = (uint32_t)rel_area->x1;
    cfg.in_bg.block_offset_y = (uint32_t)rel_area->y1;
    cfg.in_bg.blend_cm = lv_color_format_to_ppa_blend(draw_buf->header.cf);
    cfg.bg_alpha_update_mode = PPA_ALPHA_NO_CHANGE;

    cfg.in_fg.buffer = (void *)mask;
    cfg.in_fg.pic_w = mask_stride;
    cfg.in_fg.pic_h = block_h;
    cfg.in_fg.block_w = block_w;
    cfg.in_fg.block_h = block_h;
    cfg.in_fg.block_offset_x = 0;
    cfg.in_fg.block_offset_y = 0;
    cfg.in_fg.blend_cm = PPA_BLEND_COLOR_MODE_A8;
    cfg.fg_fix_rgb_val.b = px->b;
    cfg.fg_fix_rgb_val.g = px->g;
    cfg.fg_fix_rgb_val.r = px->r;
    if (opa >= (lv_opa_t)LV_OPA_MAX) {
        cfg.fg_alpha_update_mode = PPA_ALPHA_NO_CHANGE;
    } else {
        cfg.fg_alpha_update_mode = PPA_ALPHA_SCALE;
        cfg.fg_alpha_scale_ratio = opa_ratio;
    }

    cfg.out.buffer = draw_buf->data;
    cfg.out.buffer_size = draw_buf->data_size;
    cfg.out.pic_w = dest_pic_w;
    cfg.out.pic_h = draw_buf->header.h;
    cfg.out.block_offset_x = (uint32_t)rel_area->x1;
    cfg.out.block_offset_y = (uint32_t)rel_area->y1;
    cfg.out.blend_cm = lv_color_format_to_ppa_blend(draw_buf->header.cf);
    cfg.bg_byte_swap = lv_color_format_needs_ppa_byte_swap(draw_buf->header.cf);
    cfg.mode = LV_PPA_TRANS_MODE;
    cfg.user_data = u;

    lv_draw_ppa_begin_op(u);
    esp_err_t ret = ppa_do_blend(u->blend_client, &cfg);
    if (ret != ESP_OK) {
        lv_draw_ppa_cancel_op(u);
        LV_LOG_ERROR("PPA glyph blend failed: %d", ret);
    }
}

static uint8_t LV_ATTRIBUTE_FAST_MEM glyph_row_spans(const uint8_t *row, int32_t row_w, glyph_run_t *runs, int16_t y)
{
    uint8_t count = 0;
    int32_t x = 0;

    while (x < row_w && count < GLYPH_SPAN_MAX_RUNS) {
        while (x < row_w && row[x] <= (uint8_t)LV_OPA_MIN) {
            x++;
        }
        if (x >= row_w) {
            break;
        }

        int32_t x_start = x;
        while (x < row_w && row[x] > (uint8_t)LV_OPA_MIN) {
            x++;
        }

        runs[count].x1 = (int16_t)x_start;
        runs[count].x2 = (int16_t)(x - 1);
        runs[count].y1 = y;
        runs[count].y2 = y;
        count++;
    }

    return count;
}

static bool LV_ATTRIBUTE_FAST_MEM glyph_runs_match(const glyph_run_t *a, const glyph_run_t *b, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++) {
        if (a[i].x1 != b[i].x1 || a[i].x2 != b[i].x2) {
            return false;
        }
    }
    return true;
}

static void LV_ATTRIBUTE_FAST_MEM glyph_flush_run(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
        const glyph_run_t *run, const uint8_t *mask, uint32_t mask_stride,
        int32_t mask_ox, int32_t mask_oy, const lv_area_t *rel,
        const color_pixel_argb8888_data_t *px, lv_opa_t opa)
{
    int32_t run_x1 = rel->x1 + run->x1;
    int32_t run_x2 = rel->x1 + run->x2;

    for (int32_t y = run->y1; y <= run->y2;) {
        int32_t th = run->y2 - y + 1;
        if (th > LV_PPA_TILE_SIZE) {
            th = LV_PPA_TILE_SIZE;
        }

        for (int32_t x = run_x1; x <= run_x2;) {
            int32_t tw = run_x2 - x + 1;
            if (tw > LV_PPA_TILE_SIZE) {
                tw = LV_PPA_TILE_SIZE;
            }

            lv_area_t tile = {
                .x1 = x,
                .y1 = y,
                .x2 = x + tw - 1,
                .y2 = y + th - 1,
            };

            const uint8_t *tile_mask = mask + (uint32_t)(mask_oy + (y - rel->y1)) * mask_stride
                                       + (uint32_t)(mask_ox + (x - rel->x1));
            glyph_blend_tile(u, draw_buf, &tile, tile_mask, mask_stride, px, opa);
            x += tw;
        }
        y += th;
    }
}

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_glyph_blend(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
        const lv_area_t *mask_area, const uint8_t *mask,
        uint32_t mask_stride, lv_color_t color, lv_opa_t opa,
        const lv_area_t *clip_area, const lv_area_t *buf_area)
{
    if (opa <= (lv_opa_t)LV_OPA_MIN || mask == NULL) {
        return;
    }

    lv_area_t blend_area;
    if (!lv_area_intersect(&blend_area, mask_area, clip_area)) {
        return;
    }

    color_pixel_argb8888_data_t px;
    px.val = lv_draw_ppa_fill_color_u32(color, LV_OPA_COVER);
    px.a = 0xFF;

    lv_area_t rel;
    lv_area_copy(&rel, &blend_area);
    lv_area_move(&rel, -buf_area->x1, -buf_area->y1);

    int32_t mask_ox = blend_area.x1 - mask_area->x1;
    int32_t mask_oy = blend_area.y1 - mask_area->y1;
    int32_t row_w = lv_area_get_width(&rel);

    glyph_run_t pending[GLYPH_SPAN_MAX_RUNS];
    glyph_run_t cur[GLYPH_SPAN_MAX_RUNS];
    uint8_t pending_cnt = 0;

    for (int16_t row = (int16_t)rel.y1; row <= (int16_t)rel.y2; row++) {
        const uint8_t *row_mask = mask + (uint32_t)(mask_oy + (row - rel.y1)) * mask_stride + (uint32_t)mask_ox;
        uint8_t cur_cnt = glyph_row_spans(row_mask, row_w, cur, row);

        if (cur_cnt == 0) {
            if (pending_cnt > 0) {
                for (uint8_t i = 0; i < pending_cnt; i++) {
                    glyph_flush_run(u, draw_buf, &pending[i], mask, mask_stride, mask_ox, mask_oy, &rel, &px, opa);
                }
                pending_cnt = 0;
            }
            continue;
        }

        if (pending_cnt > 0 && pending_cnt == cur_cnt && glyph_runs_match(pending, cur, pending_cnt)) {
            for (uint8_t i = 0; i < pending_cnt; i++) {
                pending[i].y2 = row;
            }
            continue;
        }

        for (uint8_t i = 0; i < pending_cnt; i++) {
            glyph_flush_run(u, draw_buf, &pending[i], mask, mask_stride, mask_ox, mask_oy, &rel, &px, opa);
        }

        pending_cnt = cur_cnt;
        for (uint8_t i = 0; i < pending_cnt; i++) {
            pending[i] = cur[i];
        }
    }

    for (uint8_t i = 0; i < pending_cnt; i++) {
        glyph_flush_run(u, draw_buf, &pending[i], mask, mask_stride, mask_ox, mask_oy, &rel, &px, opa);
    }
}

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_solid_op(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
        const lv_area_t *rel_area, uint32_t argb_color)
{
    color_pixel_argb8888_data_t px;
    px.val = argb_color;
    lv_opa_t opa = (lv_opa_t)px.a;
    if (opa <= (lv_opa_t)LV_OPA_MIN) {
        if (argb_color == 0u) {
            solid_fill(u, draw_buf, rel_area, 0u);
        }
        return;
    }
    if (lv_area_get_width(rel_area) <= 0 || lv_area_get_height(rel_area) <= 0) {
        return;
    }

    if (opa >= (lv_opa_t)LV_OPA_MAX) {
        px.a = 0xFF;
        solid_fill(u, draw_buf, rel_area, px.val);
        return;
    }

    solid_blend_area(u, draw_buf, rel_area, &px, opa);
}

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_fill(lv_draw_task_t *t, const lv_draw_fill_dsc_t *dsc,
        const lv_area_t *coords)
{
    if (dsc->opa <= (lv_opa_t)LV_OPA_MIN) {
        return;
    }

    lv_draw_ppa_unit_t *u = (lv_draw_ppa_unit_t *)t->draw_unit;
    lv_draw_buf_t *draw_buf = t->target_layer->draw_buf;

    lv_area_t rel_coords;
    lv_area_copy(&rel_coords, coords);
    lv_area_move(&rel_coords, -t->target_layer->buf_area.x1, -t->target_layer->buf_area.y1);

    lv_area_t rel_clip_area;
    lv_area_copy(&rel_clip_area, &t->clip_area);
    lv_area_move(&rel_clip_area, -t->target_layer->buf_area.x1, -t->target_layer->buf_area.y1);

    lv_area_t blend_area;
    if (!lv_area_intersect(&blend_area, &rel_coords, &rel_clip_area)) {
        return;
    }

    lv_draw_ppa_solid_op(u, draw_buf, &blend_area, lv_draw_ppa_fill_color_u32(dsc->color, dsc->opa));
}

#endif /* LV_USE_PPA */
