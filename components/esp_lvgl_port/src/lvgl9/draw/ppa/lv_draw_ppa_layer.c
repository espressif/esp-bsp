/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PPA layer composite draw
 */

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_PPA

#if LV_USE_PPA_TILE_COMPOSER
#include "draw/lv_draw_image_private.h"
#endif

/*******************************************************************************
* Public API functions
*******************************************************************************/

/* Layer composition entry point. The actual blit reuses the image worker
 * because, from the PPA point of view, drawing a finalized child layer onto
 * the parent layer is identical to blending an ARGB8888/RGB565/RGB888 image
 * source. Behavior is shared between the LV_USE_PPA_LAYER (identity
 * composer) and LV_USE_PPA_TRANSFORM (rotate/scale/mirror) gates: the routing
 * decision sits in lv_draw_ppa.c::ppa_evaluate, so this function handles both
 * paths transparently. */
void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_layer(lv_draw_task_t *t, const lv_draw_image_dsc_t *dsc,
        const lv_area_t *coords)
{
    lv_layer_t *layer_to_draw = (lv_layer_t *)dsc->src;
    if (layer_to_draw == NULL || layer_to_draw->draw_buf == NULL) {
        return;
    }

    lv_draw_image_dsc_t new_draw_dsc = *dsc;
    new_draw_dsc.src = layer_to_draw->draw_buf;
    new_draw_dsc.header = layer_to_draw->draw_buf->header;

    lv_draw_ppa_img(t, &new_draw_dsc, coords);
}

#if LV_USE_PPA_TILE_COMPOSER

/* The combined `recolor_opa > 0 && opa < MAX` case cannot be expressed in a
 * single PPA op: the engine offers only Porter-Duff OVER blending and a
 * per-transaction alpha scale, while LVGL needs `final = mix(src, recolor,
 * recolor_opa) * opa` with the recolor mix happening before the global opa
 * scale. We split the work into three hardware passes that share an
 * intermediate ARGB8888 tile from the PSRAM pool:
 *
 *   pass 1 (BLOCKING)  fill   tile := recolor (RGB), alpha=255
 *   pass 2 (BLOCKING)  blend  tile := mix(tile, src, 1 - recolor_opa)  -> tile holds the recolored layer
 *   pass 3 (NON_BLOCK) blend  dest := mix(dest, tile, opa)             -> tile released after the queue drains
 *
 * Passes 1 and 2 use BLOCKING mode because they share the tile buffer as
 * destination and the PPA driver does not guarantee ordering across clients;
 * pass 3 stays asynchronous and benefits from the same wait_for_finish_cb
 * path used by the simpler PPA workers. The active tile is owned by the draw
 * unit (`pending_tile`) and released from `ppa_finalize_task` so it is never
 * reused while the last PPA op is still in flight.
 *
 * To keep the tile single-buffered we only accept tasks whose composed area
 * fits inside `LV_PPA_TILE_SIZE`. Layers larger than that fall back to SW;
 * scanning multiple tiles per task is a future optimization.
 */

bool lv_draw_ppa_layer_recolor_opa_supported(const lv_draw_image_dsc_t *dsc)
{
    if (dsc->recolor_opa <= LV_OPA_MIN) {
        return false;
    }
    if (dsc->opa >= (lv_opa_t)LV_OPA_MAX) {
        return false;
    }
    if (dsc->blend_mode != LV_BLEND_MODE_NORMAL) {
        return false;
    }
    if (dsc->bitmap_mask_src != NULL) {
        return false;
    }
    if (dsc->clip_radius != 0) {
        return false;
    }
    if (dsc->tile != 0) {
        return false;
    }
    if (dsc->skew_x != 0 || dsc->skew_y != 0) {
        return false;
    }
    if (dsc->scale_x != LV_SCALE_NONE || dsc->scale_y != LV_SCALE_NONE) {
        return false;
    }
    if (dsc->rotation != 0) {
        return false;
    }

    const lv_layer_t *src_layer = (const lv_layer_t *)dsc->src;
    if (src_layer == NULL || src_layer->draw_buf == NULL) {
        return false;
    }

    /* Source layers with a meaningful alpha channel make the recolor mix
     * formula approximate (the Porter-Duff fallback paints the recolor color
     * where src is transparent). Restrict to opaque source formats for now. */
    lv_color_format_t src_cf = src_layer->draw_buf->header.cf;
    if (src_cf != LV_COLOR_FORMAT_RGB565 && src_cf != LV_COLOR_FORMAT_RGB888
            && src_cf != LV_COLOR_FORMAT_XRGB8888) {
        return false;
    }

    if (!ppa_dest_cf_supported(dsc->base.layer->color_format)) {
        return false;
    }
    return true;
}

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_layer_composite(lv_draw_task_t *t, const lv_draw_image_dsc_t *dsc,
        const lv_area_t *coords)
{
    LV_UNUSED(coords);
    lv_draw_ppa_unit_t *u = (lv_draw_ppa_unit_t *)t->draw_unit;
    lv_layer_t *dest_layer = t->target_layer;
    lv_draw_buf_t *dest_buf = dest_layer->draw_buf;
    lv_layer_t *src_layer  = (lv_layer_t *)dsc->src;
    lv_draw_buf_t *src_buf = src_layer->draw_buf;

    /* Translate the affected area into both buffers' coordinate spaces. */
    lv_area_t draw_area;
    if (!lv_area_intersect(&draw_area, &t->area, &t->clip_area)) {
        return;
    }

    int32_t block_w = lv_area_get_width(&draw_area);
    int32_t block_h = lv_area_get_height(&draw_area);
    if (block_w <= 0 || block_h <= 0) {
        return;
    }

    /* Tile is single-buffered ARGB8888 of side LV_PPA_TILE_SIZE: anything
     * larger requires a tile-by-tile loop that we are not implementing yet. */
    if (block_w > LV_PPA_TILE_SIZE || block_h > LV_PPA_TILE_SIZE) {
        LV_LOG_WARN("PPA composite area %" PRId32 "x%" PRId32 " exceeds tile size %d",
                    block_w, block_h, LV_PPA_TILE_SIZE);
        return;
    }

    lv_draw_ppa_tile_t *tile = lv_draw_ppa_tile_acquire(u);
    if (tile == NULL) {
        LV_LOG_WARN("PPA composite: no free tile in pool");
        return;
    }
    u->pending_tile = tile;

    int32_t dst_x = draw_area.x1 - dest_layer->buf_area.x1;
    int32_t dst_y = draw_area.y1 - dest_layer->buf_area.y1;
    int32_t src_x = draw_area.x1 - src_layer->buf_area.x1;
    int32_t src_y = draw_area.y1 - src_layer->buf_area.y1;

    uint32_t src_bpp  = lv_color_format_get_bpp(src_buf->header.cf);
    uint32_t dest_bpp = lv_color_format_get_bpp(dest_buf->header.cf);
    if (src_bpp == 0 || dest_bpp == 0) {
        return;
    }
    uint32_t src_pic_w  = (src_buf->header.stride * 8U) / src_bpp;
    uint32_t dest_pic_w = (dest_buf->header.stride * 8U) / dest_bpp;

    /* Pass 1: fill the tile with the recolor color, fully opaque. The fill
     * client only writes the block we care about, so the rest of the tile
     * may carry stale data; it does not matter because passes 2/3 only read
     * the same block. */
    {
        ppa_fill_oper_config_t cfg = {0};
        /* lv_color_to_u32 already sets alpha to 0xFF for opaque colors. */
        cfg.fill_argb_color.val = lv_color_to_u32(dsc->recolor);
        cfg.out.buffer          = tile->data;
        cfg.out.buffer_size     = tile->size;
        cfg.out.pic_w           = LV_PPA_TILE_SIZE;
        cfg.out.pic_h           = LV_PPA_TILE_SIZE;
        cfg.out.block_offset_x  = 0;
        cfg.out.block_offset_y  = 0;
        cfg.out.fill_cm         = PPA_FILL_COLOR_MODE_ARGB8888;
        cfg.fill_block_w        = (uint32_t)block_w;
        cfg.fill_block_h        = (uint32_t)block_h;
        cfg.mode                = PPA_TRANS_MODE_BLOCKING;
        cfg.user_data           = u;
        esp_err_t ret = ppa_do_fill(u->fill_client, &cfg);
        if (ret != ESP_OK) {
            LV_LOG_ERROR("PPA composite pass 1 failed: %d", ret);
            goto release;
        }
    }

    /* Pass 2: blend the source layer over the recolored tile with
     * fg_alpha_scale = (1 - recolor_opa). The bg keeps full alpha, so the
     * Porter-Duff formula collapses to `mix(recolor, src, 1 - recolor_opa)`
     * for opaque source layers (which is the only case we accept upstream). */
    float keep_src_ratio = (float)(255 - dsc->recolor_opa) / 255.0f;
    if (keep_src_ratio <= 0.0f) {
        keep_src_ratio = 1.0f / 256.0f;
    }
    if (keep_src_ratio >= 1.0f) {
        keep_src_ratio = 255.0f / 256.0f;
    }
    {
        ppa_blend_oper_config_t cfg = {0};
        cfg.in_bg.buffer = tile->data;
        cfg.in_bg.pic_w  = LV_PPA_TILE_SIZE;
        cfg.in_bg.pic_h  = LV_PPA_TILE_SIZE;
        cfg.in_bg.block_w = (uint32_t)block_w;
        cfg.in_bg.block_h = (uint32_t)block_h;
        cfg.in_bg.block_offset_x = 0;
        cfg.in_bg.block_offset_y = 0;
        cfg.in_bg.blend_cm = PPA_BLEND_COLOR_MODE_ARGB8888;
        cfg.bg_alpha_update_mode = PPA_ALPHA_NO_CHANGE;

        cfg.in_fg.buffer = src_buf->data;
        cfg.in_fg.pic_w  = src_pic_w;
        cfg.in_fg.pic_h  = src_buf->header.h;
        cfg.in_fg.block_w = (uint32_t)block_w;
        cfg.in_fg.block_h = (uint32_t)block_h;
        cfg.in_fg.block_offset_x = (uint32_t)src_x;
        cfg.in_fg.block_offset_y = (uint32_t)src_y;
        cfg.in_fg.blend_cm = lv_color_format_to_ppa_blend(src_buf->header.cf);
        cfg.fg_alpha_update_mode = PPA_ALPHA_SCALE;
        cfg.fg_alpha_scale_ratio = keep_src_ratio;

        cfg.out.buffer = tile->data;
        cfg.out.buffer_size = tile->size;
        cfg.out.pic_w  = LV_PPA_TILE_SIZE;
        cfg.out.pic_h  = LV_PPA_TILE_SIZE;
        cfg.out.block_offset_x = 0;
        cfg.out.block_offset_y = 0;
        cfg.out.blend_cm = PPA_BLEND_COLOR_MODE_ARGB8888;
        cfg.mode = PPA_TRANS_MODE_BLOCKING;
        cfg.user_data = u;

        esp_err_t ret = ppa_do_blend(u->blend_client, &cfg);
        if (ret != ESP_OK) {
            LV_LOG_ERROR("PPA composite pass 2 failed: %d", ret);
            goto release;
        }
    }

    /* Pass 3: blend the recolored tile over the destination using global opa.
     * This is the only async pass; the tile is released by ppa_finalize_task
     * once the ISR signals completion. */
    float opa_ratio = (float)dsc->opa / 255.0f;
    if (opa_ratio <= 0.0f) {
        opa_ratio = 1.0f / 256.0f;
    }
    if (opa_ratio >= 1.0f) {
        opa_ratio = 255.0f / 256.0f;
    }
    {
        ppa_blend_oper_config_t cfg = {0};
        cfg.in_bg.buffer = dest_buf->data;
        cfg.in_bg.pic_w  = dest_pic_w;
        cfg.in_bg.pic_h  = dest_buf->header.h;
        cfg.in_bg.block_w = (uint32_t)block_w;
        cfg.in_bg.block_h = (uint32_t)block_h;
        cfg.in_bg.block_offset_x = (uint32_t)dst_x;
        cfg.in_bg.block_offset_y = (uint32_t)dst_y;
        cfg.in_bg.blend_cm = lv_color_format_to_ppa_blend(dest_buf->header.cf);
        cfg.bg_alpha_update_mode = PPA_ALPHA_NO_CHANGE;

        cfg.in_fg.buffer = tile->data;
        cfg.in_fg.pic_w  = LV_PPA_TILE_SIZE;
        cfg.in_fg.pic_h  = LV_PPA_TILE_SIZE;
        cfg.in_fg.block_w = (uint32_t)block_w;
        cfg.in_fg.block_h = (uint32_t)block_h;
        cfg.in_fg.block_offset_x = 0;
        cfg.in_fg.block_offset_y = 0;
        cfg.in_fg.blend_cm = PPA_BLEND_COLOR_MODE_ARGB8888;
        cfg.fg_alpha_update_mode = PPA_ALPHA_SCALE;
        cfg.fg_alpha_scale_ratio = opa_ratio;

        cfg.out.buffer = dest_buf->data;
        cfg.out.buffer_size = dest_buf->data_size;
        cfg.out.pic_w  = dest_pic_w;
        cfg.out.pic_h  = dest_buf->header.h;
        cfg.out.block_offset_x = (uint32_t)dst_x;
        cfg.out.block_offset_y = (uint32_t)dst_y;
        cfg.out.blend_cm = lv_color_format_to_ppa_blend(dest_buf->header.cf);
        cfg.mode = LV_PPA_TRANS_MODE;
        cfg.user_data = u;

        lv_draw_ppa_begin_op(u);
        esp_err_t ret = ppa_do_blend(u->blend_client, &cfg);
        if (ret != ESP_OK) {
            lv_draw_ppa_cancel_op(u);
            LV_LOG_ERROR("PPA composite pass 3 failed: %d", ret);
            goto release;
        }
    }

    /* Tile released later by ppa_finalize_task once pass 3 completes. */
    return;

release:
    lv_draw_ppa_tile_release(u, tile);
    u->pending_tile = NULL;
}

#endif /* LV_USE_PPA_TILE_COMPOSER */

#endif /* LV_USE_PPA */
