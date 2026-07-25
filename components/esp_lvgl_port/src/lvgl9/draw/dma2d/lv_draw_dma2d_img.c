/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP DMA2D image draw
 */

#include "lv_draw_esp_dma2d_private.h"

#if LV_USE_ESP_DMA2D

#include "draw/sw/lv_draw_sw.h"

static void lv_draw_dma2d_image_core(lv_draw_task_t *t, const lv_draw_image_dsc_t *draw_dsc,
                                     const lv_image_decoder_dsc_t *decoder_dsc, lv_draw_image_sup_t *sup,
                                     const lv_area_t *img_coords, const lv_area_t *clipped_img_area);

void LV_ATTRIBUTE_FAST_MEM lv_draw_esp_dma2d_image(lv_draw_task_t *t, const lv_draw_image_dsc_t *draw_dsc,
        const lv_area_t *coords)
{
    if (draw_dsc->opa <= (lv_opa_t)LV_OPA_MIN) {
        return;
    }
    lv_draw_image_normal_helper(t, draw_dsc, coords, lv_draw_dma2d_image_core, NULL);
}

static void LV_ATTRIBUTE_FAST_MEM lv_draw_dma2d_image_core(lv_draw_task_t *t, const lv_draw_image_dsc_t *draw_dsc,
        const lv_image_decoder_dsc_t *decoder_dsc, lv_draw_image_sup_t *sup,
        const lv_area_t *img_coords, const lv_area_t *clipped_img_area)
{
    LV_UNUSED(sup);

    lv_layer_t *layer = t->target_layer;
    lv_draw_buf_t *draw_buf = layer->draw_buf;
    const lv_draw_buf_t *decoded = decoder_dsc->decoded;
    lv_draw_dma2d_unit_t *u = (lv_draw_dma2d_unit_t *)t->draw_unit;

    if (decoded == NULL || decoded->data == NULL || draw_buf == NULL || draw_buf->data == NULL) {
        return;
    }

    lv_area_t rel_clip_area;
    lv_area_copy(&rel_clip_area, clipped_img_area);
    lv_area_move(&rel_clip_area, -img_coords->x1, -img_coords->y1);

    lv_area_t rel_img_coords;
    lv_area_copy(&rel_img_coords, img_coords);
    lv_area_move(&rel_img_coords, -img_coords->x1, -img_coords->y1);

    lv_area_t src_area;
    if (!lv_area_intersect(&src_area, &rel_clip_area, &rel_img_coords)) {
        return;
    }

    lv_area_t dst_area;
    lv_area_copy(&dst_area, clipped_img_area);
    lv_area_move(&dst_area, -layer->buf_area.x1, -layer->buf_area.y1);

    lv_color_format_t src_cf = decoded->header.cf;
    lv_color_format_t dst_cf = draw_buf->header.cf;
    uint32_t src_bpp = lv_color_format_get_bpp(src_cf);
    uint32_t dst_bpp = lv_color_format_get_bpp(dst_cf);
    if (src_bpp == 0 || dst_bpp == 0) {
        return;
    }

    uint32_t src_pic_w = (decoded->header.stride * 8U) / src_bpp;
    uint32_t dst_pic_w = (draw_buf->header.stride * 8U) / dst_bpp;
    uint32_t block_w = (uint32_t)lv_area_get_width(&src_area);
    uint32_t block_h = (uint32_t)lv_area_get_height(&src_area);

    /* Source may still hold dirty lines in the CPU cache (decoded/CPU fills).
     * Dest writeback happens in dma2d_execute_drawing; sync source here so DMA
     * reads coherent RAM. */
    lv_draw_buf_invalidate_cache((lv_draw_buf_t *)decoded, NULL);

    esp_err_t ret = lv_draw_esp_dma2d_blit(u,
                                           decoded->data, src_pic_w, decoded->header.h,
                                           src_area.x1, src_area.y1,
                                           draw_buf->data, dst_pic_w, draw_buf->header.h,
                                           dst_area.x1, dst_area.y1,
                                           block_w, block_h,
                                           src_cf, dst_cf);
    if (ret != ESP_OK) {
        /* Preferred-unit dispatch already claimed this task; SW redraw so LVGL
         * does not keep a blank/stale area after a failed DMA2D enqueue. */
        LV_LOG_WARN("DMA2D image blit failed (%d), falling back to SW", (int)ret);
        lv_draw_sw_image(t, draw_dsc, img_coords);
    }
}

#endif /* LV_USE_ESP_DMA2D */
