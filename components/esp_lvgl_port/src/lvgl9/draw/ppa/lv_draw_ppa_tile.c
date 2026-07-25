/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief PPA intermediate tile pool
 */

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_PPA_TILE_COMPOSER

/*******************************************************************************
* Defines
*******************************************************************************/

#define TILE_BYTES_PER_PIXEL 4U /* tiles are always ARGB8888 */
#define TILE_SIZE_BYTES (TILE_BYTES_PER_PIXEL * (uint32_t)LV_PPA_TILE_SIZE * (uint32_t)LV_PPA_TILE_SIZE)

/*******************************************************************************
* Public API functions
*******************************************************************************/

/* Allocate `LV_PPA_TILE_POOL_SIZE` tiles in PSRAM with cache-line alignment.
 * The pool is created once at draw-unit init and lives for the lifetime of
 * the unit; release operations only flip the in-use flag. Returning false
 * lets the caller fall back to the SW path without aborting the build. */
bool LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_tile_pool_init(lv_draw_ppa_unit_t *u)
{
    u->tile_cursor = 0;
    for (uint32_t i = 0; i < LV_PPA_TILE_POOL_SIZE; i++) {
        u->tiles[i].size   = TILE_SIZE_BYTES;
        u->tiles[i].in_use = false;
        /* MALLOC_CAP_SPIRAM keeps these large buffers out of internal RAM and
         * MALLOC_CAP_DMA guarantees the address can drive the PPA's DMA. */
        u->tiles[i].data = heap_caps_aligned_calloc(CONFIG_CACHE_L2_CACHE_LINE_SIZE, 1,
                           TILE_SIZE_BYTES,
                           MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        if (u->tiles[i].data == NULL) {
            LV_LOG_ERROR("PPA tile pool: failed to allocate tile %u (%u bytes)",
                         (unsigned)i, (unsigned)TILE_SIZE_BYTES);
            lv_draw_ppa_tile_pool_deinit(u);
            return false;
        }
    }
    return true;
}

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_tile_pool_deinit(lv_draw_ppa_unit_t *u)
{
    for (uint32_t i = 0; i < LV_PPA_TILE_POOL_SIZE; i++) {
        if (u->tiles[i].data) {
            heap_caps_free(u->tiles[i].data);
            u->tiles[i].data = NULL;
        }
        u->tiles[i].size   = 0;
        u->tiles[i].in_use = false;
    }
}

/* Round-robin acquire: walks at most LV_PPA_TILE_POOL_SIZE entries starting at
 * the cursor; returns the first free slot or NULL if every tile is busy. The
 * caller is expected to release the tile before submitting more compose work
 * because the pool is small by design. */
lv_draw_ppa_tile_t *LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_tile_acquire(lv_draw_ppa_unit_t *u)
{
    for (uint32_t step = 0; step < LV_PPA_TILE_POOL_SIZE; step++) {
        uint32_t idx = (u->tile_cursor + step) % LV_PPA_TILE_POOL_SIZE;
        if (!u->tiles[idx].in_use && u->tiles[idx].data != NULL) {
            u->tiles[idx].in_use = true;
            u->tile_cursor = (idx + 1) % LV_PPA_TILE_POOL_SIZE;
            return &u->tiles[idx];
        }
    }
    return NULL;
}

void LV_ATTRIBUTE_FAST_MEM lv_draw_ppa_tile_release(lv_draw_ppa_unit_t *u, lv_draw_ppa_tile_t *tile)
{
    LV_UNUSED(u);
    if (tile == NULL) {
        return;
    }
    tile->in_use = false;
}

#endif /* LV_USE_PPA_TILE_COMPOSER */
