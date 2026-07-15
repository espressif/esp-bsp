/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lv_draw_ppa_private.h
 *
 */

#ifndef LV_DRAW_PPA_PRIVATE_H
#define LV_DRAW_PPA_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
*      INCLUDES
*********************/
#include "lvgl_public.h"

#if LV_USE_PPA

#include "draw/lv_draw_private.h"
#include "display/lv_display_private.h"
#include "misc/lv_area_private.h"

/* The ppa driver depends heavily on the esp-idf headers*/
#include <sdkconfig.h>
#include <stdatomic.h>

#if (CONFIG_LV_DRAW_BUF_ALIGN != CONFIG_CACHE_L2_CACHE_LINE_SIZE)
#error "CONFIG_LV_DRAW_BUF_ALIGN must be equal to CONFIG_CACHE_L2_CACHE_LINE_SIZE!"
#endif


#ifndef CONFIG_SOC_PPA_SUPPORTED
#error "This SoC does not support PPA"
#endif

#include <driver/ppa.h>
#include <esp_heap_caps.h>
#include <esp_err.h>
#include <hal/color_hal.h>
#include <esp_cache.h>
#include <esp_log.h>
#if LV_USE_PPA_ASYNC
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif
/*********************
*      DEFINES
*********************/

/**********************
*      TYPEDEFS
**********************/
#if LV_USE_PPA_TILE_COMPOSER
/* Single intermediate buffer used to chain multi-pass PPA operations. Tiles
 * live in PSRAM and are aligned to the L2 cache line so they can serve as
 * either input or output of any PPA client. */
typedef struct {
    void *data;
    size_t size;
    bool   in_use;
} lv_draw_ppa_tile_t;
#endif

typedef struct lv_draw_ppa_unit {
    lv_draw_unit_t base_unit;
    lv_draw_task_t *task_act;
    ppa_client_handle_t srm_client;
    ppa_client_handle_t fill_client;
    ppa_client_handle_t blend_client;
    uint8_t *buf;
    /* DMA-capable A8 mask for solid-color blends when opa < LV_OPA_MAX. Tiled to
     * LV_PPA_TILE_SIZE so a single allocation covers the worst strip size. */
    uint8_t *a8_scratch;
    size_t    a8_scratch_size;
#if LV_USE_PPA_ASYNC
    /* Binary semaphore signaled from the PPA ISR when the last sub-operation of
     * the active LVGL task completes. The draw scheduler waits on this from
     * `wait_for_finish_cb` before consuming the rendered buffer. The semaphore
     * is allocated statically so the draw unit lifecycle does not depend on a
     * heap allocation, and the FreeRTOS primitives are used directly because
     * `lv_thread_sync_*` collapses into no-ops when LV_USE_OS = LV_OS_NONE,
     * which would silently break the async path on the LVGL ESP-IDF port. */
    SemaphoreHandle_t done_sem;
    StaticSemaphore_t done_sem_buffer;
    /* Number of PPA sub-operations enqueued for the current LVGL task. A single LVGL
     * draw task may decompose into several PPA ops (e.g. border = 4 fills). Each
     * ISR completion decrements the counter; reaching zero signals `done_sem`. */
    atomic_int pending_ops;
#endif
#if LV_USE_PPA_TILE_COMPOSER
    lv_draw_ppa_tile_t tiles[LV_PPA_TILE_POOL_SIZE];
    /* Round-robin cursor used by the tile allocator. */
    uint32_t tile_cursor;
    /* Tile borrowed for the active task. The release is deferred to
     * ppa_finalize_task so the buffer is not reused while the last hardware
     * pass that consumes it is still pending in the PPA queue. */
    lv_draw_ppa_tile_t *pending_tile;
#endif
#if LV_USE_PPA_RUNTIME_TUNING
    /* Cached client config so runtime retune calls preserve the unchanged
     * fields when re-registering a client. */
    ppa_client_config_t client_cfg[3];
#endif
#if LV_USE_PPA_STATS
    /* Counters live in the unit so the stats API does not need a separate
     * shadow object. They are updated atomically because the ISR completion
     * callback also bumps total_ops/failed_ops. */
    atomic_uint stat_total_tasks;
    atomic_uint stat_total_ops;
    atomic_uint stat_failed_ops;
    atomic_uint stat_max_pending;
    atomic_ullong stat_total_wait_us;
#endif
} lv_draw_ppa_unit_t;

void lv_draw_ppa_solid_op(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
                          const lv_area_t *rel_area, uint32_t argb_color);

void lv_draw_ppa_glyph_blend(lv_draw_ppa_unit_t *u, lv_draw_buf_t *draw_buf,
                             const lv_area_t *mask_area, const uint8_t *mask, uint32_t mask_stride,
                             lv_color_t color, lv_opa_t opa, const lv_area_t *clip_area,
                             const lv_area_t *buf_area);

#if LV_USE_PPA_RUNTIME_TUNING || LV_USE_PPA_STATS
/* Single-instance pointer so the runtime/stats APIs can reach the draw unit
 * without going through the LVGL draw scheduler. The pointer is set by
 * lv_draw_ppa_init and cleared by ppa_delete. */
extern lv_draw_ppa_unit_t *lv_draw_ppa_unit_instance;
#endif

#if LV_USE_PPA_ASYNC
/* Shared ISR completion callback. Exposed so the runtime tuning module can
 * re-register it on the new client handle after a re-registration. */
bool lv_draw_ppa_trans_done_cb(ppa_client_handle_t client, ppa_event_data_t *evt, void *user_data);
#endif

#if LV_USE_PPA_TILE_COMPOSER
/* Pool lifecycle and acquire/release API. The pool is always sized to
 * LV_PPA_TILE_POOL_SIZE square ARGB8888 buffers of LV_PPA_TILE_SIZE pixels. */
bool lv_draw_ppa_tile_pool_init(lv_draw_ppa_unit_t *u);
void lv_draw_ppa_tile_pool_deinit(lv_draw_ppa_unit_t *u);
lv_draw_ppa_tile_t *lv_draw_ppa_tile_acquire(lv_draw_ppa_unit_t *u);
void lv_draw_ppa_tile_release(lv_draw_ppa_unit_t *u, lv_draw_ppa_tile_t *tile);
#endif

/**********************
*  STATIC PROTOTYPES
**********************/

/**********************
* GLOBAL PROTOTYPES
**********************/

/**********************
*      MACROS
**********************/

/**********************
*   STATIC FUNCTIONS
**********************/

/* Color-format support mirrors what the PPA hardware advertises in
 * ppa_blend_color_mode_t / ppa_srm_color_mode_t / ppa_fill_color_mode_t.
 * The matching `lv_color_format_to_ppa_*` helpers below pair LVGL formats
 * with the closest hardware mode and assume the worker drives `byte_swap`
 * or `*_alpha_update_mode` when the LVGL pixel layout is not 1:1 with the
 * hardware (e.g. XRGB ignoring the 4th byte, RGB565_SWAPPED swapping bytes,
 * A8 masks pulling fg_fix_rgb_val for the colour). */

static inline bool ppa_src_cf_supported(lv_color_format_t cf)
{
    switch (cf) {
    case LV_COLOR_FORMAT_RGB565:
    case LV_COLOR_FORMAT_RGB565_SWAPPED:
    case LV_COLOR_FORMAT_RGB888:
    case LV_COLOR_FORMAT_ARGB8888:
    case LV_COLOR_FORMAT_XRGB8888:
    case LV_COLOR_FORMAT_A8:
        return true;
    default:
        return false;
    }
}

static inline bool ppa_dest_cf_supported(lv_color_format_t cf)
{
    switch (cf) {
    case LV_COLOR_FORMAT_RGB565:
    case LV_COLOR_FORMAT_RGB565_SWAPPED:
    case LV_COLOR_FORMAT_RGB888:
    case LV_COLOR_FORMAT_ARGB8888:
    case LV_COLOR_FORMAT_XRGB8888:
        return true;
    default:
        return false;
    }
}

static inline bool ppa_srm_src_cf_supported(lv_color_format_t cf)
{
    switch (cf) {
    case LV_COLOR_FORMAT_RGB565:
    case LV_COLOR_FORMAT_RGB565_SWAPPED:
    case LV_COLOR_FORMAT_RGB888:
    case LV_COLOR_FORMAT_ARGB8888:
    case LV_COLOR_FORMAT_XRGB8888:
        return true;
    default:
        return false;
    }
}

static inline bool ppa_srm_dest_cf_supported(lv_color_format_t cf)
{
    switch (cf) {
    case LV_COLOR_FORMAT_RGB565:
    case LV_COLOR_FORMAT_RGB565_SWAPPED:
    case LV_COLOR_FORMAT_RGB888:
    case LV_COLOR_FORMAT_ARGB8888:
    case LV_COLOR_FORMAT_XRGB8888:
        return true;
    default:
        return false;
    }
}

static inline uint32_t lv_draw_ppa_fill_color_u32(lv_color_t color, lv_opa_t opa)
{
    return (lv_color_to_u32(color) & 0x00FFFFFFu) | ((uint32_t)opa << 24);
}

static inline ppa_fill_color_mode_t lv_color_format_to_ppa_fill(lv_color_format_t lv_fmt)
{
    switch (lv_fmt) {
    case LV_COLOR_FORMAT_RGB565:
    case LV_COLOR_FORMAT_RGB565_SWAPPED:
        return PPA_FILL_COLOR_MODE_RGB565;
    case LV_COLOR_FORMAT_RGB888:
        return PPA_FILL_COLOR_MODE_RGB888;
    case LV_COLOR_FORMAT_ARGB8888:
    case LV_COLOR_FORMAT_XRGB8888:
        return PPA_FILL_COLOR_MODE_ARGB8888;
    default:
        return PPA_FILL_COLOR_MODE_RGB565;
    }
}

static inline ppa_blend_color_mode_t lv_color_format_to_ppa_blend(lv_color_format_t lv_fmt)
{
    switch (lv_fmt) {
    case LV_COLOR_FORMAT_RGB565:
    case LV_COLOR_FORMAT_RGB565_SWAPPED:
        return PPA_BLEND_COLOR_MODE_RGB565;
    case LV_COLOR_FORMAT_RGB888:
        return PPA_BLEND_COLOR_MODE_RGB888;
    case LV_COLOR_FORMAT_ARGB8888:
    case LV_COLOR_FORMAT_XRGB8888:
        return PPA_BLEND_COLOR_MODE_ARGB8888;
    case LV_COLOR_FORMAT_A8:
        /* A8 is only legal on the FG input of a blend; the worker that
         * uses it must populate `in_fg.blend_cm` with this value and
         * supply the colour through `fg_fix_rgb_val`. */
        return PPA_BLEND_COLOR_MODE_A8;
    default:
        return PPA_BLEND_COLOR_MODE_RGB565;
    }
}

static inline ppa_srm_color_mode_t lv_color_format_to_ppa_srm(lv_color_format_t lv_fmt)
{
    switch (lv_fmt) {
    case LV_COLOR_FORMAT_RGB565:
    case LV_COLOR_FORMAT_RGB565_SWAPPED:
        return PPA_SRM_COLOR_MODE_RGB565;
    case LV_COLOR_FORMAT_RGB888:
        return PPA_SRM_COLOR_MODE_RGB888;
    case LV_COLOR_FORMAT_ARGB8888:
    case LV_COLOR_FORMAT_XRGB8888:
        return PPA_SRM_COLOR_MODE_ARGB8888;
    default:
        return PPA_SRM_COLOR_MODE_RGB565;
    }
}

/* Returns true if the LVGL format stores the bytes in the opposite endianness
 * of what the PPA reads, so the caller can flip `byte_swap` on the matching
 * picture-block descriptor. */
static inline bool lv_color_format_needs_ppa_byte_swap(lv_color_format_t cf)
{
    return cf == LV_COLOR_FORMAT_RGB565_SWAPPED;
}

/* Common transfer mode used by every `ppa_do_*` call. With async enabled the PPA driver
 * returns immediately and signals `done_sync` from its ISR; otherwise the call blocks
 * until the engine finishes the operation (legacy behavior). */
#if LV_USE_PPA_ASYNC
#define LV_PPA_TRANS_MODE PPA_TRANS_MODE_NON_BLOCKING
#else
#define LV_PPA_TRANS_MODE PPA_TRANS_MODE_BLOCKING
#endif

/* Sub-operation accounting helpers used by every `lv_draw_ppa_*` worker.
 * `begin_op` must be called before each `ppa_do_*` enqueue, `cancel_op`
 * compensates the counter when the enqueue itself fails so the wait callback
 * does not deadlock. */
static inline void lv_draw_ppa_begin_op(lv_draw_ppa_unit_t *u)
{
#if LV_USE_PPA_ASYNC
    int prev = atomic_fetch_add(&u->pending_ops, 1);
    (void)prev;
#if LV_USE_PPA_STATS
    /* Track the peak in-flight count and total ops submitted for telemetry.
     * The CAS loop keeps `stat_max_pending` non-decreasing under ISR + task
     * concurrency. */
    atomic_fetch_add(&u->stat_total_ops, 1u);
    unsigned new_pending = (unsigned)(prev + 1);
    unsigned cur_max = atomic_load(&u->stat_max_pending);
    while (new_pending > cur_max) {
        if (atomic_compare_exchange_weak(&u->stat_max_pending, &cur_max, new_pending)) {
            break;
        }
    }
#endif
#else
    LV_UNUSED(u);
#if LV_USE_PPA_STATS
    atomic_fetch_add(&u->stat_total_ops, 1u);
#endif
#endif
}

static inline void lv_draw_ppa_cancel_op(lv_draw_ppa_unit_t *u)
{
#if LV_USE_PPA_ASYNC
    atomic_fetch_sub(&u->pending_ops, 1);
#else
    LV_UNUSED(u);
#endif
#if LV_USE_PPA_STATS
    atomic_fetch_add(&u->stat_failed_ops, 1u);
#endif
}

#define LV_DRAW_PPA_A8_SCRATCH_BYTES ((size_t)LV_PPA_TILE_SIZE * (size_t)LV_PPA_TILE_SIZE)

#define PPA_ALIGN_UP(x, align)  ((((x) + (align) - 1) / (align)) * (align))
#define PPA_PTR_ALIGN_UP(p, align) \
    ((void*)(((uintptr_t)(p) + (uintptr_t)((align) - 1)) & ~(uintptr_t)((align) - 1)))

#define PPA_ALIGN_DOWN(x, align)  ((((x) - (align) - 1) / (align)) * (align))
#define PPA_PTR_ALIGN_DOWN(p, align) \
    ((void*)(((uintptr_t)(p) - (uintptr_t)((align) - 1)) & ~(uintptr_t)((align) - 1)))

#endif /* LV_USE_PPA */

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /* LV_DRAW_PPA_PRIVATE_H */
