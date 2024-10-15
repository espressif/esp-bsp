/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_heap_caps.h"
#include "lvgl.h"

#if LV_USE_STDLIB_MALLOC == LV_STDLIB_CUSTOM
void lv_mem_init(void)
{
    return; /*Nothing to init*/
}

void lv_mem_deinit(void)
{
    return; /*Nothing to deinit*/
}

lv_mem_pool_t lv_mem_add_pool(void *mem, size_t bytes)
{
    /*Not supported*/
    LV_UNUSED(mem);
    LV_UNUSED(bytes);
    return NULL;
}

void lv_mem_remove_pool(lv_mem_pool_t pool)
{
    /*Not supported*/
    LV_UNUSED(pool);
    return;
}

void *lv_malloc_core(size_t size)
{
    if (size > 128 * 1024) {
        return heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
    }
    return malloc(size);
}

void *lv_realloc_core(void *p, size_t new_size)
{
    if (new_size > 128 * 1024) {
        free(p);
        return heap_caps_malloc(new_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_CACHE_ALIGNED);
    }
    return realloc(p, new_size);
}

void lv_free_core(void *p)
{
    free(p);
}

void lv_mem_monitor_core(lv_mem_monitor_t *mon_p)
{
    /*Not supported*/
    LV_UNUSED(mon_p);
}

lv_result_t lv_mem_test_core(void)
{
    /*Not supported*/
    return LV_RESULT_OK;
}
#endif // LV_STDLIB_CUSTOM
