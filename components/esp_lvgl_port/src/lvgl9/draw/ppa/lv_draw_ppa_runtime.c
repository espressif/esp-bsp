/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file lv_draw_ppa_runtime.c
 *
 */

/*********************
 *      INCLUDES
 *********************/

#include "lv_draw_ppa_private.h"
#include "lv_draw_ppa.h"

#if LV_USE_PPA_RUNTIME_TUNING || LV_USE_PPA_STATS

/**********************
 *  STATIC PROTOTYPES
 **********************/

#if LV_USE_PPA_RUNTIME_TUNING
static ppa_client_handle_t *client_handle_slot(lv_draw_ppa_unit_t *u, lv_draw_ppa_client_kind_t kind);
static esp_err_t reregister_client(lv_draw_ppa_unit_t *u, lv_draw_ppa_client_kind_t kind);
static ppa_data_burst_length_t burst_to_driver(lv_draw_ppa_burst_kind_t b);
#endif

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

#if LV_USE_PPA_RUNTIME_TUNING

esp_err_t lv_draw_ppa_set_burst_length(lv_draw_ppa_client_kind_t client, lv_draw_ppa_burst_kind_t burst)
{
    lv_draw_ppa_unit_t *u = lv_draw_ppa_unit_instance;
    if (u == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (client >= LV_DRAW_PPA_CLIENT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    u->client_cfg[client].data_burst_length = burst_to_driver(burst);
    return reregister_client(u, client);
}

esp_err_t lv_draw_ppa_set_pending_trans(lv_draw_ppa_client_kind_t client, uint8_t pending)
{
    lv_draw_ppa_unit_t *u = lv_draw_ppa_unit_instance;
    if (u == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (client >= LV_DRAW_PPA_CLIENT_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (pending == 0 || pending > 512) {
        return ESP_ERR_INVALID_ARG;
    }

    u->client_cfg[client].max_pending_trans_num = pending;
    return reregister_client(u, client);
}

#endif /* LV_USE_PPA_RUNTIME_TUNING */

#if LV_USE_PPA_STATS

void lv_draw_ppa_get_stats(lv_draw_ppa_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    lv_draw_ppa_unit_t *u = lv_draw_ppa_unit_instance;
    if (u == NULL) {
        lv_memzero(out, sizeof(*out));
        return;
    }
    out->total_tasks      = atomic_load(&u->stat_total_tasks);
    out->total_ops        = atomic_load(&u->stat_total_ops);
    out->failed_ops       = atomic_load(&u->stat_failed_ops);
    out->max_pending_seen = atomic_load(&u->stat_max_pending);
    out->total_wait_us    = atomic_load(&u->stat_total_wait_us);
}

void lv_draw_ppa_reset_stats(void)
{
    lv_draw_ppa_unit_t *u = lv_draw_ppa_unit_instance;
    if (u == NULL) {
        return;
    }
    atomic_store(&u->stat_total_tasks, 0u);
    atomic_store(&u->stat_total_ops, 0u);
    atomic_store(&u->stat_failed_ops, 0u);
    atomic_store(&u->stat_max_pending, 0u);
    atomic_store(&u->stat_total_wait_us, 0ull);
}

#endif /* LV_USE_PPA_STATS */

/**********************
 *   STATIC FUNCTIONS
 **********************/

#if LV_USE_PPA_RUNTIME_TUNING

static ppa_client_handle_t *client_handle_slot(lv_draw_ppa_unit_t *u, lv_draw_ppa_client_kind_t kind)
{
    switch (kind) {
    case LV_DRAW_PPA_CLIENT_FILL:
        return &u->fill_client;
    case LV_DRAW_PPA_CLIENT_BLEND:
        return &u->blend_client;
    case LV_DRAW_PPA_CLIENT_SRM:
        return &u->srm_client;
    default:
        return NULL;
    }
}

/* Drain whatever the unit is doing right now (caller must hold the LVGL lock
 * so no new tasks are dispatched in parallel) and re-register the requested
 * PPA client with the cached configuration. The driver requires the client
 * to be idle for `ppa_unregister_client` to succeed; we satisfy that by
 * forcing the wait callback if a task is still active. */
static esp_err_t reregister_client(lv_draw_ppa_unit_t *u, lv_draw_ppa_client_kind_t kind)
{
    ppa_client_handle_t *slot = client_handle_slot(u, kind);
    if (slot == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

#if LV_USE_PPA_ASYNC
    /* Drain any in-flight work before unregistering the affected client. */
    if (u->task_act != NULL) {
        if (atomic_load(&u->pending_ops) > 0) {
            xSemaphoreTake(u->done_sem, portMAX_DELAY);
        }
        u->task_act->state = LV_DRAW_TASK_STATE_FINISHED;
        u->task_act = NULL;
    }
#endif

    esp_err_t res = ppa_unregister_client(*slot);
    if (res != ESP_OK) {
        return res;
    }

    res = ppa_register_client(&u->client_cfg[kind], slot);
    if (res != ESP_OK) {
        return res;
    }

#if LV_USE_PPA_ASYNC
    /* Re-register the ISR completion callback on the freshly-registered
     * client so async dispatch keeps working after the retune. */
    const ppa_event_callbacks_t cbs = { .on_trans_done = lv_draw_ppa_trans_done_cb };
    res = ppa_client_register_event_callbacks(*slot, &cbs);
    if (res != ESP_OK) {
        return res;
    }
#endif
    return ESP_OK;
}

static ppa_data_burst_length_t burst_to_driver(lv_draw_ppa_burst_kind_t b)
{
    switch (b) {
    case LV_DRAW_PPA_BURST_8:
        return PPA_DATA_BURST_LENGTH_8;
    case LV_DRAW_PPA_BURST_16:
        return PPA_DATA_BURST_LENGTH_16;
    case LV_DRAW_PPA_BURST_32:
        return PPA_DATA_BURST_LENGTH_32;
    case LV_DRAW_PPA_BURST_64:
        return PPA_DATA_BURST_LENGTH_64;
    case LV_DRAW_PPA_BURST_128:
        return PPA_DATA_BURST_LENGTH_128;
    default:
        return PPA_DATA_BURST_LENGTH_64;
    }
}

#endif /* LV_USE_PPA_RUNTIME_TUNING */

#endif /* LV_USE_PPA_RUNTIME_TUNING || LV_USE_PPA_STATS */
