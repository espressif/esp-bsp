/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Shared draw-buffer cache handlers for ESP PPA/DMA2D units
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#if LV_USE_PPA || LV_USE_ESP_DMA2D

void lv_draw_esp_buf_init_handlers(void);

#endif

#ifdef __cplusplus
}
#endif
