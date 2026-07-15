/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ESP DMA2D LVGL draw unit
 */

#pragma once

#include "lvgl_public.h"

#ifdef __cplusplus
extern "C" {
#endif

#if LV_USE_ESP_DMA2D

/*******************************************************************************
* Public API
*******************************************************************************/

void lv_draw_esp_dma2d_init(void);
void lv_draw_esp_dma2d_deinit(void);

#endif /* LV_USE_ESP_DMA2D */

#ifdef __cplusplus
}
#endif
