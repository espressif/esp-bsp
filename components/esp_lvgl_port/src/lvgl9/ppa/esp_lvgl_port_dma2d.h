/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_private/esp_cache_private.h"
#include "esp_memory_utils.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief A collection of configuration items for perferming a memcpy or color convert operation with 2D-DMA
 */
typedef struct {
    color_space_pixel_format_t format;
    uint8_t *pic_buf;
    uint32_t pic_w;
    uint32_t pic_h;
    uint32_t offset_x;
    uint32_t offset_y;
    uint32_t block_w;
    uint32_t block_h;
} dma2d_pic_config_t;

/**
 * @brief Perform memcpy or color convert operations using 2D-DMA module
 */
esp_err_t dma2d_color_convert(dma2d_pic_config_t *src, dma2d_pic_config_t *dest);

#ifdef __cplusplus
}
#endif
