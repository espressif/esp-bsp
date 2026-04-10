/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "sdkconfig.h"

/**************************************************************************************************
 * BSP configuration
 **************************************************************************************************/
/* LVGL integration: default on; disable via menuconfig (CONFIG_BSP_LVGL_INTEGRATION). */
#if CONFIG_BSP_LVGL_INTEGRATION
#define BSP_CONFIG_NO_GRAPHIC_LIB (0)
#else
#define BSP_CONFIG_NO_GRAPHIC_LIB (1)
#endif
