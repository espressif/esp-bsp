/*
 * SPDX-FileCopyrightText: 2024-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Compatibility shim for out-of-tree draw units
 *
 * Recent LVGL trees expose `src/lvgl_public.h`. Older LVGL 9 releases used
 * by the IDF Component Registry may not. Prefer the public LVGL header so
 * the port builds against either.
 */

#pragma once

#include "lvgl.h"
