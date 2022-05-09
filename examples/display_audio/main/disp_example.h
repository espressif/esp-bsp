/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#pragma once
#include <stdbool.h>

/**
 * @brief Example: Create basic graphics widgets
 *
 */
void disp_init(void);

/**
 * @brief Example: Set volume on arc widget
 *
 * @param[in] volume Volume to be set [0-100].
 */
void disp_set_volume(int volume);

/**
 * @brief Example: Set state of Playing checkbox
 *
 * @param[in] set Checkbox state (on/off)
 */
void disp_set_playing(bool set);

/**
 * @brief Example: Set state of Recording checkbox
 *
 * @param[in] set Checkbox state (on/off)
 */
void disp_set_recording(bool set);
