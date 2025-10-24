/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Initialize the board's display and show test progress screen
 */
esp_err_t example_display_init();

/**
 * @brief Set display to error screen with error code
 *
 * @param err Error code to display
 */
void example_display_failed_test_screen(const esp_err_t err);

/**
 * @brief Set display to the main screen
 *
 * @param show_test_box Opens up test result message box when true
 */
void example_display_main_screen(bool show_test_box);

/**
 * @brief Used for polling from the main task
 */
void example_display_poll();
