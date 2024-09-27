/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "led_indicator.h"
#include "bsp/esp-bsp.h"

/*********************************** Config Blink List ***********************************/
/**
 * @brief LED on
 *
 */
static const blink_step_t bsp_led_on[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 0},
    {LED_BLINK_STOP, 0, 0},
};

/**
 * @brief LED off
 *
 */
static const blink_step_t bsp_led_off[] = {
    {LED_BLINK_HOLD, LED_STATE_OFF, 0},
    {LED_BLINK_STOP, 0, 0},
};

/**
 * @brief LED blink fast
 *
 */
static const blink_step_t bsp_led_blink_fast[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 500},
    {LED_BLINK_HOLD, LED_STATE_OFF, 500},
    {LED_BLINK_LOOP, 0, 0},
};

/**
 * @brief LED blink slow
 *
 */
static const blink_step_t bsp_led_blink_slow[] = {
    {LED_BLINK_HOLD, LED_STATE_ON, 1000},
    {LED_BLINK_HOLD, LED_STATE_OFF, 1000},
    {LED_BLINK_LOOP, 0, 0},
};

/**
 * @brief LED breathe fast
 *
 */
static const blink_step_t bsp_led_breathe_fast[] = {
    {LED_BLINK_BREATHE, LED_STATE_ON, 500},
    {LED_BLINK_HOLD, LED_STATE_ON, 500},
    {LED_BLINK_BREATHE, LED_STATE_OFF, 500},
    {LED_BLINK_HOLD, LED_STATE_OFF, 500},
    {LED_BLINK_LOOP, 0, 0},
};

/**
 * @brief LED breathe slow
 *
 */
static const blink_step_t bsp_led_breathe_slow[] = {
    {LED_BLINK_BREATHE, LED_STATE_OFF, 2000},
    {LED_BLINK_BREATHE, LED_STATE_ON, 2000},
    {LED_BLINK_LOOP, 0, 0},
};

/**
 * @brief LED blink lists
 *
 */
blink_step_t const *bsp_led_blink_defaults_lists[] = {
    [BSP_LED_ON] = bsp_led_on,
    [BSP_LED_OFF] = bsp_led_off,
    [BSP_LED_BLINK_FAST] = bsp_led_blink_fast,
    [BSP_LED_BLINK_SLOW] = bsp_led_blink_slow,
    [BSP_LED_BREATHE_FAST] = bsp_led_breathe_fast,
    [BSP_LED_BREATHE_SLOW] = bsp_led_breathe_slow,
    [BSP_LED_MAX] = NULL,
};
