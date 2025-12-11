/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/**
 * @file
 * @brief ESP LCD: ILI9341 Init Commands 1
 */

#pragma once
#include "esp_lcd_ili9341.h"

static const ili9341_lcd_init_cmd_t ili9341_lcd_init_vendor[] = {
    //  {cmd, { data }, data_size, delay_ms}
    /* Power control B */
    {0xCF, (uint8_t []){0x00, 0xC1, 0x30}, 3, 0},
    /* Power on sequence control */
    {0xED, (uint8_t []){0x64, 0x03, 0x12, 0x81}, 4, 0},
    /* Driver timing control A */
    {0xE8, (uint8_t []){0x85, 0x00, 0x78}, 3, 0},
    /* Power control A */
    {0xCB, (uint8_t []){0x39, 0x2C, 0x00, 0x34, 0x02}, 5, 0},
    /* Pump ratio control */
    {0xF7, (uint8_t []){0x20}, 1, 0},
    /* Driver timing control */
    {0xEA, (uint8_t []){0x00, 0x00}, 2, 0},
    /* Power control 1 */
    {0xC0, (uint8_t []){0x10}, 1, 0},
    /* Power control 2 */
    {0xC1, (uint8_t []){0x00}, 1, 0},
    /* VCOM control 1 */
    {0xC5, (uint8_t []){0x30, 0x30}, 2, 0},
    /* VCOM control 2 */
    {0xC7, (uint8_t []){0xB7}, 1, 0},
    /* Frame rate control */
    {0xB1, (uint8_t []){0x00, 0x1A}, 2, 0},
    /* Enable 3G */
    {0xF2, (uint8_t []){0x00}, 1, 0},
    /* Gamma set */
    {0x26, (uint8_t []){0x01}, 1, 0},
    /* Positive gamma correction */
    {0xE0, (uint8_t []){0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00}, 15, 0}, // Adjusted for ILI9341_2_DRIVER
    /* Negative gamma correction */
    {0xE1, (uint8_t []){0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F}, 15, 0}, // Adjusted for ILI9341_2_DRIVER
    /* Entry mode set */
    {0xB7, (uint8_t []){0x07}, 1, 0},
    /* Display function control */
    {0xB6, (uint8_t []){0x08, 0x82, 0x27}, 3, 0},
};
