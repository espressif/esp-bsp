/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief SSD1306 driver
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include "driver/i2c.h"
#include "stdint.h"
#include "ssd1306_fonts.h"

/**
 * @brief  I2C address.
 */
#define SSD1306_I2C_ADDRESS    ((uint8_t)0x3C)

#define SSD1306_WIDTH               128
#define SSD1306_HEIGHT              64

typedef void *ssd1306_handle_t;                         /*handle of ssd1306*/

/**
 * @brief   device initialization
 *
 * @param   dev object handle of ssd1306
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */
esp_err_t ssd1306_init(ssd1306_handle_t dev);

/**
 * @brief   Create and initialization device object and return a device handle
 *
 * @param   port     I2C port object handle
 * @param   dev_addr I2C device address of device
 *
 * @return
 *     - device object handle of ssd1306
 */
ssd1306_handle_t ssd1306_create(i2c_port_t port, uint16_t dev_addr);

/**
 * @brief   Delete and release a device object
 *
 * @param   dev object handle of ssd1306
 */
void ssd1306_delete(ssd1306_handle_t dev);

/**
 * @brief   draw point on (x, y)
 *
 * @param   dev object handle of ssd1306
 * @param   chXpos Specifies the X position
 * @param   chYpos Specifies the Y position
 * @param   chPoint fill point
 */
void ssd1306_fill_point(ssd1306_handle_t dev, uint8_t chXpos, uint8_t chYpos, uint8_t chPoint);

/**
 * @brief   Draw rectangle on (x1,y1)-(x2,y2)
 *
 * @param   dev object handle of ssd1306
 * @param   chXpos1
 * @param   chYpos1
 * @param   chXpos2
 * @param   chYpos2
 * @param   chDot fill point
 */
void ssd1306_fill_rectangle(ssd1306_handle_t dev, uint8_t chXpos1, uint8_t chYpos1,
                            uint8_t chXpos2, uint8_t chYpos2, uint8_t chDot);

/**
 * @brief   display char on (x, y),and set size, mode
 *
 * @param   dev object handle of ssd1306
 * @param   chXpos Specifies the X position
 * @param   chYpos Specifies the Y position
 * @param   chSize char size
 * @param   chChr draw char
 * @param   chMode display mode
 */
void ssd1306_draw_char(ssd1306_handle_t dev, uint8_t chXpos,
                       uint8_t chYpos, uint8_t chChr, uint8_t chSize, uint8_t chMode);

/**
 * @brief   display number on (x, y),and set length, size, mode
 *
 * @param   dev object handle of ssd1306
 * @param   chXpos Specifies the X position
 * @param   chYpos Specifies the Y position
 * @param   chNum draw num
 * @param   chLen length
 * @param   chSize display size
 */
void ssd1306_draw_num(ssd1306_handle_t dev, uint8_t chXpos,
                      uint8_t chYpos, uint32_t chNum, uint8_t chLen, uint8_t chSize);

/**
 * @brief   display 1616char on (x, y)
 *
 * @param   dev object handle of ssd1306
 * @param   chXpos Specifies the X position
 * @param   chYpos Specifies the Y position
 * @param   chChar draw char
 */
void ssd1306_draw_1616char(ssd1306_handle_t dev, uint8_t chXpos, uint8_t chYpos, uint8_t chChar);

/**
 * @brief   display 3216char on (x, y)
 *
 * @param   dev object handle of ssd1306
 * @param   chXpos Specifies the X position
 * @param   chYpos Specifies the Y position
 * @param   chChar draw char
 */
void ssd1306_draw_3216char(ssd1306_handle_t dev, uint8_t chXpos, uint8_t chYpos, uint8_t chChar);

/**
 * @brief   draw bitmap on (x, y),and set width, height
 *
 * @param   dev object handle of ssd1306
 * @param   chXpos Specifies the X position
 * @param   chYpos Specifies the Y position
 * @param   pchBmp point to BMP data
 * @param   chWidth picture width
 * @param   chHeight picture heght
 */
void ssd1306_draw_bitmap(ssd1306_handle_t dev, uint8_t chXpos, uint8_t chYpos,
                         const uint8_t *pchBmp, uint8_t chWidth, uint8_t chHeight);

/**
 * @brief   draw line between two specified points
 *
 * @param   dev object handle of ssd1306
 * @param   chXpos1 Specifies the X position of the starting point of the line
 * @param   chYpos1 Specifies the Y position of the starting point of the line
 * @param   chXpos2 Specifies the X position of the ending point of the line
 * @param   chYpos2 Specifies the Y position of the ending point of the line
 */
void ssd1306_draw_line(ssd1306_handle_t dev, int16_t chXpos1, int16_t chYpos1, int16_t chXpos2, int16_t chYpos2);

/**
 * @brief   refresh dot matrix panel
 *
 * @param   dev object handle of ssd1306

 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 **/
esp_err_t ssd1306_refresh_gram(ssd1306_handle_t dev);

/**
 * @brief   Clear screen
 *
 * @param   dev object handle of ssd1306
 * @param   chFill whether fill and fill char
 **/
void ssd1306_clear_screen(ssd1306_handle_t dev, uint8_t chFill);

/**
 * @brief   Displays a string on the screen
 *
 * @param   dev object handle of ssd1306
 * @param   chXpos Specifies the X position
 * @param   chYpos Specifies the Y position
 * @param   pchString Pointer to a string to display on the screen
 * @param   chSize char size
 * @param   chMode display mode
 **/
void ssd1306_draw_string(ssd1306_handle_t dev, uint8_t chXpos, uint8_t chYpos,
                         const uint8_t *pchString, uint8_t chSize, uint8_t chMode);

#ifdef __cplusplus
}
#endif
