/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TEST_LCD_H_RES          240
#define TEST_LCD_V_RES          280

#define TEST_LCD_PIXEL_CLOCK_HZ 8000000  // 8MHz

#if CONFIG_IDF_TARGET_ESP32S3
#define TEST_LCD_BK_LIGHT_GPIO (18)
#define TEST_LCD_RST_GPIO      (5)
#define TEST_LCD_CS_GPIO       (0)
#define TEST_LCD_DC_GPIO       (19)
#define TEST_LCD_PCLK_GPIO     (2)
#define TEST_LCD_DATA0_GPIO    (4)
#define TEST_LCD_DATA1_GPIO    (7)
#define TEST_LCD_DATA2_GPIO    (8)
#define TEST_LCD_DATA3_GPIO    (9)
#define TEST_LCD_DATA4_GPIO    (10)
#define TEST_LCD_DATA5_GPIO    (11)
#define TEST_LCD_DATA6_GPIO    (12)
#define TEST_LCD_DATA7_GPIO    (13)
#elif CONFIG_IDF_TARGET_ESP32S2
#define TEST_LCD_BK_LIGHT_GPIO (0)
#define TEST_LCD_RST_GPIO      (18)
#define TEST_LCD_CS_GPIO       (19)
#define TEST_LCD_DC_GPIO       (38)
#define TEST_LCD_PCLK_GPIO     (33)
#define TEST_LCD_DATA0_GPIO    (1)
#define TEST_LCD_DATA1_GPIO    (10)
#define TEST_LCD_DATA2_GPIO    (2)
#define TEST_LCD_DATA3_GPIO    (11)
#define TEST_LCD_DATA4_GPIO    (3)
#define TEST_LCD_DATA5_GPIO    (12)
#define TEST_LCD_DATA6_GPIO    (4)
#define TEST_LCD_DATA7_GPIO    (13)
#elif CONFIG_IDF_TARGET_ESP32
#define TEST_LCD_BK_LIGHT_GPIO (2)
#define TEST_LCD_RST_GPIO      (-1)
#define TEST_LCD_CS_GPIO       (4)
#define TEST_LCD_DC_GPIO       (5)
#define TEST_LCD_PCLK_GPIO     (18)
#define TEST_LCD_DATA0_GPIO    (19)
#define TEST_LCD_DATA1_GPIO    (21)
#define TEST_LCD_DATA2_GPIO    (0)
#define TEST_LCD_DATA3_GPIO    (22)
#define TEST_LCD_DATA4_GPIO    (23)
#define TEST_LCD_DATA5_GPIO    (33)
#define TEST_LCD_DATA6_GPIO    (32)
#define TEST_LCD_DATA7_GPIO    (27)
#endif

// SPI2 supports octal mode, which can be used to emulate the i80 bus with 8 data lines
#define TEST_SPI_HOST_ID  SPI2_HOST

#ifdef __cplusplus
}
#endif
