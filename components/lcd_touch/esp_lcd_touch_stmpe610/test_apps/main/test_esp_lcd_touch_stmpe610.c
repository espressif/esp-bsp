/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <inttypes.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "unity.h"
#include "unity_test_runner.h"

#include "esp_lcd_touch_stmpe610.h"

#define TEST_LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define TEST_PIN_NUM_SCLK           18
#define TEST_PIN_NUM_MOSI           19
#define TEST_PIN_NUM_MISO           21
#define TEST_PIN_NUM_TOUCH_CS       15

#define TEST_LCD_H_RES              (240)
#define TEST_LCD_V_RES              (240)

/* LCD touch pins */
#define TEST_TOUCH_I2C_SCL       (GPIO_NUM_18)
#define TEST_TOUCH_I2C_SDA       (GPIO_NUM_8)
#define TEST_TOUCH_GPIO_INT      (GPIO_NUM_3)

TEST_CASE("test stmpe610 to initialize touch", "[stmpe610][i2c]")
{
    /* Initilize SPI bus */
    spi_bus_config_t buscfg = {
        .sclk_io_num = TEST_PIN_NUM_SCLK,
        .mosi_io_num = TEST_PIN_NUM_MOSI,
        .miso_io_num = TEST_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TEST_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    TEST_ESP_OK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config = ESP_LCD_TOUCH_IO_SPI_STMPE610_CONFIG(TEST_PIN_NUM_TOUCH_CS);
    // Attach the TOUCH to the SPI bus
    TEST_ESP_OK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = TEST_LCD_H_RES,
        .y_max = TEST_LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_touch_handle_t tp = NULL;

    TEST_ESP_OK(esp_lcd_touch_new_spi_stmpe610(tp_io_handle, &tp_cfg, &tp));
}

void app_main(void)
{
    printf("STMPE610 Test\r\n");
    unity_run_menu();
}
