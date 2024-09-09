/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdbool.h>
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "driver/spi_master.h"

#include "bsp/esp-bsp.h"
#include "pretty_effect.h"

// To speed up transfers, every SPI transfer sends a bunch of lines. This define specifies how many.
// More means more memory use, but less overhead for setting up / finishing transfers
#define PARALLEL_LINES (240 / 5)
#define FRAME_BUF_SIZE (320 * PARALLEL_LINES * BSP_LCD_BITS_PER_PIXEL / 8)
#define FRAME_BUF_NUM  (2)

// The number of frames to show before rotate the graph
#define ROTATE_FRAME   30

// Simple routine to generate some patterns and send them to the LCD. Because the
// SPI driver handles transactions in the background, we can calculate the next line
// while the previous one is being sent.
static uint16_t *s_lines[FRAME_BUF_NUM];
static void display_pretty_colors(esp_lcd_panel_handle_t panel_handle)
{
    int frame = 0;
    // Indexes of the line currently being sent to the LCD and the line we're calculating
    int sending_line = 0;
    int calc_line = 0;

    // After ROTATE_FRAME frames, the image will be rotated
    while (frame <= ROTATE_FRAME) {
        frame++;
        for (int y = 0; y < 240; y += PARALLEL_LINES) {
            // Calculate a line
            pretty_effect_calc_lines(s_lines[calc_line], y, frame, PARALLEL_LINES);
            sending_line = calc_line;
            calc_line = !calc_line;
            // Send the calculated data
            esp_lcd_panel_draw_bitmap(panel_handle, 0, y, 0 + 320, y + PARALLEL_LINES, s_lines[sending_line]);
        }
    }
}

void app_main(void)
{
    bool is_rotated = false;

    // Non-graphical API can be used without any restrictions in noglib version
    ESP_ERROR_CHECK(bsp_leds_init());

    // Only API from bsp/display.h can be used in noglib version
    ESP_ERROR_CHECK(bsp_display_brightness_init());
    ESP_ERROR_CHECK(bsp_display_backlight_off());

    esp_lcd_panel_handle_t lcd_panel;
    esp_lcd_panel_io_handle_t lcd_panel_io;
    const bsp_display_config_t lcd_config = {
        .max_transfer_sz = FRAME_BUF_SIZE,
    };
    ESP_ERROR_CHECK(bsp_display_new(&lcd_config, &lcd_panel, &lcd_panel_io));

    // The LCD can be accessed through esp_lcd API
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(lcd_panel, true));
    ESP_ERROR_CHECK(bsp_display_backlight_on());

    // Allocate memory for the pixel buffers
    for (int i = 0; i < FRAME_BUF_NUM; i++) {
        s_lines[i] = heap_caps_malloc(FRAME_BUF_SIZE, MALLOC_CAP_DMA);
        assert(s_lines[i] != NULL);
    }

    // Start and rotate
    while (1) {
        // Set driver configuration to rotate 180 degrees each time
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(lcd_panel, is_rotated, is_rotated));
        // Display
        display_pretty_colors(lcd_panel);
        is_rotated = !is_rotated;

        ESP_ERROR_CHECK(bsp_led_set(BSP_LED_BLUE, is_rotated));
    }

    // Clean-up with esp_lcd API
    esp_lcd_panel_del(lcd_panel);
    esp_lcd_panel_io_del(lcd_panel_io);
    spi_bus_free(BSP_LCD_SPI_NUM);
}
