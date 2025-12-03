/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <inttypes.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "soc/soc_caps.h"

#include "test_i80_board.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "esp_lcd_nt35510.h"

#define TEST_RANDOM_BLOCK_SIZE (100 * 100 * sizeof(uint16_t))

void test_nt35510_draw_random_block(esp_lcd_panel_handle_t panel_handle)
{
    uint8_t *img = heap_caps_malloc(TEST_RANDOM_BLOCK_SIZE, MALLOC_CAP_DMA);
    TEST_ASSERT_NOT_NULL(img);

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    // Set inversion, x/y coordinate order, x/y mirror according to your LCD module spec
    // the gap is LCD panel specific, even panels with the same driver IC, can have different gap value
    esp_lcd_panel_swap_xy(panel_handle, true);
    esp_lcd_panel_mirror(panel_handle, true, false);
    // turn on display
    esp_lcd_panel_disp_on_off(panel_handle, true);

    for (int i = 0; i < 100; i++) {
        uint8_t color_byte = esp_random() & 0xFF;
        int x_start = esp_random() % (TEST_LCD_H_RES - 100);
        int y_start = esp_random() % (TEST_LCD_V_RES - 100);
        memset(img, color_byte, TEST_RANDOM_BLOCK_SIZE);
        esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_start + 100, y_start + 100, img);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    // turn off screen
    esp_lcd_panel_disp_on_off(panel_handle, false);

    free(img);
}

#if SOC_LCD_I80_SUPPORTED
TEST_CASE("test NT35510 with i80 interface", "[NT35510][i80]")
{
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dc_gpio_num = TEST_LCD_DC_GPIO,
        .wr_gpio_num = TEST_LCD_PCLK_GPIO,
        .data_gpio_nums = {
            TEST_LCD_DATA0_GPIO,
            TEST_LCD_DATA1_GPIO,
            TEST_LCD_DATA2_GPIO,
            TEST_LCD_DATA3_GPIO,
            TEST_LCD_DATA4_GPIO,
            TEST_LCD_DATA5_GPIO,
            TEST_LCD_DATA6_GPIO,
            TEST_LCD_DATA7_GPIO,
        },
        .bus_width = 8,
        .max_transfer_bytes = TEST_RANDOM_BLOCK_SIZE + 10,
    };
    TEST_ESP_OK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = TEST_LCD_CS_GPIO,
        .pclk_hz = TEST_LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .lcd_cmd_bits = 16,
        .lcd_param_bits = 16,
    };
    TEST_ESP_OK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = TEST_LCD_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    TEST_ESP_OK(esp_lcd_new_panel_nt35510(io_handle, &panel_config, &panel_handle));

    test_nt35510_draw_random_block(panel_handle);

    TEST_ESP_OK(esp_lcd_panel_del(panel_handle));
    TEST_ESP_OK(esp_lcd_panel_io_del(io_handle));
    TEST_ESP_OK(esp_lcd_del_i80_bus(i80_bus));
}
#endif // SOC_LCD_I80_SUPPORTED

// Some resources are lazy allocated in the LCD driver, the threshold is left for that case
#define TEST_MEMORY_LEAK_THRESHOLD (-300)

static size_t before_free_8bit;
static size_t before_free_32bit;

static void check_leak(size_t before_free, size_t after_free, const char *type)
{
    ssize_t delta = after_free - before_free;
    printf("MALLOC_CAP_%s: Before %u bytes free, After %u bytes free (delta %d)\n", type, before_free, after_free, delta);
    TEST_ASSERT_MESSAGE(delta >= TEST_MEMORY_LEAK_THRESHOLD, "memory leak");
}

void setUp(void)
{
    before_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    before_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
}

void tearDown(void)
{
    size_t after_free_8bit = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    size_t after_free_32bit = heap_caps_get_free_size(MALLOC_CAP_32BIT);
    check_leak(before_free_8bit, after_free_8bit, "8BIT");
    check_leak(before_free_32bit, after_free_32bit, "32BIT");
}

void app_main(void)
{
    unity_run_menu();
}
