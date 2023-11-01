/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "unity.h"
#include "unity_test_runner.h"

#include "esp_lcd_st7796.h"

#define TEST_LCD_H_RES              (320)
#define TEST_LCD_V_RES              (480)
#define TEST_LCD_BIT_PER_PIXEL      (16)
#define TEST_LCD_DATA_WIDTH         (8)

#define TEST_PIN_NUM_LCD_CS         (GPIO_NUM_17)
#define TEST_PIN_NUM_LCD_DC         (GPIO_NUM_46)
#define TEST_PIN_NUM_LCD_WR         (GPIO_NUM_3)
#define TEST_PIN_NUM_LCD_DATA0      (GPIO_NUM_9)
#define TEST_PIN_NUM_LCD_DATA1      (GPIO_NUM_12)
#define TEST_PIN_NUM_LCD_DATA2      (GPIO_NUM_11)
#define TEST_PIN_NUM_LCD_DATA3      (GPIO_NUM_14)
#define TEST_PIN_NUM_LCD_DATA4      (GPIO_NUM_13)
#define TEST_PIN_NUM_LCD_DATA5      (GPIO_NUM_47)
#define TEST_PIN_NUM_LCD_DATA6      (GPIO_NUM_21)
#define TEST_PIN_NUM_LCD_DATA7      (GPIO_NUM_45)
#define TEST_PIN_NUM_LCD_DATA8      (-1)
#define TEST_PIN_NUM_LCD_DATA9      (-1)
#define TEST_PIN_NUM_LCD_DATA10     (-1)
#define TEST_PIN_NUM_LCD_DATA11     (-1)
#define TEST_PIN_NUM_LCD_DATA12     (-1)
#define TEST_PIN_NUM_LCD_DATA13     (-1)
#define TEST_PIN_NUM_LCD_DATA14     (-1)
#define TEST_PIN_NUM_LCD_DATA15     (-1)
#define TEST_PIN_NUM_LCD_RST        (GPIO_NUM_NC)

#define TEST_DELAY_TIME_MS          (3000)

static char *TAG = "st7796_test";
static SemaphoreHandle_t refresh_finish = NULL;

IRAM_ATTR static bool test_notify_refresh_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    BaseType_t need_yield = pdFALSE;

    xSemaphoreGiveFromISR(refresh_finish, &need_yield);
    return (need_yield == pdTRUE);
}

static void test_draw_bitmap(esp_lcd_panel_handle_t panel_handle)
{
    refresh_finish = xSemaphoreCreateBinary();
    TEST_ASSERT_NOT_NULL(refresh_finish);

    uint16_t row_line = TEST_LCD_V_RES / TEST_LCD_BIT_PER_PIXEL;
    uint8_t byte_per_pixel = TEST_LCD_BIT_PER_PIXEL / 8;
    uint8_t *color = (uint8_t *)heap_caps_calloc(1, row_line * TEST_LCD_H_RES * byte_per_pixel, MALLOC_CAP_DMA);
    TEST_ASSERT_NOT_NULL(color);

    for (int j = 0; j < TEST_LCD_BIT_PER_PIXEL; j++) {
        for (int i = 0; i < row_line * TEST_LCD_H_RES; i++) {
            for (int k = 0; k < byte_per_pixel; k++) {
                color[i * byte_per_pixel + k] = (BIT(j) >> (k * 8)) & 0xff;
            }
        }
        TEST_ESP_OK(esp_lcd_panel_draw_bitmap(panel_handle, 0, j * row_line, TEST_LCD_H_RES, (j + 1) * row_line, color));
        xSemaphoreTake(refresh_finish, portMAX_DELAY);
    }
    free(color);
}

TEST_CASE("test st7796 to draw color bar with I80 interface", "[st7796][i80]")
{
    ESP_LOGI(TAG, "Initialize Intel 8080 bus");
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = ST7796_PANEL_BUS_I80_CONFIG(
            TEST_LCD_H_RES * TEST_LCD_V_RES * TEST_LCD_BIT_PER_PIXEL / 8, TEST_LCD_DATA_WIDTH,
            TEST_PIN_NUM_LCD_DC, TEST_PIN_NUM_LCD_WR,
            TEST_PIN_NUM_LCD_DATA0, TEST_PIN_NUM_LCD_DATA1, TEST_PIN_NUM_LCD_DATA2, TEST_PIN_NUM_LCD_DATA3,
            TEST_PIN_NUM_LCD_DATA4, TEST_PIN_NUM_LCD_DATA5, TEST_PIN_NUM_LCD_DATA6, TEST_PIN_NUM_LCD_DATA7,
            TEST_PIN_NUM_LCD_DATA8, TEST_PIN_NUM_LCD_DATA9, TEST_PIN_NUM_LCD_DATA10, TEST_PIN_NUM_LCD_DATA11,
            TEST_PIN_NUM_LCD_DATA12, TEST_PIN_NUM_LCD_DATA13, TEST_PIN_NUM_LCD_DATA14, TEST_PIN_NUM_LCD_DATA15);
    TEST_ESP_OK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i80_config_t io_config = ST7796_PANEL_IO_I80_CONFIG(TEST_PIN_NUM_LCD_CS, test_notify_refresh_ready, NULL);
    TEST_ESP_OK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install ST7796 panel driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = TEST_PIN_NUM_LCD_RST,
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
#else
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
#endif
        .bits_per_pixel = TEST_LCD_BIT_PER_PIXEL,
    };
    esp_lcd_panel_handle_t panel_handle = NULL;
    TEST_ESP_OK(esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle));
    TEST_ESP_OK(esp_lcd_panel_reset(panel_handle));
    TEST_ESP_OK(esp_lcd_panel_init(panel_handle));
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    TEST_ESP_OK(esp_lcd_panel_disp_off(panel_handle, false));
#else
    TEST_ESP_OK(esp_lcd_panel_disp_on_off(panel_handle, true));
#endif

    test_draw_bitmap(panel_handle);
    vTaskDelay(pdMS_TO_TICKS(TEST_DELAY_TIME_MS));

    TEST_ESP_OK(esp_lcd_panel_del(panel_handle));
    TEST_ESP_OK(esp_lcd_panel_io_del(io_handle));
    TEST_ESP_OK(esp_lcd_del_i80_bus(i80_bus));
}

// Some resources are lazy allocated in the LCD driver, the threadhold is left for that case
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
    /**
    *   __  _____ _____ _____ ___   __
    *  / _\/__   \___  |___  / _ \ / /_
    *  \ \   / /\/  / /   / / (_) | '_ \
    *  _\ \ / /    / /   / / \__, | (_) |
    *  \__/ \/    /_/   /_/    /_/ \___/
    */
    printf(" __  _____ _____ _____ ___   __\r\n");
    printf("/ _\\/__   \\___  |___  / _ \\ / /_\r\n");
    printf("\\ \\   / /\\/  / /   / / (_) | '_ \\\r\n");
    printf("_\\ \\ / /    / /   / / \\__, | (_) |\r\n");
    printf("\\__/ \\/    /_/   /_/    /_/ \\___/\r\n");
    unity_run_menu();
}
