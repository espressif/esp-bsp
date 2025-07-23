/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

// Full screen test controls
#define MIRROR_NO 1
#define MIRROR_X 1
#define MIRROR_Y 1
#define MIRROR_XY 1

// second part of test, smaller than full screen, move images around
#define SHOW_PENGUIN 1

#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ssd1681.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"

//#include "ssd1681_waveshare_1in54_lut.h"
#include "img_bitmap.h"

// Test Images
#if WAVE_154
#define FULL_IMAGE  BITMAP_200_200
#define FULL_INVERT false
#elif WAVE_27
//#define FULL_IMAGE  BITMAP_176_264
#define FULL_IMAGE  TEXT_176_264
#define FULL_INVERT true    // the test image was constructed wrong with WHITE=0xff and BLACK=0x00
#endif

// SPI Bus
#define EPD_PANEL_SPI_CLK           1000000
#define EPD_PANEL_SPI_CMD_BITS      8
#define EPD_PANEL_SPI_PARAM_BITS    8
#define EPD_PANEL_SPI_MODE          0

// ESP32 GPIO pins used
#if 0
// e-Paper GPIO
#define EXAMPLE_PIN_NUM_EPD_DC      9
#define EXAMPLE_PIN_NUM_EPD_RST     4
#define EXAMPLE_PIN_NUM_EPD_CS      10
#define EXAMPLE_PIN_NUM_EPD_BUSY    18
// e-Paper SPI
#define EXAMPLE_PIN_NUM_MOSI        7
#define EXAMPLE_PIN_NUM_SCLK        6
#endif
#if 0
// e-Paper GPIO with ESP32C6
#define EXAMPLE_PIN_NUM_EPD_DC      8
#define EXAMPLE_PIN_NUM_EPD_RST     10
#define EXAMPLE_PIN_NUM_EPD_CS      5
#define EXAMPLE_PIN_NUM_EPD_BUSY    11
// e-Paper SPI
#define EXAMPLE_PIN_NUM_MOSI        15
#define EXAMPLE_PIN_NUM_SCLK        18
#endif
#if 1
// e-Paper GPIO with Waveshare ESP32S3
#define EXAMPLE_PIN_NUM_EPD_DC      8
#define EXAMPLE_PIN_NUM_EPD_RST     10
#define EXAMPLE_PIN_NUM_EPD_CS      5
#define EXAMPLE_PIN_NUM_EPD_BUSY    11
// e-Paper SPI
#define EXAMPLE_PIN_NUM_MOSI        15
#define EXAMPLE_PIN_NUM_SCLK        18
#endif

static const char *TAG = "epaper_demo_plain";

static bool give_semaphore_in_isr(const esp_lcd_panel_handle_t handle, const void *edata, void *user_data)
{
    SemaphoreHandle_t *epaper_panel_semaphore_ptr = user_data;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(*epaper_panel_semaphore_ptr, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
        return true;
    }
    return false;
}

// Copy a bitmap image to display buffer. Not used when FULL_IMAGE is same size as screen.
// Also the display buffer is usually assumed to be full screen.
uint8_t* crop_bitmap(const uint8_t* in_bitmap, int in_x, int in_y, int out_x, int out_y)
{
    out_x = (out_x < in_x) ? out_x : in_x;
    int b_dup = 0;
    if (out_y > in_y) b_dup = 1;

    ESP_LOGD(TAG, "out_x = %d, out_y = %d", out_x, out_y);
    uint8_t *out_bitmap = heap_caps_malloc(DISPLAY_W * DISPLAY_H / 8, MALLOC_CAP_DMA);
    memset(out_bitmap, 0, DISPLAY_W * DISPLAY_H / 8);
    if (in_x == out_x && in_y == out_y) {
        memcpy(out_bitmap, in_bitmap, in_x * in_y / 8);
    }
    else {
        uint8_t *out_ptr = out_bitmap;
        uint8_t *in_ptr = in_bitmap;
        ESP_LOGD(TAG, "in_bitmap = %p", (void *)in_bitmap);
        ESP_LOGD(TAG, "out_bitmap = %p", (void *)out_bitmap);
        // row index
        for (int iy = 0; iy < out_y; iy++) {
            if (b_dup && iy >= in_y) { in_ptr = in_bitmap; b_dup = 0;}
            // ix is column index. Inner loop copies a single row.
            for (int ix = 0; ix < out_x/8; ix++) {
                *out_ptr = *in_ptr;
                out_ptr++; in_ptr++;
            }
            //ESP_LOGI(TAG, "crop_bitmap: row = %d, in_ptr idx = %d", iy, in_ptr-in_bitmap);
            in_ptr += (in_x - out_x)/8;
        }
        ESP_LOGD(TAG, "crop_bitmap copied %d bytes", (out_ptr - out_bitmap));
    }
    return out_bitmap;
}

void app_main(void)
{
    esp_err_t ret;
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    // --- Init SPI Bus
    ESP_LOGI(TAG, "Initializing SPI Bus...");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    // --- Init ESP_LCD IO
    ESP_LOGI(TAG, "Initializing panel IO...");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_EPD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_EPD_CS,
        .pclk_hz = EPD_PANEL_SPI_CLK,
        .lcd_cmd_bits = EPD_PANEL_SPI_CMD_BITS,
        .lcd_param_bits = EPD_PANEL_SPI_PARAM_BITS,
        .spi_mode = EPD_PANEL_SPI_MODE,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) SPI2_HOST, &io_config, &io_handle));
    // --- Create esp_lcd panel
    ESP_LOGI(TAG, "Creating SSD1681 panel...");
    esp_lcd_ssd1681_config_t epaper_ssd1681_config = {
        .busy_gpio_num = EXAMPLE_PIN_NUM_EPD_BUSY,
        // NOTE: Enable this to reduce one buffer copy if you do not use swap-xy, mirror y or invert color
        // since those operations are not supported by ssd1681 and are implemented by software
        // Better use DMA-capable memory region, to avoid additional data copy
        .non_copy_mode = false,
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_EPD_RST,
        .flags.reset_active_high = false,
        .vendor_config = &epaper_ssd1681_config
    };
    esp_lcd_panel_handle_t panel_handle = NULL;
    // NOTE: Please call gpio_install_isr_service() manually before esp_lcd_new_panel_ssd1681()
    // because gpio_isr_handler_add() is called in esp_lcd_new_panel_ssd1681()
    gpio_install_isr_service(0);
    ret = esp_lcd_new_panel_ssd1681(io_handle, &panel_config, &panel_handle);
    ESP_ERROR_CHECK(ret);
    // --- Reset the display
    ESP_LOGI(TAG, "Resetting e-Paper display...");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // --- Initialize LCD panel
    ESP_LOGI(TAG, "Initializing e-Paper display...");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // Turn on the screen
    ESP_LOGI(TAG, "Turning e-Paper display on...");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    // Set custom lut
    // NOTE: Setting custom LUT is not necessary. Panel built-in LUT is used calling after esp_lcd_panel_disp_on_off()
    // NOTE: Uncomment code below to see difference between full refresh & fast refresh
    // NOTE: epaper_panel_set_custom_lut() must be called AFTER calling esp_lcd_panel_disp_on_off()
    // static uint8_t fast_refresh_lut[] = SSD1681_WAVESHARE_1IN54_V2_LUT_FAST_REFRESH_KEEP;
    // ESP_ERROR_CHECK(epaper_panel_set_custom_lut(panel_handle, fast_refresh_lut, 159));

    vTaskDelay(100 / portTICK_PERIOD_MS);

    static SemaphoreHandle_t epaper_panel_semaphore;
    epaper_panel_semaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(epaper_panel_semaphore);

    // --- Register the e-Paper refresh done callback
    // cbs does not have to be static for ssd1681 driver, for the callback ptr is copied, not pointed
    epaper_panel_callbacks_t cbs = {
        .on_epaper_refresh_done = give_semaphore_in_isr,
    };
    epaper_panel_register_event_callbacks(panel_handle, &cbs, &epaper_panel_semaphore);

    // --- Clear the VRAM of RED and BLACK
    ESP_LOGI(TAG, "Clear Screen");
    uint8_t *empty_bitmap = heap_caps_malloc(DISPLAY_W * DISPLAY_H / 8, MALLOC_CAP_DMA);
    memset(empty_bitmap, 0, DISPLAY_W * DISPLAY_H / 8);
#if 0   // no RED on my panel
    epaper_panel_set_bitmap_color(panel_handle, SSD1681_EPAPER_BITMAP_RED);
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_W, DISPLAY_H, empty_bitmap);
#endif
    epaper_panel_set_bitmap_color(panel_handle, SSD1681_EPAPER_BITMAP_BLACK);
    esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_W, DISPLAY_H, empty_bitmap);
    // refresh screen just so I can see that it is blank - not required
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
    // Maybe SPI operation is still going on, so set a delay
    vTaskDelay(pdMS_TO_TICKS(5000));
    heap_caps_free(empty_bitmap);

    // --- Draw full-screen bitmap
    ESP_LOGI(TAG, "Drawing bitmap...");
    ESP_LOGI(TAG, "Show image full-screen");
    uint8_t *crop_image = heap_caps_malloc(DISPLAY_W * DISPLAY_H / 8, MALLOC_CAP_DMA);
    memset(crop_image, 0, DISPLAY_W * DISPLAY_H / 8);
    memcpy(crop_image, FULL_IMAGE, DISPLAY_W * DISPLAY_H / 8);
#if MIRROR_NO
    ESP_LOGI(TAG, "Draw image NO Mirror");
    xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, FULL_INVERT));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_LOGI(TAG, "Drawing bitmap...");
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_W, DISPLAY_H, crop_image));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
#endif
#if MIRROR_Y
    ESP_LOGI(TAG, "Mirror Y axis");
    vTaskDelay(pdMS_TO_TICKS(5000));
    xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, FULL_INVERT));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_LOGI(TAG, "Drawing bitmap...");
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_W, DISPLAY_H, crop_image));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
#endif
#if MIRROR_X
    ESP_LOGI(TAG, "Mirror X axis");
    vTaskDelay(pdMS_TO_TICKS(5000));
    xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, FULL_INVERT));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_LOGI(TAG, "Drawing bitmap...");
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_W, DISPLAY_H, crop_image));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
#endif
#if MIRROR_XY
    ESP_LOGI(TAG, "Mirror X&Y axis");
    vTaskDelay(pdMS_TO_TICKS(5000));
    xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, FULL_INVERT));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_LOGI(TAG, "Drawing bitmap...");
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_W, DISPLAY_H, crop_image));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
#endif
#if SQUARE_PANEL   // swap_xy tests: not implemented for non-square panels
    // TODO: if display is normally portrait, then test swap_xy by inputting a landscape image
    xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_LOGI(TAG, "Drawing bitmap...");
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_W, DISPLAY_H, BITMAP_200_200));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));

    xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_LOGI(TAG, "Drawing bitmap...");
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_W, DISPLAY_H, BITMAP_200_200));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));

    xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_LOGI(TAG, "Drawing bitmap...");
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, DISPLAY_W, DISPLAY_H, BITMAP_200_200));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
#endif
    heap_caps_free(crop_image);

#if SHOW_PENGUIN
    // Show penguin images that are smaller than full screen.
    // This code does not clear the display so the previous full screen image remains and goes thru
    // various XY mirrors. Furthermore the full screen image may have been displayed with invert_color,
    // if so it will not be inverted here. 
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Go to sleep mode...");
    esp_lcd_panel_disp_on_off(panel_handle, false);
    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGI(TAG, "e-Paper resuming...");
    esp_lcd_panel_reset(panel_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    esp_lcd_panel_init(panel_handle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    esp_lcd_panel_disp_on_off(panel_handle, true);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    // NOTE: If you want to use a custom LUT, you'll have to set it again after resume
    // --- Draw partial bitmap
    ESP_LOGI(TAG, "Show image partial");
    xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_LOGI(TAG, "Drawing bitmap...");
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 16, 0, 80, 128, BITMAP_64_128));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));

    xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_LOGI(TAG, "Drawing bitmap...");
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 16, 64, 144, 128, BITMAP_128_64));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));

    xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_LOGI(TAG, "Drawing bitmap...");
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 16, 64, 144, 128, BITMAP_128_64));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));

    xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_LOGI(TAG, "Drawing bitmap...");
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, 16, 0, 144, 64, BITMAP_128_64));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
#endif
    vTaskDelay(pdMS_TO_TICKS(5000));
    ESP_LOGI(TAG, "Go to sleep mode...");
    esp_lcd_panel_disp_on_off(panel_handle, false);
}
