/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <esp_lcd_panel_ssd1681.h>
#include <esp_lcd_ssd1681_commands.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "lvgl.h"

#include "ssd1681_waveshare_1in54_lut.h"

static const char *TAG = "main";

// Program displays screen with each of these rotation values
const uint32_t ROTATE_TEST[] = {
    LV_DISP_ROT_NONE,
    LV_DISP_ROT_90,
    LV_DISP_ROT_180,
    LV_DISP_ROT_270
};

#define TEST_DELAY  15000   // time between each rotate of display, milliseconds

// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configurations according to your LCD spec /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)

// ------ Waveshare or GooDisplay 2.7" B&W with SSD1680 controller ------ //
#define WAVE_27 1

// The pixel dimension in horizontal and vertical
#if defined(WAVE_27) && WAVE_27==1
#define LCD_H_RES              176
#define LCD_V_RES              264
#define SQUARE_DISP 0
#define BLACK_RED_DISP  0       // don't know if this example supports it
#elif defined(WAVE_154) && WAVE_154==1
#define LCD_H_RES              200
#define LCD_V_RES              200
#define SQUARE_DISP 1
#define BLACK_RED_DISP  1
#endif

#define DISPLAY_X   LCD_H_RES
#define DISPLAY_Y   LCD_V_RES

// ----- GPIO pin assignments ----- //
#if 0
#define PIN_NUM_SCLK           6
#define PIN_NUM_MOSI           7
#define PIN_NUM_MISO           (-1)   // Unused
#define PIN_NUM_EPD_DC         9
#define PIN_NUM_EPD_RST        4
#define PIN_NUM_EPD_CS         10
#define PIN_NUM_EPD_BUSY       18
#endif
#if 1
// e-Paper GPIO with Waveshare ESP32S3
#define PIN_NUM_EPD_DC      8
#define PIN_NUM_EPD_RST     10
#define PIN_NUM_EPD_CS      5
#define PIN_NUM_EPD_BUSY    11
// e-Paper SPI
#define PIN_NUM_MOSI        15
#define PIN_NUM_SCLK        18
#define PIN_NUM_MISO           (-1)   // Unused
#endif

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

#define LVGL_TICK_PERIOD_MS    2

static uint8_t *converted_buffer_black;
static uint8_t *converted_buffer_red;
#define TRAVERSE_COLS   0   // experiment if ==1, traverse columns

static SemaphoreHandle_t panel_refreshing_sem = NULL;

extern void lvgl_canvas_ui(int xdim, int ydim, uint32_t rotate);

IRAM_ATTR bool epaper_flush_ready_callback(const esp_lcd_panel_handle_t handle, const void *edata, void *user_data)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *) user_data;
    lv_disp_flush_ready(disp_driver);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(panel_refreshing_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        return true;
    }
    return false;
}

// Can rotate input buffer by 0, 90, 180 or 270 degrees.
// The buffers represent the display as an array of bytes, each byte contains 8 monochrome pixels in a row.
// xlen: number of pixels in a row.
// ylen: number of rows.
// Therefore each row has xlen/8 bytes.
void rotate_buffer(uint32_t rotate, int xlen, int ylen, uint8_t *in_buf, uint8_t *out_buf)
{
    esp_log_level_set("rotate_buffer", ESP_LOG_DEBUG);
    ESP_LOGD("rotate_buffer", "in_buf: xlen=%d, ylen=%d", xlen, ylen);
    const int MAX_ERR_MSG = 5;
    int bad0 = 0, bad1 = 0;     // record any mistakes where the index is out of bounds (I think this is completely debugged now)
    memset(out_buf, 0x0, xlen * ylen / 8);

    if (rotate == LV_DISP_ROT_90 || rotate == LV_DISP_ROT_270) {
        // -------  Code for 90 degree rotation  ---------- //
        // ---- 270 rotation uses this code followed by mirror on both X&Y axes ---- //
        int in_idx, out_idx;
        // iterate over columns of in_buf
        for (int ix = 0; ix < xlen / 8; ix++) {
            // iterate over rows of in_buf
            for (int iy = 0; iy < ylen; iy += 1) {
                in_idx = (iy * xlen) / 8 + ix;
                if (in_idx >= (xlen * ylen) / 8 || in_idx < 0) {
                    // BIG surprise if this ever hahpens!
                    ESP_LOGE("rotate_buffer", "in_buf: out-of-bounds: in_idx = %d, ix = %d, iy = %d", in_idx, ix, iy);
                    continue;
                }
                uint8_t inbits = in_buf[in_idx];
                // with 90 rotate, idx0 is the out_buf location for the first bit (bit7 of inbits)
                int idx0 = (xlen - ix * 8 - 1) * ylen / 8 + iy / 8;
                // iterate over bits
                for (int ii = 0; ii < 8; ii++) {
                    out_idx = idx0 - ii * ylen / 8; // each bit from inbits translates to the next row up in out_buf
                    if (out_idx >= (xlen * ylen) / 8) {
                        // When I was debugging this I had instances of impossible values
                        if (bad1 < MAX_ERR_MSG) {
                            ESP_LOGE("rotate_buffer", "out_idx = %d, ix = %d, iy = %d, ii = %d", out_idx, ix, iy, ii);
                        }
                        bad1++;
                        continue;
                    }
                    if (out_idx < 0) {
                        // When I was debugging this I had instances of impossible values
                        if (bad0 < MAX_ERR_MSG) {
                            ESP_LOGE("rotate_buffer", "out_idx = %d, ix = %d, iy = %d, ii = %d", out_idx, ix, iy, ii);
                        }
                        bad0++;
                        continue;
                    }
                    uint8_t inbit = (inbits & (1 << (7 - ii))) ? 0x1 : 0x0; // the current bit
                    out_buf[ out_idx ] |= inbit << (7 - iy % 8);
                }
            }
        }
        if (bad0 || bad1) {
            ESP_LOGE("rotate_buffer", "out_buf: out-of-bounds: idx < 0: %d, idx > max: %d", bad0, bad1);
        }
    } else {
        // Either 0 or 180 degree rotate which is accomplished by using function mirror_xy elsewhere in code
        // Do nothing here, just copy in_buf to out_buf
        memcpy(out_buf, in_buf, (xlen * ylen) / 8);
    }
#if 0   // print entire buffer for debugging
#if 0   // print in_buf
    // write buffer to stdout
    for (int iy = 0; iy < ylen; iy++) {
        //printf("%3d: ", iy);
        for (int ix = 0; ix < xlen / 8; ix++) {
            printf("0x%02x, ", in_buf[ix + iy * xlen / 8]);
        }
        printf("\n");
    }
#else// prin out_buf
    int itmp = xlen;
    xlen = ylen;
    ylen = itmp;
    for (int iy = 0; iy < ylen; iy++) {
        //printf("%3d: ", iy);
        for (int ix = 0; ix < xlen / 8; ix++) {
            printf("0x%02x, ", out_buf[ix + iy * xlen / 8]);
        }
        printf("\n");
    }
#endif
#endif
    ESP_LOGD("rotate_buffer", "OK DONE");
}

/**
 * Callback function that transfer data to display.
 */
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    ESP_LOGD(TAG, "flush_cb: X offset = (%d, %d), Y offset = (%d, %d)", offsetx1, offsetx2, offsety1, offsety2);
    int xlen = abs(offsetx1 - offsetx2) + 1;
    int ylen = abs(offsety1 - offsety2) + 1;
    // --- Convert buffer from color to monochrome bitmap
    int len_bits = (xlen * ylen);

    memset(converted_buffer_black, 0x00, len_bits / 8);
#if BLACK_RED_DISP
    memset(converted_buffer_red, 0x00, len_bits / 8);
#endif
    for (int i = 0; i < len_bits; i++) {
        // NOTE: Set bits of converted_buffer[] FROM LOW ADDR TO HIGH ADDR, FROM HSB TO LSB
        // NOTE: 1 means BLACK/RED, 0 means WHITE
#if !TRAVERSE_COLS
        // Horizontal traverse lvgl framebuffer (by row)
        converted_buffer_black[i / 8] |= (((lv_color_brightness(color_map[i])) < 251) << (7 - (i % 8)));
#if BLACK_RED_DISP
        converted_buffer_red[i / 8] |= ((((color_map[i].ch.red) > 3) && ((lv_color_brightness(color_map[i])) < 251)) << (7 - (i % 8)));
#endif
#else   // TRAVERSE_COLS==1. An experiment.
        // Vertical traverse lvgl framebuffer (by column)
        // NOTE: If your screen rotation requires setting the pixels vertically, you could use the code below.
        //       I don't think this functions as a way to change default orientation from portrait to landscape.
        int ii;
        ii = ((i * xlen) % len_bits) + i / ylen;
        converted_buffer_black[i / 8] |= (((lv_color_brightness(color_map[ii])) < 251) << (7 - (i % 8)));
#if BLACK_RED_DISP
        converted_buffer_red[i / 8] |= ((((color_map[ii].ch.red) > 3) && ((lv_color_brightness(color_map[ii])) < 251)) << (7 - (i % 8)));
#endif
#endif
    }

    // --- Draw bitmap - this performs DMA transfer
    ESP_ERROR_CHECK(epaper_panel_set_bitmap_color(panel_handle, SSD1681_EPAPER_BITMAP_BLACK));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, converted_buffer_black));
#if BLACK_RED_DISP
    ESP_ERROR_CHECK(epaper_panel_set_bitmap_color(panel_handle, SSD1681_EPAPER_BITMAP_RED));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, converted_buffer_red));
#endif
    // refresh_screen is required to make the new screen image visible. It is accompanied by visible flashing of eInk display.
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
}

static void lvgl_wait_cb(struct _lv_disp_drv_t *disp_drv)
{
    xSemaphoreTake(panel_refreshing_sem, portMAX_DELAY);
}

// normally this would be done in lvgl_port_update_callback, but non-square eInk displays have problems with that.
static void set_swap_mirror(lv_disp_drv_t *drv, uint32_t rotate)
{
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    ESP_LOGD(TAG, "set_swap_mirror: rotated = %lu", rotate);
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
#if SQUARE_DISP // square display can accomplish display rotation with combination of mirror and swap_xy
    switch (rotate) {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    }
#else   // not SQUARE
    // Non-square display cannot use swap_xy, must be handled by LVGL sw_rotate,
    // so be sure to set all to false.
    switch (rotate) {
    case LV_DISP_ROT_NONE:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    case LV_DISP_ROT_180:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_90:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    case LV_DISP_ROT_270:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    }
#endif
}

/**
 * Tell LVGL to rotate display and touch.
 * Called when driver parameters are updated.
 * NOTE: this function causes problems with non-square displays, principally because
 *       if drv->rotated is 90 or 270 then LVGL does things that are incompatible
 *       with the intent of changing orientation from portrait to landscape.
 *
 */
static void lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    ESP_LOGD(TAG, "lvgl_port_update_callback");
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
#if SQUARE_DISP // square display can accomplish display rotation with combination of mirror and swap_xy
    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    }
#else   // not SQUARE
    // Non-square display cannot use swap_xy, must be handled by LVGL sw_rotate,
    // so be sure to set all to false.
    ESP_LOGD(TAG, "port_update: rotated = %d", drv->rotated);
    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    case LV_DISP_ROT_180:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_90:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    case LV_DISP_ROT_270:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    }
#endif
}

/**
 * This function carries over from another example, don't know if it's needed.
 */
static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    panel_refreshing_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(panel_refreshing_sem);

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISPLAY_X * DISPLAY_Y / 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_EPD_DC,
        .cs_gpio_num = PIN_NUM_EPD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
    };
    // --- Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) LCD_HOST, &io_config, &io_handle));
    esp_lcd_panel_handle_t panel_handle = NULL;

    // --- Create esp_lcd panel
    esp_lcd_ssd1681_config_t epaper_ssd1681_config = {
        .busy_gpio_num = PIN_NUM_EPD_BUSY,
        // NOTE: Enable this to reduce one buffer copy if you do not use swap-xy, mirror y or invert color
        // since those operations are not supported by ssd1681 and are implemented by software
        // Better use DMA-capable memory region, to avoid additional data copy
        .non_copy_mode = false,  // TODO: was true
        // panel dimensions - not used by LVGL
        .display_x = DISPLAY_X,
        .display_y = DISPLAY_Y,
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_EPD_RST,
        .flags.reset_active_high = false,
        .vendor_config = &epaper_ssd1681_config
    };
    gpio_install_isr_service(0);
    // panel_handle is returned. io_handle is a member.
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1681(io_handle, &panel_config, &panel_handle));

    // --- Reset the display
    ESP_LOGI(TAG, "Resetting e-Paper display...");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(100 / portTICK_PERIOD_MS);   // for ESP32 this equates to a delay of 100 ms
    // --- Initialize panel
    ESP_LOGI(TAG, "Initializing e-Paper display...");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    // --- Register the e-Paper refresh done callback
    epaper_panel_callbacks_t cbs = {
        .on_epaper_refresh_done = epaper_flush_ready_callback
    };
    epaper_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // --- Turn on display
    ESP_LOGI(TAG, "Turning e-Paper display on...");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // --- Configurate the screen
    // NOTE: the configurations below are all FALSE by default
    //ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    //ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    esp_lcd_panel_invert_color(panel_handle, false);
    // NOTE: Calling esp_lcd_panel_disp_on_off(panel_handle, true) will reset the LUT to the panel built-in one,
    // custom LUT will not take effect any more after calling esp_lcd_panel_disp_on_off(panel_handle, true)
    // TODO: why call this a second time?
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    // --------------------------- //
    // ----- Initialize LVGL ----- //
    // --------------------------- //
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    // TODO: for B&W epaper, shouldn't the buffer be divide by 8?
    lv_color_t *buf1 = heap_caps_malloc(DISPLAY_X * DISPLAY_Y * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
#if 0
    // optional second display buffer for faster updates - not useful for eInk
    lv_color_t *buf2 = heap_caps_malloc(DISPLAY_X * DISPLAY_Y * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
#endif
    // alloc bitmap buffer to draw. Add 8 for monochrome palette?
    converted_buffer_black = heap_caps_malloc(DISPLAY_X * DISPLAY_Y / 8 + 8, MALLOC_CAP_DMA);
#if BLACK_RED_DISP
    converted_buffer_red = heap_caps_malloc(DISPLAY_X * DISPLAY_Y / 8 + 8, MALLOC_CAP_DMA);
#endif
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, NULL, DISPLAY_X * DISPLAY_Y);
    // initialize LVGL display driver
    lv_disp_drv_init(&disp_drv);
    //lv_display_set_color_format(disp_drv, LV_COLOR_FORMAT_I1);    // recommended but did not compile
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.wait_cb = lvgl_wait_cb;
    disp_drv.drv_update_cb = lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    // NOTE: The ssd1681 e-paper is monochrome and 1 byte represents 8 pixels
    // so full_refresh is MANDATORY because we cannot set position to bitmap at pixel level
    // NOTE (GDN): eInk controllers don't seem to have hardware rotate,
    // so we have to provide our own rotate function.
    // Do not use sw_totate=true for eInk displays because it results in multiple refreshes of screen
    // and the rotated image will always be clipped.
    disp_drv.rotated = LV_DISP_ROT_NONE;
    disp_drv.full_refresh = true;
    disp_drv.sw_rotate = false;
    disp_drv.hor_res = DISPLAY_X;
    disp_drv.ver_res = DISPLAY_Y;
    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
#if 0   // lvgl_port_update_callback causes trouble if you want to rotate by 90 or 270
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);    // must do this in order to activate lvgl_port_update_callback
#endif

    // init lvgl tick - but is it needed?
    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    // ----- The main loop. eInk demo is not interactive, so the loop exits. ----- //
    uint32_t lv_time;
    for (int i = 0; i < sizeof(ROTATE_TEST) / sizeof(ROTATE_TEST[0]); i++) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_time = lv_timer_handler();
        ESP_LOGD(TAG, "lv_time returned = %lu", lv_time);
        set_swap_mirror(&disp_drv, ROTATE_TEST[i]);
        lvgl_canvas_ui(DISPLAY_X, DISPLAY_Y, ROTATE_TEST[i]);
        int delay_ms = 0;
        while (delay_ms < TEST_DELAY) {
            // waste time before changing display
            lv_time = lv_timer_handler();
            // 10 is the smallest delay with FreeRTOS and ESP-IDF when using vTaskDelay
            vTaskDelay(pdMS_TO_TICKS(10));
            delay_ms += 10;
        }
    }

    // Do not leave eInk panels powered on when you're done.
    esp_lcd_panel_disp_on_off(panel_handle, false);
}
