/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#if CONFIG_LCD_ENABLE_DEBUG_LOG
// The local log level must be defined before including esp_log.h
// Set the maximum log level for this source file
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#endif
#include "esp_log.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
#include "esp_attr.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_ssd1681.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_ssd1681_commands.h"

#define SSD1681_LUT_SIZE                   159
#define SSD1681_EPD_1IN54_V2_WIDTH         200
#define SSD1681_EPD_1IN54_V2_HEIGHT        200


static const char *TAG = "lcd_panel.epaper";

typedef struct {
    esp_lcd_epaper_panel_cb_t callback_ptr;
    void *args;
} epaper_panel_callback_t;

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    // --- Normal configurations
    // Configurations from panel_dev_config
    int reset_gpio_num;
    bool reset_level;
    // Configurations from epaper_ssd1681_conf
    int busy_gpio_num;
    bool full_refresh;
    // Configurations from interface functions
    int gap_x;
    int gap_y;
    // Configurations from e-Paper specific public functions
    epaper_panel_callback_t epaper_refresh_done_isr_callback;
    esp_lcd_ssd1681_bitmap_color_t bitmap_color;
    // --- Associated configurations
    // SHOULD NOT modify directly
    // in order to avoid going into undefined state
    bool _non_copy_mode;
    bool _mirror_y;
    bool _swap_xy;
    // --- Other private fields
    bool _mirror_x;
    uint8_t *_framebuffer;
    bool _invert_color;
} epaper_panel_t;

// --- Utility functions
static inline uint8_t byte_reverse(uint8_t data);
static esp_err_t process_bitmap(esp_lcd_panel_t *panel, int len_x, int len_y, int buffer_size, const void *color_data);
static esp_err_t panel_epaper_wait_busy(esp_lcd_panel_t *panel);
// --- Callback functions & ISRs
static void epaper_driver_gpio_isr_handler(void *arg);
// --- IO wrapper functions, simply send command/param/buffer
static esp_err_t epaper_set_lut(esp_lcd_panel_io_handle_t io, const uint8_t *lut);
static esp_err_t epaper_set_cursor(esp_lcd_panel_io_handle_t io, uint32_t cur_x, uint32_t cur_y);
static esp_err_t epaper_set_area(esp_lcd_panel_io_handle_t io, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y);
static esp_err_t panel_epaper_set_vram(esp_lcd_panel_io_handle_t io, uint8_t *bw_bitmap, uint8_t *red_bitmap, size_t size);
// --- SSD1681 specific functions, exported to user in public header file
// extern esp_err_t esp_lcd_new_panel_ssd1681(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
//                                            esp_lcd_panel_handle_t *ret_panel);
// extern esp_err_t epaper_panel_register_event_callbacks(esp_lcd_panel_t *panel, epaper_panel_callbacks_t* cbs, void* user_ctx);
// extern esp_err_t epaper_panel_refresh_screen(esp_lcd_panel_t *panel);
// extern esp_err_t epaper_panel_set_bitmap_color(esp_lcd_panel_t* panel, esp_lcd_ssd1681_bitmap_color_t color);
// extern esp_err_t epaper_panel_set_custom_lut(esp_lcd_panel_t *panel, uint8_t *lut, size_t size);
// --- Used to implement esp_lcd_panel_interface
static esp_err_t epaper_panel_del(esp_lcd_panel_t *panel);
static esp_err_t epaper_panel_reset(esp_lcd_panel_t *panel);
static esp_err_t epaper_panel_init(esp_lcd_panel_t *panel);
static esp_err_t epaper_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t epaper_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t epaper_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t epaper_panel_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t epaper_panel_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t epaper_panel_disp_on_off(esp_lcd_panel_t *panel, bool on_off);


static void epaper_driver_gpio_isr_handler(void *arg)
{
    epaper_panel_t *epaper_panel = arg;
    // --- Disable ISR handling
    gpio_intr_disable(epaper_panel->busy_gpio_num);

    // --- Call user callback func
    if (epaper_panel->epaper_refresh_done_isr_callback.callback_ptr) {
        (epaper_panel->epaper_refresh_done_isr_callback.callback_ptr)(&(epaper_panel->base), NULL, epaper_panel->epaper_refresh_done_isr_callback.args);
    }
}

esp_err_t epaper_panel_register_event_callbacks(esp_lcd_panel_t *panel, epaper_panel_callbacks_t *cbs, void *user_ctx)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handler is NULL");
    ESP_RETURN_ON_FALSE(cbs, ESP_ERR_INVALID_ARG, TAG, "cbs is NULL");
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    (epaper_panel->epaper_refresh_done_isr_callback).callback_ptr = cbs->on_epaper_refresh_done;
    (epaper_panel->epaper_refresh_done_isr_callback).args = user_ctx;
    return ESP_OK;
}

esp_err_t epaper_panel_set_custom_lut(esp_lcd_panel_t *panel, uint8_t *lut, size_t size)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handler is NULL");
    ESP_RETURN_ON_FALSE(lut, ESP_ERR_INVALID_ARG, TAG, "lut is NULL");
    ESP_RETURN_ON_FALSE(size == SSD1681_LUT_SIZE, ESP_ERR_INVALID_ARG, TAG, "Invalid lut size");
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_set_lut(epaper_panel->io, lut);
    return ESP_OK;
}

static esp_err_t epaper_set_lut(esp_lcd_panel_io_handle_t io, const uint8_t *lut)
{
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_LUT_REG, lut, 153), TAG, "SSD1681_CMD_OUTPUT_CTRL err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_DISP_UPDATE_CTRL, (uint8_t[]) {
        lut[153]
    }, 1), TAG, "SSD1681_CMD_SET_END_OPTION err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_GATE_DRIVING_VOLTAGE, (uint8_t[]) {
        lut[154]
    }, 1), TAG, "SSD1681_CMD_SET_END_OPTION err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_SRC_DRIVING_VOLTAGE, (uint8_t[]) {
        lut[155], lut[156], lut[157]
    }, 3), TAG, "SSD1681_CMD_SET_SRC_DRIVING_VOLTAGE err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_VCOM_REG, (uint8_t[]) {
        lut[158]
    }, 1), TAG, "SSD1681_CMD_SET_VCOM_REG err");

    return ESP_OK;
}

static esp_err_t epaper_set_cursor(esp_lcd_panel_io_handle_t io, uint32_t cur_x, uint32_t cur_y)
{
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_INIT_X_ADDR_COUNTER, (uint8_t[]) {
        (uint8_t)((cur_x >> 3) & 0xff)
    }, 1), TAG, "SSD1681_CMD_SET_INIT_X_ADDR_COUNTER err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_INIT_Y_ADDR_COUNTER, (uint8_t[]) {
        (uint8_t)(cur_y & 0xff),        // cur_y[7:0]
        (uint8_t)((cur_y >> 8) & 0xff)  // cur_y[8]
    }, 2), TAG, "SSD1681_CMD_SET_INIT_Y_ADDR_COUNTER err");

    return ESP_OK;
}

static esp_err_t epaper_set_area(esp_lcd_panel_io_handle_t io, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y)
{
    // --- Set RAMX Start/End Position
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_RAMX_START_END_POS, (uint8_t[]) {
        (start_x >> 3) & 0xff,  // start_x
        (end_x >> 3) & 0xff     // end_x
    }, 2), TAG, "SSD1681_CMD_SET_RAMX_START_END_POS err");

    // --- Set RAMY Start/End Position
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SET_RAMY_START_END_POS, (uint8_t[]) {
        (start_y) & 0xff,          // start_y[7:0]
        (start_y >> 8) & 0xff,     // start_y[8]
        end_y & 0xff,              // end_y[7:0]
        (end_y >> 8) & 0xff        // end_y[8]
    }, 4), TAG, "SSD1681_CMD_SET_RAMX_START_END_POS err");

    return ESP_OK;
}

static esp_err_t panel_epaper_wait_busy(esp_lcd_panel_t *panel)
{
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    while (gpio_get_level(epaper_panel->busy_gpio_num)) {
        vTaskDelay(pdMS_TO_TICKS(15));
    }
    return ESP_OK;
}

esp_err_t panel_epaper_set_vram(esp_lcd_panel_io_handle_t io, uint8_t *bw_bitmap, uint8_t *red_bitmap, size_t size)
{
    // Note: the screen region to be used to draw bitmap had been defined
    // The region of BLACK VRAM and RED VRAM are set by the same series of command, the two bitmaps will be drawn at
    // the same region, so the two bitmaps can share a same size.
    // --- Validate and Transfer data
    if (bw_bitmap && (size > 0)) {
        // tx WHITE/BLACK bitmap
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, SSD1681_CMD_WRITE_BLACK_VRAM, bw_bitmap, size), TAG,
                            "data bw_bitmap err");
    }
    if (red_bitmap && (size > 0)) {
        // tx RED bitmap
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_color(io, SSD1681_CMD_WRITE_RED_VRAM, red_bitmap, size), TAG,
                            "data red_bitmap err");
    }
    return ESP_OK;
}

esp_err_t epaper_panel_refresh_screen(esp_lcd_panel_t *panel)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handler is NULL");
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // --- Set color invert
    uint8_t duc_flag = 0x00;
    if (!(epaper_panel->_invert_color)) {
        duc_flag |= SSD1681_PARAM_COLOR_BW_INVERSE_BIT;
        duc_flag &= (~SSD1681_PARAM_COLOR_RW_INVERSE_BIT);
    } else {
        duc_flag &= (~SSD1681_PARAM_COLOR_BW_INVERSE_BIT);
        duc_flag |= SSD1681_PARAM_COLOR_RW_INVERSE_BIT;
    }
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_DISP_UPDATE_CTRL, (uint8_t[]) {
        duc_flag  // Color invert flag
    }, 1), TAG, "SSD1681_CMD_DISP_UPDATE_CTRL err");
    // --- Enable refresh done handler isr
    gpio_intr_enable(epaper_panel->busy_gpio_num);
    // --- Send refresh command
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_SET_DISP_UPDATE_CTRL, (uint8_t[]) {
        SSD1681_PARAM_DISP_WITH_MODE_2
    }, 1), TAG, "SSD1681_CMD_SET_DISP_UPDATE_CTRL err");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_ACTIVE_DISP_UPDATE_SEQ, NULL, 0), TAG,
                        "SSD1681_CMD_ACTIVE_DISP_UPDATE_SEQ err");

    return ESP_OK;
}

esp_err_t
esp_lcd_new_panel_ssd1681(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *const panel_dev_config,
                          esp_lcd_panel_handle_t *const ret_panel)
{
#if CONFIG_LCD_ENABLE_DEBUG_LOG
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
#endif
    ESP_RETURN_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, TAG, "1 or more args is NULL");
    esp_lcd_ssd1681_config_t *epaper_ssd1681_conf = panel_dev_config->vendor_config;
    esp_err_t ret = ESP_OK;
    // --- Allocate epaper_panel memory on HEAP
    epaper_panel_t *epaper_panel = NULL;
    epaper_panel = calloc(1, sizeof(epaper_panel_t));
    ESP_GOTO_ON_FALSE(epaper_panel, ESP_ERR_NO_MEM, err, TAG, "no mem for epaper panel");

    // --- Construct panel & implement interface
    // defaults
    epaper_panel->_invert_color = false;
    epaper_panel->_swap_xy = false;
    epaper_panel->_mirror_x = false;
    epaper_panel->_mirror_y = false;
    epaper_panel->_framebuffer = NULL;
    epaper_panel->gap_x = 0;
    epaper_panel->gap_y = 0;
    epaper_panel->bitmap_color = SSD1681_EPAPER_BITMAP_BLACK;
    epaper_panel->full_refresh = true;
    // configurations
    epaper_panel->io = io;
    epaper_panel->reset_gpio_num = panel_dev_config->reset_gpio_num;
    epaper_panel->busy_gpio_num = epaper_ssd1681_conf->busy_gpio_num;
    epaper_panel->reset_level = panel_dev_config->flags.reset_active_high;
    epaper_panel->_non_copy_mode = epaper_ssd1681_conf->non_copy_mode;
    // functions
    epaper_panel->base.del = epaper_panel_del;
    epaper_panel->base.reset = epaper_panel_reset;
    epaper_panel->base.init = epaper_panel_init;
    epaper_panel->base.draw_bitmap = epaper_panel_draw_bitmap;
    epaper_panel->base.invert_color = epaper_panel_invert_color;
    epaper_panel->base.set_gap = epaper_panel_set_gap;
    epaper_panel->base.mirror = epaper_panel_mirror;
    epaper_panel->base.swap_xy = epaper_panel_swap_xy;
    epaper_panel->base.disp_on_off = epaper_panel_disp_on_off;
    *ret_panel = &(epaper_panel->base);
    // --- Init framebuffer
    if (!(epaper_panel->_non_copy_mode)) {
        epaper_panel->_framebuffer = heap_caps_malloc(SSD1681_EPD_1IN54_V2_WIDTH * SSD1681_EPD_1IN54_V2_HEIGHT / 8,
                                     MALLOC_CAP_DMA);
        ESP_RETURN_ON_FALSE(epaper_panel->_framebuffer, ESP_ERR_NO_MEM, TAG, "epaper_panel_draw_bitmap allocating buffer memory err");
    }
    // --- Init GPIO
    // init RST GPIO
    if (epaper_panel->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line err");
    }
    // init BUSY GPIO
    if (epaper_panel->busy_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_INPUT,
            .pull_down_en = 0x01,
            .pin_bit_mask = 1ULL << epaper_panel->busy_gpio_num,
        };
        io_conf.intr_type = GPIO_INTR_NEGEDGE;
        ESP_LOGI(TAG, "Add handler for GPIO %d", epaper_panel->busy_gpio_num);
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for BUSY line err");
        ESP_GOTO_ON_ERROR(gpio_isr_handler_add(epaper_panel->busy_gpio_num, epaper_driver_gpio_isr_handler, epaper_panel),
                          err, TAG, "configure GPIO for BUSY line err");
        // Enable GPIO intr only before refreshing, to avoid other commands caused intr trigger
        gpio_intr_disable(epaper_panel->busy_gpio_num);
    }
    ESP_LOGD(TAG, "new epaper panel @%p", epaper_panel);
    return ret;
err:
    if (epaper_panel) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        if (epaper_ssd1681_conf->busy_gpio_num >= 0) {
            gpio_reset_pin(epaper_ssd1681_conf->busy_gpio_num);
        }
        free(epaper_panel);
    }
    return ret;
}

static esp_err_t epaper_panel_del(esp_lcd_panel_t *panel)
{
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // --- Reset used GPIO pins
    if ((epaper_panel->reset_gpio_num) >= 0) {
        gpio_reset_pin(epaper_panel->reset_gpio_num);
    }
    gpio_reset_pin(epaper_panel->busy_gpio_num);
    // --- Free allocated RAM
    if ((epaper_panel->_framebuffer) && (!(epaper_panel->_non_copy_mode))) {
        // Should not free if buffer is not allocated by driver
        free(epaper_panel->_framebuffer);
    }
    ESP_LOGD(TAG, "del ssd1681 epaper panel @%p", epaper_panel);
    free(epaper_panel);
    return ESP_OK;
}

static esp_err_t epaper_panel_reset(esp_lcd_panel_t *panel)
{
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = epaper_panel->io;

    // perform hardware reset
    if (epaper_panel->reset_gpio_num >= 0) {
        ESP_RETURN_ON_ERROR(gpio_set_level(epaper_panel->reset_gpio_num, epaper_panel->reset_level), TAG,
                            "gpio_set_level error");
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(gpio_set_level(epaper_panel->reset_gpio_num, !epaper_panel->reset_level), TAG,
                            "gpio_set_level error");
        vTaskDelay(pdMS_TO_TICKS(10));
    } else {
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SWRST, NULL, 0), TAG,
                            "param SSD1681_CMD_SWRST err");
    }
    panel_epaper_wait_busy(panel);
    return ESP_OK;
}

esp_err_t epaper_panel_set_bitmap_color(esp_lcd_panel_t *panel, esp_lcd_ssd1681_bitmap_color_t color)
{
    ESP_RETURN_ON_FALSE(panel, ESP_ERR_INVALID_ARG, TAG, "panel handler is NULL");
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->bitmap_color = color;
    return ESP_OK;
}

static esp_err_t epaper_panel_init(esp_lcd_panel_t *panel)
{
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = epaper_panel->io;
    // --- SWRST
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_SWRST, NULL, 0), TAG,
                        "param SSD1681_CMD_SWRST err");
    panel_epaper_wait_busy(panel);
    // --- Driver Output Control
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_OUTPUT_CTRL,
                        SSD1681_PARAM_OUTPUT_CTRL, 3), TAG, "SSD1681_CMD_OUTPUT_CTRL err");

    // --- Border Waveform Control
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_SET_BORDER_WAVEFORM, (uint8_t[]) {
        SSD1681_PARAM_BORDER_WAVEFORM
    }, 1), TAG, "SSD1681_CMD_SET_BORDER_WAVEFORM err");

    // --- Temperature Sensor Control
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_SET_TEMP_SENSOR, (uint8_t[]) {
        SSD1681_PARAM_TEMP_SENSOR
    }, 1), TAG, "SSD1681_CMD_SET_TEMP_SENSOR err");
    // --- Load built-in waveform LUT
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_SET_DISP_UPDATE_CTRL, (uint8_t[]) {
        SSD1681_PARAM_DISP_UPDATE_MODE_1
    }, 1), TAG, "SSD1681_CMD_SET_DISP_UPDATE_CTRL err");
    // --- Display end option
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_SET_END_OPTION, (uint8_t[]) {
        SSD1681_PARAM_END_OPTION_KEEP
    }, 1), TAG, "SSD1681_CMD_SET_END_OPTION err");
    // --- Active Display Update Sequence
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_ACTIVE_DISP_UPDATE_SEQ, NULL, 0), TAG,
                        "param SSD1681_CMD_SET_DISP_UPDATE_CTRL err");
    panel_epaper_wait_busy(panel);

    return ESP_OK;
}

static esp_err_t
epaper_panel_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    if (gpio_get_level(epaper_panel->busy_gpio_num)) {
        return ESP_ERR_NOT_FINISHED;
    }
    x_start += epaper_panel->gap_x;
    x_end += epaper_panel->gap_x;
    y_start += epaper_panel->gap_y;
    y_end += epaper_panel->gap_y;
    // --- Assert & check configuration
    if (epaper_panel->_non_copy_mode) {
        ESP_RETURN_ON_FALSE(!(epaper_panel->_swap_xy), ESP_ERR_INVALID_ARG, TAG, "swap-xy is unavailable when enabling non-copy mode");
        ESP_RETURN_ON_FALSE(!(epaper_panel->_mirror_y), ESP_ERR_INVALID_ARG, TAG, "mirror_y is unavailable when enabling non-copy mode");
    }
    ESP_RETURN_ON_FALSE(color_data, ESP_ERR_INVALID_ARG, TAG, "bitmap is null");
    ESP_RETURN_ON_FALSE((x_start < x_end) && (y_start < y_end), ESP_ERR_INVALID_ARG, TAG, "start position must be smaller than end position");
    // --- Calculate coordinates & sizes
    int len_x = abs(x_start - x_end);
    int len_y = abs(y_start - y_end);
    x_end --; y_end --;
    int buffer_size = len_x * len_y / 8;
    // --- Data copy & preprocess
    // prepare buffer
    if (epaper_panel->_non_copy_mode) {
        // Use user-passed framebuffer
        epaper_panel->_framebuffer = (uint8_t *)color_data;
        if (!esp_ptr_dma_capable(epaper_panel->_framebuffer)) {
            ESP_LOGW(TAG, "Bitmap not DMA capable, use DMA capable memory to avoid additional data copy.");
        }
    } else {
        // Copy & convert image according to configuration
        process_bitmap(panel, len_x, len_y, buffer_size, color_data);
    }
    // --- Set cursor & data entry sequence
    if ((!(epaper_panel->_mirror_x)) && (!(epaper_panel->_mirror_y))) {
        // --- Cursor Settings
        ESP_RETURN_ON_ERROR(epaper_set_area(epaper_panel->io, x_start, y_start, x_end, y_end), TAG,
                            "epaper_set_area() error");
        ESP_RETURN_ON_ERROR(epaper_set_cursor(epaper_panel->io, x_start, y_start), TAG,
                            "epaper_set_cursor() error");
        // --- Data Entry Sequence Setting
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_DATA_ENTRY_MODE, (uint8_t[]) {
            SSD1681_PARAM_DATA_ENTRY_MODE_3
        }, 1), TAG, "SSD1681_CMD_DATA_ENTRY_MODE err");
    }
    if ((!(epaper_panel->_mirror_x)) && (epaper_panel->_mirror_y)) {
        // --- Cursor Settings
        ESP_RETURN_ON_ERROR(epaper_set_area(epaper_panel->io, x_start, y_start + len_y - 1, x_end, y_end + len_y - 1), TAG,
                            "epaper_set_area() error");
        ESP_RETURN_ON_ERROR(epaper_set_cursor(epaper_panel->io, x_start, y_start + len_y - 1), TAG,
                            "epaper_set_cursor() error");
        // --- Data Entry Sequence Setting
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_DATA_ENTRY_MODE, (uint8_t[]) {
            SSD1681_PARAM_DATA_ENTRY_MODE_1
        }, 1), TAG, "SSD1681_CMD_DATA_ENTRY_MODE err");
    }
    if (((epaper_panel->_mirror_x)) && (!(epaper_panel->_mirror_y))) {
        // --- Cursor Settings
        ESP_RETURN_ON_ERROR(epaper_set_area(epaper_panel->io, x_start, y_start + len_y - 1, x_end, y_end + len_y - 1), TAG,
                            "epaper_set_area() error");
        ESP_RETURN_ON_ERROR(epaper_set_cursor(epaper_panel->io, x_start, y_start + len_y - 1), TAG,
                            "epaper_set_cursor() error");
        // --- Data Entry Sequence Setting
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_DATA_ENTRY_MODE, (uint8_t[]) {
            SSD1681_PARAM_DATA_ENTRY_MODE_1
        }, 1), TAG, "SSD1681_CMD_DATA_ENTRY_MODE err");
    }
    if (((epaper_panel->_mirror_x)) && (epaper_panel->_mirror_y)) {
        // --- Cursor Settings
        ESP_RETURN_ON_ERROR(epaper_set_area(epaper_panel->io, x_start, y_start, x_end, y_end), TAG,
                            "epaper_set_area() error");
        ESP_RETURN_ON_ERROR(epaper_set_cursor(epaper_panel->io, x_start, y_start), TAG,
                            "epaper_set_cursor() error");
        // --- Data Entry Sequence Setting
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_DATA_ENTRY_MODE, (uint8_t[]) {
            SSD1681_PARAM_DATA_ENTRY_MODE_3
        }, 1), TAG, "SSD1681_CMD_DATA_ENTRY_MODE err");
    }
    // --- Send bitmap to e-Paper VRAM
    if (epaper_panel->bitmap_color == SSD1681_EPAPER_BITMAP_BLACK) {
        ESP_RETURN_ON_ERROR(panel_epaper_set_vram(epaper_panel->io, (uint8_t *) (epaper_panel->_framebuffer), NULL,
                            (len_x * len_y / 8)),
                            TAG, "panel_epaper_set_vram error");
    } else if (epaper_panel->bitmap_color == SSD1681_EPAPER_BITMAP_RED) {
        ESP_RETURN_ON_ERROR(panel_epaper_set_vram(epaper_panel->io, NULL, (uint8_t *) (epaper_panel->_framebuffer),
                            (len_x * len_y / 8)),
                            TAG, "panel_epaper_set_vram error");
    }
    // --- Refresh the display, show image in VRAM
    // tx_param will wait until DMA transaction finishes, so it is safe to call panel_epaper_refresh_screen at once.
    // The driver will not call the `epaper_panel_refresh_screen` automatically, please call it manually.
    return ESP_OK;
}

static esp_err_t epaper_panel_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->_invert_color = invert_color_data;
    return ESP_OK;
}

static esp_err_t epaper_panel_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    if (mirror_y) {
        if (epaper_panel->_non_copy_mode) {
            ESP_LOGE(TAG, "mirror_y is unavailable when enabling non-copy mode");
            return ESP_ERR_INVALID_ARG;
        }
    }
    epaper_panel->_mirror_x = mirror_x;
    epaper_panel->_mirror_y = mirror_y;

    return ESP_OK;
}

static esp_err_t epaper_panel_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    if (swap_axes) {
        if (epaper_panel->_non_copy_mode) {
            ESP_LOGE(TAG, "swap_xy is unavailable when enabling non-copy mode");
            return ESP_ERR_INVALID_ARG;
        }
    }
    epaper_panel->_swap_xy = swap_axes;
    return ESP_OK;
}

static esp_err_t epaper_panel_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    epaper_panel->gap_x = x_gap;
    epaper_panel->gap_y = y_gap;
    return ESP_OK;
}

static esp_err_t epaper_panel_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    esp_lcd_panel_io_handle_t io = epaper_panel->io;
    if (on_off) {
        // Turn on display
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_SET_DISP_UPDATE_CTRL, (uint8_t[]) {
            SSD1681_PARAM_DISP_UPDATE_MODE_1
        }, 1), TAG, "SSD1681_CMD_SET_DISP_UPDATE_CTRL err");
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(io, SSD1681_CMD_ACTIVE_DISP_UPDATE_SEQ, NULL, 0), TAG,
                            "SSD1681_CMD_ACTIVE_DISP_UPDATE_SEQ err");
        panel_epaper_wait_busy(panel);
    } else {
        // Sleep mode, BUSY pin will keep HIGH after entering sleep mode
        // Perform reset and re-run init to resume the display
        ESP_RETURN_ON_ERROR(esp_lcd_panel_io_tx_param(epaper_panel->io, SSD1681_CMD_SLEEP_CTRL, (uint8_t[]) {
            SSD1681_PARAM_SLEEP_MODE_1
        }, 1), TAG, "SSD1681_CMD_SLEEP_CTRL err");
        // BUSY pin will stay HIGH, so do not call panel_epaper_wait_busy() here
    }

    return ESP_OK;
}

static esp_err_t process_bitmap(esp_lcd_panel_t *panel, int len_x, int len_y, int buffer_size, const void *color_data)
{
    epaper_panel_t *epaper_panel = __containerof(panel, epaper_panel_t, base);
    // --- Convert image according to configuration
    if ((!(epaper_panel->_mirror_x)) && (!(epaper_panel->_mirror_y))) {
        if (!(epaper_panel->_non_copy_mode)) {
            if (epaper_panel->_swap_xy) {
                memset(epaper_panel->_framebuffer, 0, 200 * 200 / 8);
                for (int i = 0; i < buffer_size * 8; i++) {
                    uint8_t bitmap_byte = ((uint8_t *) (color_data))[i / 8];
                    uint8_t bitmap_pixel = (bitmap_byte & (0x01 << (7 - (i % 8)))) ? 0x01 : 0x00;
                    (epaper_panel->_framebuffer)[((i * len_y / 8) % buffer_size) + (i / 8 / len_x)] |= (bitmap_pixel << (7 - ((i / len_x) % 8)));
                }
            } else {
                for (int i = 0; i < buffer_size; i++) {
                    (epaper_panel->_framebuffer)[i] = ((uint8_t *) (color_data))[i];
                }
            }
        }
    }
    if ((!(epaper_panel->_mirror_x)) && (epaper_panel->_mirror_y)) {
        if (epaper_panel->_swap_xy) {
            memset((epaper_panel->_framebuffer), 0, 200 * 200 / 8);
            for (int i = 0; i < buffer_size * 8; i++) {
                uint8_t bitmap_byte = ((uint8_t *) (color_data))[i / 8];
                uint8_t bitmap_pixel = (bitmap_byte & (0x01 << (7 - (i % 8)))) ? 0x01 : 0x00;
                (epaper_panel->_framebuffer)[buffer_size - (((i * len_y / 8) % buffer_size) + (i / 8 / len_x)) - 1] |= (bitmap_pixel << (((i / len_x) % 8)));
            }
        } else {
            for (int i = 0; i < buffer_size; i++) {
                (epaper_panel->_framebuffer)[buffer_size - i - 1] = byte_reverse(((uint8_t *)(color_data))[i]);
            }
        }
    }
    if (((epaper_panel->_mirror_x)) && (!(epaper_panel->_mirror_y))) {
        if (!(epaper_panel->_non_copy_mode)) {
            if (epaper_panel->_swap_xy) {
                memset((epaper_panel->_framebuffer), 0, 200 * 200 / 8);
                for (int i = 0; i < buffer_size * 8; i++) {
                    uint8_t bitmap_byte = ((uint8_t *) (color_data))[i / 8];
                    uint8_t bitmap_pixel = (bitmap_byte & (0x01 << (7 - (i % 8)))) ? 0x01 : 0x00;
                    (epaper_panel->_framebuffer)[((i * len_y / 8) % buffer_size) + (i / 8 / len_x)] |= (bitmap_pixel << (7 - ((i / len_x) % 8)));
                }
            } else {
                for (int i = 0; i < buffer_size; i++) {
                    (epaper_panel->_framebuffer)[i] = ((uint8_t *) (color_data))[i];
                }
            }
        }
    }
    if (((epaper_panel->_mirror_x)) && (epaper_panel->_mirror_y)) {
        if (epaper_panel->_swap_xy) {
            memset((epaper_panel->_framebuffer), 0, 200 * 200 / 8);
            for (int i = 0; i < buffer_size * 8; i++) {
                uint8_t bitmap_byte = ((uint8_t *) (color_data))[i / 8];
                uint8_t bitmap_pixel = (bitmap_byte & (0x01 << (7 - (i % 8)))) ? 0x01 : 0x00;
                (epaper_panel->_framebuffer)[buffer_size - (((i * len_y / 8) % buffer_size) + (i / 8 / len_x)) - 1] |= (bitmap_pixel << (((i / len_x) % 8)));
            }
        } else {
            for (int i = 0; i < buffer_size; i++) {
                (epaper_panel->_framebuffer)[buffer_size - i - 1] = byte_reverse(((uint8_t *)(color_data))[i]);
            }
        }
    }

    return ESP_OK;
}

static inline uint8_t byte_reverse(uint8_t data)
{
    static uint8_t _4bit_reverse_lut[] =  {
        0x00, 0x08, 0x04, 0x0C, 0x02, 0x0A, 0x06, 0x0E,
        0x01, 0x09, 0x05, 0x0D, 0x03, 0x0B, 0x07, 0x0F
    };
    uint8_t result = 0x00;
    // Reverse low 4 bits
    result |= (uint8_t)((_4bit_reverse_lut[data & 0x0f]) << 4);
    // Reverse high 4 bits
    result |= _4bit_reverse_lut[data >> 4];
    return result;
}
