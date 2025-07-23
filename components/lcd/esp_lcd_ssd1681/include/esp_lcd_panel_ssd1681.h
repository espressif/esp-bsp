/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdbool.h>
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"

// Select the Waveshare or GooDisplay epaper display that you have.
// The display must use a SSD1680 or 1681 controller (see Waveshare documentation to be sure).
#define WAVE_27 1   // 2.7 inch V2
#define WAVE_154 0  // 1.54 inch square (this code originally only supported this model)

/* NOTE: (GDN) in my limited experience the rectangular displays assume portrait orientation:
 * the X coordinate is the small dimension, Y is long. When transferring bitmaps to the display,
 * the controller increments X (index within a row) and increments Y (row selector) when X has reached the end of a row.
 * There are command options to change the direction of X and/or Y, from increment to decrement. But the controller always
 * writes a row and then incrments/decrements the Y index.
 * If you orient the display in landscape, then you may want to increnent Y from 0 to HEIGHT and then increment X to select
 * the next column. See SSD1681_CMD_DATA_ENTRY_MODE in esp_lcd_ssd1681_commands.h for more info.
 */
#define SSD1681_LUT_SIZE                   159
#if defined(WAVE_154) && WAVE_154==1
#define SSD1681_EPD_1IN54_V2_WIDTH         200
#define SSD1681_EPD_1IN54_V2_HEIGHT        200
#define DISPLAY_W 200
#define DISPLAY_V 200
#define SQUARE_PANEL    1
#elif defined(WAVE_27) && WAVE_27==1
// Waveshare 2.7 inch
#define WAVE_27_V2_WIDTH         176
#define WAVE_27_V2_HEIGHT        264
#define DISPLAY_W 176
#define DISPLAY_V 264
#endif
#define DISPLAY_H DISPLAY_V		// Vertical or Height, use either in the code

#ifdef __cplusplus
extern "C" {
#endif

/**
* @brief Prototype of ssd1681 driver event callback
*
* @param[in] handle    esp_lcd_panel_handle_t esp_lcd driver handle
* @param[in] edata     reserved
* @param[in] user_data User registered context, registered in `epaper_panel_register_event_callbacks()`
*
* @return Whether a high priority task is woken up by this function
*/
typedef bool (*esp_lcd_epaper_panel_cb_t)(const esp_lcd_panel_handle_t handle, const void *edata, void *user_data);

/**
 * @brief Type of ssd1681 e-paper callbacks
 */
typedef struct {
    esp_lcd_epaper_panel_cb_t on_epaper_refresh_done;  /*!< Callback invoked when e-paper refresh finishes */
} epaper_panel_callbacks_t;

/**
 * @brief Type of additional configuration needed by ssd1681 e-paper panel
 *        Please set the object of this struct to esp_lcd_panel_dev_config_t->vendor_config
 */
typedef struct {
    int busy_gpio_num;         /*!< GPIO num of the BUSY pin */
    bool non_copy_mode;        /*!< If the bitmap would be copied or not.
                                *   Image rotation and mirror is limited when enabling. */
} esp_lcd_ssd1681_config_t;

/**
 * @brief Enum of colors available of ssd1681 e-paper
 *        Some of the ssd1681 e-paper panels support not only black pixels but also red pixels.
 *        Use this enum to select the color of the bitmap you want to display.
 *        Need be set using `epaper_panel_set_bitmap_color()` before calling `epaper_panel_draw_bitmap()`.
 * @note Default to `SSD1681_EPAPER_BITMAP_BLACK` if not set.
 */
typedef enum {
    SSD1681_EPAPER_BITMAP_BLACK, /*!< Draw the bitmap in black */
    SSD1681_EPAPER_BITMAP_RED    /*!< Draw the bitmap in red */
} esp_lcd_ssd1681_bitmap_color_t;

/**
 * @brief Create LCD panel for model ssd1681 e-Paper
 * @attention
 *        Need to call `gpio_install_isr_service()` before calling this function.
 * @param[in] io LCD panel IO handle
 * @param[in] panel_dev_config general panel device configuration
 * @param[out] ret_panel Returned LCD panel handle
 * @return
 *          - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_ERR_NO_MEM        if out of memory
 *          - ESP_OK                on success
 */
esp_err_t esp_lcd_new_panel_ssd1681(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config,
                                    esp_lcd_panel_handle_t *ret_panel);

/**
 * @brief Refresh the e-Paper
 *
 * @note This function is called automatically in `draw_bitmap()` function.
 *       This function will return right after the refresh commands finish transmitting.
 * @attention
 *       If you want to call this function, you have to wait manually until the BUSY pin goes LOW
 *       before calling other functions that interacts with the e-paper.
 *
 * @param[in] panel LCD panel handle
 * @return
 *          - ESP_ERR_INVALID_ARG   if parameter is invalid
 *          - ESP_OK                on success
 */
esp_err_t epaper_panel_refresh_screen(esp_lcd_panel_t *panel);

/**
 * @brief Set the color of the next bitmap
 *
 * @note The SSD1681 has two separate vrams. One for black and the other for red(or maybe some other color).
 *       Call this function to set which vram the next bitmap would write into.
 *       If you set `epaper_panel_invert_color()` to false, then 0 means WHITE.
 *
 * @param[in] panel LCD panel handle
 * @param[in] color a enum value, SSD1681_EPAPER_BITMAP_BLACK or SSD1681_EPAPER_BITMAP_RED
 * @return ESP_OK                on success
 */
esp_err_t epaper_panel_set_bitmap_color(esp_lcd_panel_t *panel, esp_lcd_ssd1681_bitmap_color_t color);

/**
 * @brief Set the callback function
 *
 * @note The callback function `on_epaper_refresh_done` will only be called right after a `draw_bitmap()` caused refresh finishes.
 * @note You could set the `epaper_panel_callbacks_t->esp_lcd_epaper_panel_cb_t` to NULL to unregister the callback.
 * @attention
 *       The callback function `on_epaper_refresh_done` will be called in isr context, so please keep it as neat as possible.
 *
 * @param[in] panel LCD panel handle
 * @param[in] cbs callback functions
 * @param[in] user_ctx arg to be passed to your callback function
 * @return ESP_OK                on success
 *         ESP_ERR_INVALID_ARG   if parameter is invalid
 */
esp_err_t epaper_panel_register_event_callbacks(esp_lcd_panel_t *panel, epaper_panel_callbacks_t *cbs, void *user_ctx);

/**
 * @brief Set a custom waveform lut
 *
 * @note Set a custom waveform lut for your e-paper panel.
 * @note You do not have to call this function because e-paper panels usually have waveform lut built-in.
 * @note You could call `disp_on_off(panel_handle, true)` to reset the waveform LUT to the built-in one
 *
 * @param[in] panel LCD panel handle
 * @param[in] lut your custom lut array
 * @param[in] size size of your lut array, make sure it is SSD1681_LUT_SIZE bytes
 * @return  ESP_OK                on success
 *          ESP_ERR_INVALID_ARG   if parameter is invalid
 */
esp_err_t epaper_panel_set_custom_lut(esp_lcd_panel_t *panel, uint8_t *lut, size_t size);


#ifdef __cplusplus
}
#endif
