/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

// This demo UI is adapted from LVGL official example: https://docs.lvgl.io/master/widgets/extra/meter.html#simple-meter
// This demo writes text onto canvas that can later be rotated to appear on a portrait or landscape oriened display.

#define LV_FONT_MONTSERRAT_28 1 // 28 pt font. Must precede "lvgl.h".

#include "lvgl.h"
#include "esp_heap_caps.h"

#define CANVAS_COLOR_FORMAT LV_IMG_CF_ALPHA_1BIT    // desginates 1 bit per pixel display.

extern void rotate_buffer(uint32_t rotate, int xlen, int ylen, uint8_t *in_buf, uint8_t *out_buf);

static lv_style_t style_28;
static uint8_t *canvas_buffer = NULL;
/**
 *
 * @brief Use LVGL canvas to generate display and demonstrate rotation
 * @param width pixels of non-rotated display
 * @param height pixels of non-rotated display
 * @param rotate LVGL rotation value, [0, 1, 2, 3] for LV_DISP_ROT_<NONE|90|180|270]>
 * @note A canvas is created with no parent. Text is written to the canvas.
 *       The canvas is then assigned the active screen as parent.
 *       This apparently causes lvgl_flush_callback to be invoked.
 */
void lvgl_canvas_ui(int width, int height, uint32_t rotate)

{
    int disp_x, disp_y;
    if (rotate == LV_DISP_ROT_90 || rotate == LV_DISP_ROT_270) {
        disp_x = height;
        disp_y = width;
    } else {
        disp_y = height;
        disp_x = width;
    }
    if (!canvas_buffer) {
        // Needs 8 extra bytes for monochrome displays?
        canvas_buffer = heap_caps_malloc(disp_x * disp_y / 8 + 8, MALLOC_CAP_DMA);
    }
    memset(canvas_buffer, 0x00, disp_x * disp_y / 8 + 8);

    // Create a screen (a necessary, non-visible step in LVGL)
    lv_obj_t *scr = lv_obj_create(NULL);    // canvas must not be attached to display until later
    // Create the canvas on the screen
    lv_obj_t *canvas = lv_canvas_create(scr);
    // Assign canvas_buffer to the canvas. The canvas is configured to be either portrait or landscape orientation.
    lv_canvas_set_buffer(canvas, canvas_buffer, disp_x, disp_y, CANVAS_COLOR_FORMAT);
    lv_canvas_fill_bg(canvas, lv_color_hex(0x000000), LV_OPA_0);    // 0x000000 is a white background on eInk displays

#if 0
    // NOTE: palette is only used with indexed color formats, not monochrome.
    // Might be used for multi-color eInk displays?
    lv_canvas_set_palette(canvas, 0, LV_COLOR_WHITE);
    lv_canvas_set_palette(canvas, 1, LV_COLOR_BLACK);
#endif

    lv_style_init(&style_28);
    lv_style_set_text_font(&style_28, &lv_font_montserrat_28);
    lv_draw_label_dsc_t label_dsc;  // label properties descriptor
    lv_draw_label_dsc_init(&label_dsc);
    label_dsc.color = lv_color_make(0xFF, 0xFF, 0xFF); // Black color (or use lv_color_hex(0xFFFFFF))
    label_dsc.font = &lv_font_montserrat_28; // Use a built-in font
    label_dsc.align = LV_TEXT_ALIGN_LEFT;

    int xoff = 10, yoff = 10;   // offset for start of text string
    // canvas_draw_text automatically wraps text when right margin is reached
    lv_canvas_draw_text(canvas, xoff, yoff, disp_x - xoff, &label_dsc, "This text goes to end of line and then wraps. Isn't that cool?");

    // Now we need a second canvas if we want to rotate
    lv_obj_t *rot_canvas = lv_canvas_create(lv_obj_create(NULL));
    uint8_t *rot_buf = heap_caps_malloc(disp_x * disp_y / 8 + 8, MALLOC_CAP_DMA);
    memset(rot_buf, 0x00, disp_x * disp_y / 8 + 8);

    if (rotate == LV_DISP_ROT_90 || rotate == LV_DISP_ROT_270) {
#if 1    // use my rotate function
        // ROT_270 is performed by ROT90 and then mirror both X & Y (in main.c)
        rotate_buffer(rotate, disp_x, disp_y, canvas_buffer, rot_buf);
        lv_canvas_set_buffer(canvas, rot_buf, width, height, LV_IMG_CF_ALPHA_1BIT);

#else    // use lvgl_canvas_transform
        // Display of 90 appears as 0 degrees, and clipped on right side.
        // Display of 270 appears as 180 degrees, and clipped on left side of display.
        // The text is first written to a landscape canvas. Thus it seems that no rotation occurs,
        // thereby resulting in the text clipping.
        int angle = 900 * rotate;

        lv_canvas_set_buffer(rot_canvas, rot_buf, height, width, LV_IMG_CF_ALPHA_1BIT);
        lv_img_dsc_t img_desc = *lv_canvas_get_img(canvas);
        lv_canvas_transform(rot_canvas, &img_desc, angle, 256, 0, 264, width / 2, height / 2, true);
        lv_canvas_copy_buf(canvas, rot_buf, 0, 0, lv_obj_get_height(rot_canvas), lv_obj_get_width(rot_canvas));
#endif
    }
    // Must clear LVGL display, else we get "double-exposure" image on screen
    lv_obj_clean(lv_scr_act());
    // Attach the canvas to the active screen (the display device).
    // This presumably triggers LVGL flush callback (see flush_cb in main.c)
    lv_obj_set_parent(canvas, lv_scr_act());
}
