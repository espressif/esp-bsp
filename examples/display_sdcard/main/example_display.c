/*
 * SPDX-FileCopyrightText: 2022-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "dirent.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/esp-bsp.h"
#include "esp_vfs_fat.h"
#include "lvgl.h"

#include "example_display.h"

typedef enum : uint32_t {NOTIFY_FORMAT, } notif_task_t;

static const char *TAG = "example_display";

static void example_display_progress_screen(const char *message);

static lv_obj_t *progress_screen = NULL;
static lv_obj_t *main_screen = NULL;
static lv_obj_t *confirm_mbox;

static TaskHandle_t main_task_handle = {0};

/* lv_list and lv_win are deprecated in LVGL 9.6. Build both from flex containers. */

static void example_obj_set_scrollable(lv_obj_t *obj, bool en)
{
#if LV_VERSION_CHECK(9, 6, 0)
    lv_obj_set_scrollable(obj, en);
#else
    if (en) {
        lv_obj_add_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    } else {
        lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    }
#endif
}

static lv_obj_t *example_list_create(lv_obj_t *parent)
{
    lv_obj_t *list = lv_obj_create(parent);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(list, 0, 0);
    lv_obj_set_style_pad_gap(list, 0, 0);
    lv_obj_set_style_radius(list, 0, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    return list;
}

static lv_obj_t *example_list_add_text(lv_obj_t *list, const char *text)
{
    lv_obj_t *label = lv_label_create(list);
    lv_obj_set_width(label, LV_PCT(100));
    lv_obj_set_style_bg_opa(label, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(label, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);
    lv_obj_set_style_text_color(label, lv_palette_darken(LV_PALETTE_GREY, 4), 0);
    lv_obj_set_style_pad_all(label, 8, 0);
    lv_label_set_text(label, text);
    return label;
}

static lv_obj_t *example_win_create(lv_obj_t *parent)
{
    lv_obj_t *win = lv_obj_create(parent);
    lv_obj_set_size(win, lv_obj_get_width(parent), lv_obj_get_height(parent));
    lv_obj_set_flex_flow(win, LV_FLEX_FLOW_COLUMN);
    example_obj_set_scrollable(win, false);
    lv_obj_set_style_pad_all(win, 0, 0);
    lv_obj_set_style_pad_gap(win, 0, 0);
    lv_obj_set_style_border_width(win, 0, 0);
    lv_obj_set_style_radius(win, 0, 0);
    lv_obj_set_style_shadow_width(win, 0, 0);

    lv_obj_t *header = lv_obj_create(win);
    lv_obj_set_size(header, LV_PCT(100), lv_display_get_dpi(lv_obj_get_display(win)) / 2);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(header, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    example_obj_set_scrollable(header, false);
    lv_obj_set_style_pad_all(header, 2, 0);
    lv_obj_set_style_pad_gap(header, 2, 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_radius(header, 0, 0);
    lv_obj_set_style_bg_opa(header, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(header, lv_palette_lighten(LV_PALETTE_GREY, 2), 0);

    lv_obj_t *content = lv_obj_create(win);
    lv_obj_set_width(content, LV_PCT(100));
    lv_obj_set_flex_grow(content, 1);
    lv_obj_set_style_border_width(content, 0, 0);
    lv_obj_set_style_radius(content, 0, 0);
    lv_obj_set_style_shadow_width(content, 0, 0);

    return win;
}

static lv_obj_t *example_win_get_header(lv_obj_t *win)
{
    return lv_obj_get_child(win, 0);
}

static lv_obj_t *example_win_get_content(lv_obj_t *win)
{
    return lv_obj_get_child(win, 1);
}

static void example_win_add_title(lv_obj_t *win, const char *text)
{
    lv_obj_t *title = lv_label_create(example_win_get_header(win));
    lv_label_set_long_mode(title, LV_LABEL_LONG_MODE_DOTS);
    lv_label_set_text(title, text);
    lv_obj_set_flex_grow(title, 1);
}

static lv_obj_t *example_win_add_button(lv_obj_t *win, const char *icon, int32_t btn_w)
{
    lv_obj_t *btn = lv_button_create(example_win_get_header(win));
    lv_obj_set_size(btn, btn_w, LV_PCT(100));

    if (icon) {
        lv_obj_t *img = lv_image_create(btn);
        lv_image_set_src(img, icon);
        lv_obj_center(img);
    }

    return btn;
}

// Callback for close buttons
static void example_close_btn_cb (lv_event_t *evt)
{
    lv_obj_t *close_obj = lv_event_get_user_data(evt);
    lv_obj_delete(close_obj);
}

// Callback for confirmation of formatting
static void example_confirm_btn_cb (lv_event_t *evt)
{
    // Show loading screen
    example_display_progress_screen("Formatting\nthe SD card...");
    lv_obj_delete(confirm_mbox);
    // Send SD card formatting request to the main task
    xTaskNotify(main_task_handle, (uint32_t)NOTIFY_FORMAT, eSetValueWithOverwrite);
}

// Callback for requesting formatting which asks for confirmation
static void example_format_btn_cb (lv_event_t *evt)
{
    lv_obj_t *label;
    lv_obj_t *button;
    lv_obj_t *content;

    // Create and show a message box to confirm formatting
    confirm_mbox = lv_msgbox_create(main_screen);
    lv_msgbox_add_title(confirm_mbox, "Warning");
    lv_msgbox_add_close_button(confirm_mbox);
    button = lv_msgbox_add_footer_button(confirm_mbox, LV_SYMBOL_CHARGE " YES! " LV_SYMBOL_CHARGE);
    lv_obj_add_event_cb(button, example_confirm_btn_cb, LV_EVENT_CLICKED, NULL);
    content = lv_msgbox_get_content(confirm_mbox);
    label = lv_label_create(content);
    lv_label_set_text(label, "Are you sure that you want\nto format the SD card?\nYou will lose all of your data");
}


// Callback for displaying files inside the SD card
static void example_files_btn_cb (lv_event_t *evt)
{
    DIR *directory;
    struct dirent *entry;
    lv_obj_t *files_window;
    lv_obj_t *window_content;
    lv_obj_t *close_button;
    lv_obj_t *dir_list;

    // Create and show a window for a list of files
    files_window = example_win_create(lv_screen_active());
    example_win_add_title(files_window, "  List of files");
    close_button = example_win_add_button(files_window, LV_SYMBOL_CLOSE, 60);
    lv_obj_add_event_cb(close_button, example_close_btn_cb, LV_EVENT_CLICKED, files_window);

    // Create the list of files
    window_content = example_win_get_content(files_window);
    dir_list = example_list_create(window_content);
    lv_obj_set_size(dir_list, 280, 120);
    lv_obj_center(dir_list);

    // Read names of files in the SD card mount directory and add them to the list
    directory = opendir(BSP_SD_MOUNT_POINT);
    if (directory) {
        while ((entry = readdir(directory)) != NULL) {
            example_list_add_text(dir_list, entry->d_name);
        }
        closedir(directory);
    }
}

// Callback for displaying SD card information
static void example_info_btn_cb (lv_event_t *evt)
{
    char used_size_units[3];
    char table_cell_buf[16];
    unsigned int full_capacity_mb;
    unsigned int filesystem_total_mb;
    unsigned int used_size;
    sdmmc_card_t *sdcard;
    uint64_t total_bytes;
    uint64_t free_bytes;
    lv_obj_t *info_window;
    lv_obj_t *window_content;
    lv_obj_t *close_button;
    lv_obj_t *info_table;

    // Get capacity specified by the SD card
    sdcard = bsp_sdcard_get_handle();
    full_capacity_mb = ((uint64_t)sdcard->csd.capacity * (uint64_t)sdcard->csd.sector_size) / 1000000;

    // Get capacity specified by the file system
    esp_vfs_fat_info(BSP_SD_MOUNT_POINT, &total_bytes, &free_bytes);
    filesystem_total_mb = total_bytes / 1000000;
    used_size = total_bytes - free_bytes;
    if (used_size < 1000) {
        snprintf(used_size_units, 3, "B");
    } else if (used_size < 1000000) {
        used_size = used_size / 1000;
        snprintf(used_size_units, 3, "kB");
    } else {
        used_size = used_size / 1000000;
        snprintf(used_size_units, 3, "MB");
    }

    // Create and show window with a table of information
    info_window = example_win_create(lv_screen_active());
    example_win_add_title(info_window, "  SD card information");
    close_button = example_win_add_button(info_window, LV_SYMBOL_CLOSE, 60);
    lv_obj_add_event_cb(close_button, example_close_btn_cb, LV_EVENT_CLICKED, info_window);
    window_content = example_win_get_content(info_window);
    info_table = lv_table_create(window_content);
    lv_obj_center(info_table);

    snprintf(table_cell_buf, 16, "%6d MB", full_capacity_mb);
    lv_table_set_cell_value(info_table, 0, 0, "Full capacity:");
    lv_table_set_cell_value(info_table, 0, 1, table_cell_buf);

    snprintf(table_cell_buf, 16, "%6d MB", filesystem_total_mb);
    lv_table_set_cell_value(info_table, 1, 0, "File system capacity:");
    lv_table_set_cell_value(info_table, 1, 1, table_cell_buf);

    snprintf(table_cell_buf, 16, "%6d %s", used_size, used_size_units);
    lv_table_set_cell_value(info_table, 2, 0, "Used capacity:");
    lv_table_set_cell_value(info_table, 2, 1, table_cell_buf);
}

// Show progress screen with a message string
static void example_display_progress_screen(const char *message)
{
    lv_obj_t *label;
    lv_obj_t *spinner;

    progress_screen = lv_obj_create(NULL);

    // Create message label (has to be null terminated string)
    label = lv_label_create(progress_screen);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_26, 0);
    lv_label_set_text_fmt(label, "%s", message);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 30);

    // Create spinner with animation
    spinner = lv_spinner_create(progress_screen);
    lv_obj_set_size(spinner, 80, 80);
    lv_obj_align(spinner, LV_ALIGN_CENTER, 0, 30);
    lv_spinner_set_anim_params(spinner, 750, 200);

    lv_screen_load(progress_screen);
}

// Initialize the display and show the initial screen
esp_err_t example_display_init()
{
    esp_err_t err = ESP_OK;
    // This function is only called by the main task
    main_task_handle = xTaskGetCurrentTaskHandle();

    ESP_RETURN_ON_ERROR(!bsp_display_start(), TAG, "Failed to start a display");

    err = bsp_display_backlight_on();
    if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
        ESP_RETURN_ON_ERROR(err, TAG, "Failed to turn on backlight");
    }

    bsp_display_lock(0);
    example_display_progress_screen("Mounting and testing\nthe SD card...");
    bsp_display_unlock();

    return ESP_OK;
}

// Poll for execution in the main task
void example_display_poll()
{
    esp_err_t err;
    notif_task_t notification_value;
    // Receive an execution requests which would otherwise block the LVGL task
    xTaskNotifyWait(ULONG_MAX, 0, (uint32_t *)&notification_value, portMAX_DELAY);
    switch (notification_value) {
    case NOTIFY_FORMAT:
        // Format the SD card and return to the main screen
        err = esp_vfs_fat_sdcard_format(BSP_SD_MOUNT_POINT, bsp_sdcard_get_handle());
        ESP_ERROR_CHECK(err);
        example_display_main_screen(false);
        break;
    default:
        ESP_LOGW(TAG, "Reached undefined enum value");
        break;
    }
}

// Show the main screen
void example_display_main_screen(bool show_test_box)
{
    lv_obj_t *label;
    lv_obj_t *button;
    lv_obj_t *mbox;
    lv_obj_t *content;

    bsp_display_lock(0);

    main_screen = lv_obj_create(NULL);

    // Create a button to request formatting
    button = lv_button_create(main_screen);
    lv_obj_add_event_cb(button, example_format_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_align(button, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_size(button, 280, 60);
    lv_obj_set_style_bg_color(button, lv_color_hex(0xc90606), 0);
    label = lv_label_create(button);
    lv_label_set_text(label, LV_SYMBOL_WARNING "  Format SD card  " LV_SYMBOL_WARNING);
    lv_obj_center(label);

    // Create a button to display information about the SD card
    button = lv_button_create(main_screen);
    lv_obj_add_event_cb(button, example_info_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_align(button, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
    lv_obj_set_size(button, 130, 120);
    label = lv_label_create(button);
    lv_label_set_text(label, "Show\ninfo");
    lv_obj_center(label);

    // Create a button to show a list of files
    button = lv_button_create(main_screen);
    lv_obj_add_event_cb(button, example_files_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_align(button, LV_ALIGN_BOTTOM_LEFT, 20, -20);
    lv_obj_set_size(button, 130, 120);
    label = lv_label_create(button);
    lv_label_set_text(label, "Show\nfiles");
    lv_obj_center(label);

    // When requested then show a message box that the test has passed
    if (show_test_box) {
        mbox = lv_msgbox_create(main_screen);
        lv_msgbox_add_title(mbox, "Test result");
        lv_msgbox_add_close_button(mbox);
        content = lv_msgbox_get_content(mbox);
        label = lv_label_create(content);
        lv_label_set_text(label, "\n"LV_SYMBOL_OK LV_SYMBOL_OK " SD card testing passed " LV_SYMBOL_OK LV_SYMBOL_OK "\n");
    }

    lv_screen_load(main_screen);
    lv_obj_delete(progress_screen);
    bsp_display_unlock();
}

// Show fail screen
void example_display_failed_test_screen(const esp_err_t err)
{
    lv_obj_t *label;
    lv_obj_t *fail_screen;

    bsp_display_lock(0);

    fail_screen = lv_obj_create(NULL);

    // Set red background color
    lv_obj_set_style_bg_color(fail_screen, lv_color_hex(0xc90606), 0);

    // Create a label with fail reason
    label = lv_label_create(fail_screen);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
    lv_label_set_text_fmt(label, "Failed to mount and test.\nthe SD card\n%s", esp_err_to_name(err));
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    lv_screen_load(fail_screen);
    lv_obj_delete(progress_screen);
    bsp_display_unlock();
}
