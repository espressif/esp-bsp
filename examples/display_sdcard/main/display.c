/*
 * SPDX-FileCopyrightText: 2022-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "dirent.h"
#include "font/lv_symbol_def.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/esp-bsp.h"
#include "esp_vfs_fat.h"
#include "core/lv_obj_tree.h"
#include "widgets/label/lv_label.h"
#include "font/lv_font.h"
#include "misc/lv_area.h"
#include "misc/lv_event.h"
#include "widgets/msgbox/lv_msgbox.h"
#include "widgets/table/lv_table.h"

#include "display.h"

static const char *TAG = "example_display";

static lv_obj_t *progress_screen = NULL;
static lv_obj_t *main_screen = NULL;
static lv_obj_t *confirm_mbox;

static bool format_request = false;
static bool format_confirm = false;
static bool show_info = false;
static bool show_files = false;

static void display_btn_cb (lv_event_t *evt)
{
    bool *flag = lv_event_get_user_data(evt);
    *flag = true;
}

static void close_btn_cb (lv_event_t *evt)
{
    lv_obj_t *close_obj = lv_event_get_user_data(evt);
    lv_obj_delete(close_obj);
}

static void display_progress(const char *message)
{
    lv_obj_t *label;
    lv_obj_t *spinner;

    bsp_display_lock(0);

    progress_screen = lv_obj_create(NULL);

    label = lv_label_create(progress_screen);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_26, 0);
    lv_label_set_text_fmt(label, "%s", message);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 30);

    spinner = lv_spinner_create(progress_screen);
    lv_obj_set_size(spinner, 80, 80);
    lv_obj_align(spinner, LV_ALIGN_CENTER, 0, 30);
    lv_spinner_set_anim_params(spinner, 750, 200);

    lv_scr_load(progress_screen);

    bsp_display_unlock();
}

static void display_files_window()
{
    DIR *directory;
    struct dirent *entry;
    lv_obj_t *files_window;
    lv_obj_t *window_content;
    lv_obj_t *close_button;
    lv_obj_t *dir_list;

    bsp_display_lock(0);
    files_window = lv_win_create(lv_screen_active());
    lv_win_add_title(files_window, "  List of files");
    close_button = lv_win_add_button(files_window, LV_SYMBOL_CLOSE, 60);
    lv_obj_add_event_cb(close_button, close_btn_cb, LV_EVENT_CLICKED, files_window);

    window_content = lv_win_get_content(files_window);
    dir_list = lv_list_create(window_content);
    lv_obj_set_size(dir_list, 280, 120);
    lv_obj_center(dir_list);

    ESP_LOGI(TAG, "Updating directory list");
    directory = opendir(BSP_SD_MOUNT_POINT);
    if (directory) {
        while ((entry = readdir(directory)) != NULL) {
            ESP_LOGI(TAG, "%s", entry->d_name);
            lv_list_add_text(dir_list, entry->d_name);
        }
        closedir(directory);
    }

    bsp_display_unlock();
}

static void display_info_window()
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

    sdcard = bsp_sdcard_get_handle();
    full_capacity_mb = ((uint64_t)sdcard->csd.capacity * (uint64_t)sdcard->csd.sector_size) / 1000000;
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

    bsp_display_lock(0);
    info_window = lv_win_create(lv_screen_active());
    lv_win_add_title(info_window, "  SD card information");
    close_button = lv_win_add_button(info_window, LV_SYMBOL_CLOSE, 60);
    lv_obj_add_event_cb(close_button, close_btn_cb, LV_EVENT_CLICKED, info_window);
    window_content = lv_win_get_content(info_window);
    info_table = lv_table_create(window_content);

    snprintf(table_cell_buf, 16, "%6d MB", full_capacity_mb);
    lv_table_set_cell_value(info_table, 0, 0, "Full capacity:");
    lv_table_set_cell_value(info_table, 0, 1, table_cell_buf);

    snprintf(table_cell_buf, 16, "%6d MB", filesystem_total_mb);
    lv_table_set_cell_value(info_table, 1, 0, "File system capacity:");
    lv_table_set_cell_value(info_table, 1, 1, table_cell_buf);

    snprintf(table_cell_buf, 16, "%6d %s", used_size, used_size_units);
    lv_table_set_cell_value(info_table, 2, 0, "Used capacity:");
    lv_table_set_cell_value(info_table, 2, 1, table_cell_buf);

    bsp_display_unlock();
}

void display_main()
{
    lv_obj_t *label;
    lv_obj_t *button;
    lv_obj_t *mbox;
    lv_obj_t *content;

    bsp_display_lock(0);

    main_screen = lv_obj_create(NULL);

    button = lv_btn_create(main_screen);
    lv_obj_add_event_cb(button, display_btn_cb, LV_EVENT_CLICKED, &format_request);
    lv_obj_align(button, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_size(button, 280, 60);
    lv_obj_set_style_bg_color(button, lv_color_hex(0xc90606), 0);
    label = lv_label_create(button);
    lv_label_set_text(label, LV_SYMBOL_WARNING "  Format SD card  " LV_SYMBOL_WARNING);
    lv_obj_center(label);

    button = lv_btn_create(main_screen);
    lv_obj_add_event_cb(button, display_btn_cb, LV_EVENT_CLICKED, &show_info);
    lv_obj_align(button, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
    lv_obj_set_size(button, 130, 120);
    label = lv_label_create(button);
    lv_label_set_text(label, "Show\ninfo");
    lv_obj_center(label);

    button = lv_btn_create(main_screen);
    lv_obj_add_event_cb(button, display_btn_cb, LV_EVENT_CLICKED, &show_files);
    lv_obj_align(button, LV_ALIGN_BOTTOM_LEFT, 20, -20);
    lv_obj_set_size(button, 130, 120);
    label = lv_label_create(button);
    lv_label_set_text(label, "Show\nfiles");
    lv_obj_center(label);

    mbox = lv_msgbox_create(main_screen);
    lv_msgbox_add_title(mbox, "Test result");
    lv_msgbox_add_close_button(mbox);
    content = lv_msgbox_get_content(mbox);
    label = lv_label_create(content);
    lv_label_set_text(label, "\n"LV_SYMBOL_OK LV_SYMBOL_OK " SD card testing passed " LV_SYMBOL_OK LV_SYMBOL_OK "\n");

    lv_scr_load(main_screen);
    // TODO: It crashes when deleting the allocated screen
    // lv_obj_delete(progress_screen);
    bsp_display_unlock();
}

static void display_confirm()
{
    lv_obj_t *label;
    lv_obj_t *button;
    lv_obj_t *content;

    bsp_display_lock(0);

    confirm_mbox = lv_msgbox_create(main_screen);
    lv_msgbox_add_title(confirm_mbox, "Warning");
    lv_msgbox_add_close_button(confirm_mbox);
    button = lv_msgbox_add_footer_button(confirm_mbox, LV_SYMBOL_CHARGE " YES! " LV_SYMBOL_CHARGE);
    lv_obj_add_event_cb(button, display_btn_cb, LV_EVENT_CLICKED, &format_confirm);
    content = lv_msgbox_get_content(confirm_mbox);
    label = lv_label_create(content);
    lv_label_set_text(label, "Are you sure that you want\nto format the SD card?\nYou will lose all of your data");

    bsp_display_unlock();
}

void display_failed_test(const esp_err_t err)
{
    lv_obj_t *label;
    lv_obj_t *fail_screen;

    bsp_display_lock(0);

    fail_screen = lv_obj_create(NULL);

    lv_obj_set_style_bg_color(fail_screen, lv_color_hex(0xc90606), 0);

    label = lv_label_create(fail_screen);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_20, 0);
    lv_label_set_text_fmt(label, "Failed to mount and test.\nthe SD card\n%s", esp_err_to_name(err));
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    lv_scr_load(fail_screen);
    lv_obj_delete(progress_screen);
    bsp_display_unlock();
}

void display_init()
{
    if (!bsp_display_start()) {
        ESP_LOGE(TAG, "Failed to start a display");
    }

    bsp_display_backlight_on();

    display_progress("Mounting and testing\nthe SD card...");
}

void display_dispatch()
{
    if (format_request) {
        format_request = false;
        display_confirm();
    }
    if (format_confirm) {
        format_confirm = false;
        display_progress("Formatting\nthe SD card...");
        lv_obj_delete(confirm_mbox);
        esp_err_t err = esp_vfs_fat_sdcard_format(BSP_SD_MOUNT_POINT, bsp_sdcard_get_handle());
        ESP_ERROR_CHECK(err);
        vTaskDelay(pdMS_TO_TICKS(1000));
        bsp_display_lock(0);
        lv_scr_load(main_screen);
        lv_obj_delete(progress_screen);
        bsp_display_unlock();
    }
    if (show_info) {
        show_info = false;
        display_info_window();
    }
    if (show_files) {
        show_files = false;
        display_files_window();
    }
}
