/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "esp_wrover_kit.h"
#include "esp_vfs_fat.h"
#include "lvgl.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"

#define TAG "Wrover"

esp_err_t bsp_leds_init(void)
{
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BIT64(BSP_LED_RED) | BIT64(BSP_LED_GREEN) | BIT64(BSP_LED_BLUE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&led_io_config);
}

esp_err_t bsp_led_set(const bsp_led_t led_io, const bool on)
{
    return gpio_set_level((gpio_num_t) led_io, (uint32_t) on);
}

sdmmc_card_t *bsp_sdcard = NULL;
esp_err_t bsp_sdcard_mount(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_uSD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;

    return esp_vfs_fat_sdmmc_mount(CONFIG_BSP_uSD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(CONFIG_BSP_uSD_MOUNT_POINT, bsp_sdcard);
}

esp_err_t bsp_button_init(void)
{
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BIT64(BSP_BUTTON_IO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&led_io_config);
}

bool bsp_button_get(void)
{
    return !(bool)gpio_get_level(BSP_BUTTON_IO);
}

#define LV_TICK_PERIOD_MS 1
static void lv_tick(void *arg)
{
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}

static bool bsp_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, void *user_data, void *event_data)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_data;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void bsp_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

#define BSP_LCD_BK_LIGHT_ON_LEVEL  0
#define BSP_LCD_BK_LIGHT_OFF_LEVEL !BSP_LCD_BK_LIGHT_ON_LEVEL
#define BSP_PIN_NUM_CS             4
#define BSP_PIN_NUM_DC             5
#define BSP_PIN_NUM_RST            18
#define BSP_PIN_NUM_BK_LIGHT       5

// The pixel number in horizontal and vertical
#define BSP_LCD_H_RES              240
#define BSP_LCD_V_RES              320

void bsp_display_task(void *pvParameter)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BSP_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    gpio_set_level(BSP_PIN_NUM_BK_LIGHT, BSP_LCD_BK_LIGHT_OFF_LEVEL);

    spi_bus_config_t buscfg = {
        .sclk_io_num = 19,
        .mosi_io_num = 23,
        .miso_io_num = 25,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BSP_LCD_H_RES * 20 * sizeof(lv_color_t)
    };
    ESP_ERROR_CHECK(spi_bus_initialize(1, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = 21,
        .cs_gpio_num = 22,
        .pclk_hz = (40 * 1000 * 1000),
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .on_color_trans_done = bsp_notify_lvgl_flush_ready,
        .user_data = &disp_drv
    };
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) 1, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install LCD driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_PIN_NUM_RST,
#ifdef CONFIG_BSP_LCD_ILI9341
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
#else
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
#endif
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
#ifdef CONFIG_BSP_LCD_ILI9341
    esp_lcd_panel_mirror(panel_handle, true, false);
#endif

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(BSP_PIN_NUM_BK_LIGHT, BSP_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(BSP_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(BSP_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, BSP_LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = BSP_LCD_H_RES;
    disp_drv.ver_res = BSP_LCD_V_RES;
    disp_drv.flush_cb = bsp_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_drv_register(&disp_drv);

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick,
        .name = "LVGL tick"
    };
    esp_timer_handle_t lvgl_tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LV_TICK_PERIOD_MS * 1000));

    /* Notify parent task that LVGL is ready, if parent task is provided */
    if (pvParameter != NULL) {
        xTaskNotifyGive((TaskHandle_t)pvParameter);
    }

    while (1) {
        lv_task_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* A task should NEVER return */
    esp_timer_stop(lvgl_tick_timer);
    esp_timer_delete(lvgl_tick_timer);
    free(buf1);
    free(buf2);
    vTaskDelete(NULL);
}
