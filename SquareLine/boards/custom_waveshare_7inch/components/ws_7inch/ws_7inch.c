/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "bsp/ws_7inch.h"
#include "esp_lcd_ra8875.h"
#include "esp_lcd_touch_gt911.h"
#include "lvgl.h"

static const char *TAG = "WS7INCH";

/* Assert on error, if selected in menuconfig. Otherwise return error code. */
//TODO: Move this code into common header
#if CONFIG_BSP_ERROR_CHECK
#define BSP_ERROR_CHECK_RETURN_ERR(x)    ESP_ERROR_CHECK(x)
#define BSP_ERROR_CHECK_RETURN_NULL(x)   ESP_ERROR_CHECK(x)
#define BSP_NULL_CHECK(x, ret)           assert(x)
#define BSP_NULL_CHECK_GOTO(x, goto_tag) assert(x)
#else
#define BSP_ERROR_CHECK_RETURN_ERR(x) do { \
        esp_err_t err_rc_ = (x);            \
        if (unlikely(err_rc_ != ESP_OK)) {  \
            return err_rc_;                 \
        }                                   \
    } while(0)

#define BSP_ERROR_CHECK_RETURN_NULL(x)  do { \
        if (unlikely((x) != ESP_OK)) {      \
            return NULL;                    \
        }                                   \
    } while(0)

#define BSP_NULL_CHECK(x, ret)      do { \
        if ((x) == NULL) {      \
            return ret;                    \
        }                                   \
    } while(0)

#define BSP_NULL_CHECK_GOTO(x, goto_tag) do { \
        if ((x) == NULL) {      \
            goto goto_tag;      \
        }                       \
    } while(0)
#endif

static lv_disp_draw_buf_t disp_buf; // Contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;      // Contains callback functions
static esp_lcd_touch_handle_t tp;   // LCD touch handle
static SemaphoreHandle_t lvgl_mux;  // LVGL mutex

esp_err_t bsp_i2c_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = BSP_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = BSP_I2C_CLK_SPEED_HZ
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_param_config(BSP_I2C_NUM, &i2c_conf));
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_install(BSP_I2C_NUM, i2c_conf.mode, 0, 0, 0));

    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_driver_delete(BSP_I2C_NUM));
    return ESP_OK;
}

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           16
#define LCD_PARAM_BITS         8
#define LVGL_TICK_PERIOD_MS    CONFIG_BSP_DISPLAY_LVGL_TICK

static void bsp_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t) indev_drv->user_data;
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    assert(tp);


    /* Read data from touch controller into memory */
    esp_lcd_touch_read_data(tp);

    /* Read data from touch controller */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static bool lvgl_port_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_port_flush_callback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        break;
    }
}

static void lvgl_port_tick_increment(void *arg)
{
    /* Tell LVGL how many milliseconds have elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static esp_err_t lvgl_port_tick_init(void)
{
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_port_tick_increment,
        .name = "LVGL tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    BSP_ERROR_CHECK_RETURN_ERR(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    return esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000);
}

static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    while (1) {
        bsp_display_lock(0);
        uint32_t task_delay_ms = lv_timer_handler();
        bsp_display_unlock();
        if (task_delay_ms > 500) {
            task_delay_ms = 500;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

static lv_disp_t *lvgl_port_display_init(void)
{
    ESP_LOGD(TAG, "Initialize Intel 8080 bus");
    /* Init Intel 8080 bus */
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .dc_gpio_num = BSP_LCD_DC,
        .wr_gpio_num = BSP_LCD_WR,
        .data_gpio_nums = {
            BSP_LCD_DB0,
            BSP_LCD_DB1,
            BSP_LCD_DB2,
            BSP_LCD_DB3,
            BSP_LCD_DB4,
            BSP_LCD_DB5,
            BSP_LCD_DB6,
            BSP_LCD_DB7,
            BSP_LCD_DB8,
            BSP_LCD_DB9,
            BSP_LCD_DB10,
            BSP_LCD_DB11,
            BSP_LCD_DB12,
            BSP_LCD_DB13,
            BSP_LCD_DB14,
            BSP_LCD_DB15,
        },
        .bus_width = BSP_LCD_WIDTH,
        .max_transfer_bytes = (BSP_LCD_H_RES) * 80 * sizeof(uint16_t),
        .psram_trans_align = 64,
        .sram_trans_align = 4,
    };
    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    ESP_LOGD(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i80_config_t io_config = {
        .cs_gpio_num = BSP_LCD_CS,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 1,
            .dc_dummy_level = 0,
            .dc_data_level = 0,
        },
        .flags = {
            .swap_color_bytes = 1,
            .pclk_idle_low = 0,
        },
        .on_color_trans_done = lvgl_port_flush_ready,
        .user_ctx = &disp_drv,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
    };
    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

    ESP_LOGD(TAG, "Install LCD driver of RA8875");
    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_ra8875_config_t vendor_config = {
        .wait_gpio_num = BSP_LCD_WAIT,
        .lcd_width = BSP_LCD_H_RES,
        .lcd_height = BSP_LCD_V_RES,
        .mcu_bit_interface = BSP_LCD_WIDTH,
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
        .vendor_config = (void *) &vendor_config,
    };
    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_new_panel_ra8875(io_handle, &panel_config, &panel_handle));

    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_panel_reset(panel_handle));
    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_panel_init(panel_handle));
    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_panel_disp_on_off(panel_handle, true));

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(BSP_LCD_H_RES * 50 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    BSP_NULL_CHECK(buf1, NULL);
    lv_color_t *buf2 = heap_caps_malloc(BSP_LCD_H_RES * 50 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    BSP_NULL_CHECK_GOTO(buf2, ERR);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, BSP_LCD_H_RES * 50);

    ESP_LOGD(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = BSP_LCD_H_RES;
    disp_drv.ver_res = BSP_LCD_V_RES;
    disp_drv.flush_cb = lvgl_port_flush_callback;
    disp_drv.drv_update_cb = lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    return lv_disp_drv_register(&disp_drv);

#if (!CONFIG_BSP_ERROR_CHECK)
ERR:
    if (buf1) {
        free(buf1);
    }
    if (buf2) {
        free(buf2);
    }

    return NULL;
#endif
}

static esp_err_t lvgl_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv_tp;
    lv_indev_t *indev_touchpad;

    /* Initialize touch */
    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC, // Shared with LCD reset
        .int_gpio_num = BSP_LCD_TP_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)BSP_I2C_NUM, &tp_io_config, &tp_io_handle));
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));
    assert(tp);

    /* Register a touchpad input device */
    lv_indev_drv_init(&indev_drv_tp);
    indev_drv_tp.type = LV_INDEV_TYPE_POINTER;
    indev_drv_tp.read_cb = bsp_touchpad_read;
    indev_drv_tp.user_data = tp;
    indev_touchpad = lv_indev_drv_register(&indev_drv_tp);
    BSP_NULL_CHECK(indev_touchpad, ESP_ERR_NO_MEM);

    return ESP_OK;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t bsp_display_backlight_off(void)
{
    return bsp_display_brightness_set(0);
}

esp_err_t bsp_display_backlight_on(void)
{
    return bsp_display_brightness_set(100);
}

lv_disp_t *bsp_display_start(void)
{
    lv_init();
    //bsp_display_brightness_init();
    lv_disp_t *disp = lvgl_port_display_init();
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_indev_init());
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_tick_init());
    lvgl_mux = xSemaphoreCreateMutex();
    BSP_NULL_CHECK(lvgl_mux, NULL);
    xTaskCreate(lvgl_port_task, "LVGL task", 4096, NULL, CONFIG_BSP_DISPLAY_LVGL_TASK_PRIORITY, NULL);
    return disp;
}

void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    assert(lvgl_mux && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

void bsp_display_unlock(void)
{
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
}
