/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_io_expander_tca9554.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch_gt1151.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"

#include "bsp/esp32_s3_lcd_ev_board.h"
#include "bsp_err_check.h"
#include "bsp_sub_board.h"

static const char *TAG = "S3-LCD-EV-BOARD";

static esp_lcd_touch_handle_t tp = NULL;            // LCD touch panel handle
static esp_io_expander_handle_t io_expander = NULL; // IO expander tca9554 handle
static lv_disp_draw_buf_t disp_buf;                 // Contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;                      // Contains LCD panel handle and callback functions
static SemaphoreHandle_t lvgl_mux;                  // LVGL mutex
static TaskHandle_t lvgl_task_handle;

/**************************************************************************************************
 *
 * I2C Function
 *
 **************************************************************************************************/
esp_err_t bsp_i2c_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = BSP_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = CONFIG_BSP_I2C_CLK_SPEED_HZ
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

/**************************************************************************************************
 *
 * IO Expander Function
 *
 **************************************************************************************************/
esp_io_expander_handle_t bsp_io_expander_init(void)
{
    if (io_expander) {
        ESP_LOGW(TAG, "io_expander is initialized");
    } else {
        BSP_ERROR_CHECK_RETURN_NULL(esp_io_expander_new_i2c_tca9554(BSP_I2C_NUM, BSP_IO_EXPANDER_I2C_ADDRESS, &io_expander));
    }

    return io_expander;
}

/**************************************************************************************************
 *
 * I2S Audio Function
 *
 **************************************************************************************************/
esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config, i2s_chan_handle_t *tx_channel, i2s_chan_handle_t *rx_channel)
{
    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    BSP_ERROR_CHECK_RETURN_ERR(i2s_new_channel(&chan_cfg, tx_channel, rx_channel));

    /* Setup I2S channels */
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(22050);
    const i2s_std_config_t *p_i2s_cfg = &std_cfg_default;
    if (i2s_config != NULL) {
        p_i2s_cfg = i2s_config;
    }

    if (tx_channel != NULL) {
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_init_std_mode(*tx_channel, p_i2s_cfg));
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_enable(*tx_channel));
    }
    if (rx_channel != NULL) {
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_init_std_mode(*rx_channel, p_i2s_cfg));
        BSP_ERROR_CHECK_RETURN_ERR(i2s_channel_enable(*rx_channel));
    }

    /* Setup power amplifier pin */
    if (!io_expander) {
        BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_new_i2c_tca9554(BSP_I2C_NUM, BSP_IO_EXPANDER_I2C_ADDRESS, &io_expander));
    }
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(io_expander, BSP_POWER_AMP_IO, true));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, BSP_POWER_AMP_IO, false));

    return ESP_OK;
}

esp_err_t bsp_audio_poweramp_enable(bool enable)
{
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, BSP_POWER_AMP_IO, (uint8_t)enable));

    return ESP_OK;
}

/**********************************************************************************************************
 *
 * Display Function
 *
 **********************************************************************************************************/
#if CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
// This function is located in ROM (also see esp_rom/${target}/ld/${target}.rom.ld)
extern int Cache_WriteBack_Addr(uint32_t addr, uint32_t size);

/**
 * @brief Copy dirty area
 *
 * @note This function is used to avoid tearing effect, and only work with LVGL direct-mode.
 *
 */
static void lv_port_direct_mode_copy(void)
{
    lv_disp_t *disp_refr = _lv_refr_get_disp_refreshing();

    uint8_t *buf_act = disp_refr->driver->draw_buf->buf_act;
    uint8_t *buf1 = disp_refr->driver->draw_buf->buf1;
    uint8_t *buf2 = disp_refr->driver->draw_buf->buf2;
    int h_res = disp_refr->driver->hor_res;
    int v_res = disp_refr->driver->ver_res;

    uint8_t *fb_from = buf_act;
    uint8_t *fb_to = (fb_from == buf1) ? buf2 : buf1;

    lv_coord_t x_start, x_end, y_start, y_end;
    uint32_t copy_bytes_per_line;
    uint32_t bytes_to_flush;
    uint32_t bytes_per_line = h_res * 2;
    uint8_t *from, *to;
    uint8_t *flush_ptr;
    for (int i = 0; i < disp_refr->inv_p; i++) {
        /* Refresh the unjoined areas*/
        if (disp_refr->inv_area_joined[i] == 0) {
            x_start = disp_refr->inv_areas[i].x1;
            x_end = disp_refr->inv_areas[i].x2 + 1;
            y_start = disp_refr->inv_areas[i].y1;
            y_end = disp_refr->inv_areas[i].y2 + 1;

            copy_bytes_per_line = (x_end - x_start) * 2;
            from = fb_from + (y_start * h_res + x_start) * 2;
            to = fb_to + (y_start * h_res + x_start) * 2;
            for (int y = y_start; y < y_end; y++) {
                memcpy(to, from, copy_bytes_per_line);
                from += bytes_per_line;
                to += bytes_per_line;
            }
            bytes_to_flush = (y_end - y_start) * bytes_per_line;
            flush_ptr = fb_to + y_start * bytes_per_line;

            Cache_WriteBack_Addr((uint32_t)(flush_ptr), bytes_to_flush);
        }
    }
}
#endif

static void lvgl_port_flush_callback(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    const int offsetx1 = area->x1;
    const int offsetx2 = area->x2;
    const int offsety1 = area->y1;
    const int offsety2 = area->y2;

#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
    /* Due to full-refresh mode, here we just swtich pointer of frame buffer rather than draw bitmap */
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    /* Waiting for the current frame buffer to complete transmission */
    ulTaskNotifyValueClear(NULL, ULONG_MAX);
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
    /* Action after last area refresh */
    if (lv_disp_flush_is_last(drv)) {
        /* Due to direct-mode, here we just swtich driver's pointer of frame buffer rather than draw bitmap */
        esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
        /* Waiting for the current frame buffer to complete transmission */
        ulTaskNotifyValueClear(NULL, ULONG_MAX);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* Copy dirty area from the current LVGL's frame buffer to the next LVGL's frame buffer */
        lv_port_direct_mode_copy();
    }
#else
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
#endif

#if CONFIG_BSP_LCD_SUB_BOARD_800_480 || CONFIG_BSP_LCD_SUB_BOARD_480_480
    lv_disp_flush_ready(drv);
#endif
}

static void lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    case LV_DISP_ROT_90:
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_180:
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_270:
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
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
        } else if (task_delay_ms < 5) {
            task_delay_ms = 5;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

static lv_disp_t *lvgl_port_display_init(void)
{
    void *lcd_usr_data = NULL;
#if CONFIG_BSP_LCD_SUB_BOARD_480_480
    /* Sub board 2 with 480x480 uses io expander to configure LCD */
    BSP_NULL_CHECK(bsp_io_expander_init(), NULL);
    lcd_usr_data = io_expander;
#endif
    esp_lcd_panel_handle_t panel_handle = bsp_lcd_init(lcd_usr_data);

    // alloc draw buffers used by LVGL
    void *buf1 = NULL;
    void *buf2 = NULL;
    int buffer_size;
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
    buffer_size = BSP_LCD_H_RES * BSP_LCD_V_RES;
    esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2);
#elif CONFIG_BSP_LCD_SUB_BOARD_480_480 || CONFIG_BSP_LCD_SUB_BOARD_800_480
    buffer_size = BSP_LCD_H_RES * LVGL_BUFFER_HEIGHT;
    buf1 = heap_caps_malloc(buffer_size * sizeof(lv_color_t), LVGL_BUFFER_MALLOC);
    BSP_NULL_CHECK(buf1, NULL);
#endif
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, buffer_size);

    ESP_LOGD(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = BSP_LCD_H_RES;
    disp_drv.ver_res = BSP_LCD_V_RES;
    disp_drv.flush_cb = lvgl_port_flush_callback;
    disp_drv.drv_update_cb = lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
    disp_drv.full_refresh = 1;
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
    disp_drv.direct_mode = 1;
#endif
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

static void bsp_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)indev_drv->user_data;
    assert(tp);

    uint16_t touchpad_x;
    uint16_t touchpad_y;
    uint8_t touchpad_cnt = 0;
    /* Read data from touch controller into memory */
    esp_lcd_touch_read_data(tp);

    /* Read data from touch controller */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(tp, &touchpad_x, &touchpad_y, NULL, &touchpad_cnt, 1);
    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x;
        data->point.y = touchpad_y;
        data->state = LV_INDEV_STATE_PRESSED;
        ESP_LOGD(TAG, "Touch position: %d,%d", touchpad_x, touchpad_y);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static esp_err_t lvgl_port_indev_init(void)
{
    static lv_indev_drv_t indev_drv_tp;
    lv_indev_t *indev_touchpad;

    /* Initialize touch panel in sub board */
    tp = bsp_touch_panel_init();
    BSP_NULL_CHECK(tp, ESP_FAIL);

    /* Register a touchpad input device */
    lv_indev_drv_init(&indev_drv_tp);
    indev_drv_tp.type = LV_INDEV_TYPE_POINTER;
    indev_drv_tp.read_cb = bsp_touchpad_read;
    indev_drv_tp.user_data = tp;
    indev_touchpad = lv_indev_drv_register(&indev_drv_tp);
    BSP_NULL_CHECK(indev_touchpad, ESP_ERR_NO_MEM);

    return ESP_OK;
}

#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
static bool lvgl_port_task_notify(esp_lcd_panel_handle_t handle)
{
    BaseType_t need_yield = pdFALSE;
    xTaskNotifyFromISR(lvgl_task_handle, ULONG_MAX, eNoAction, &need_yield);
    return (need_yield == pdTRUE);
}
#endif

lv_disp_t *bsp_display_start(void)
{
    lv_init();
    lv_disp_t *disp = lvgl_port_display_init();
    BSP_NULL_CHECK(disp, NULL);
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_tick_init());
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_indev_init());

    lvgl_mux = xSemaphoreCreateMutex();
    BSP_NULL_CHECK(lvgl_mux, NULL);
    xTaskCreate(lvgl_port_task, "LVGL task", 4096, NULL, CONFIG_BSP_DISPLAY_LVGL_TASK_PRIORITY, &lvgl_task_handle);
#if CONFIG_BSP_DISPLAY_LVGL_AVOID_TEAR
    bsp_lcd_register_trans_done_callback(lvgl_port_task_notify);
#endif

    return disp;
}

esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    ESP_LOGW(TAG, "This board doesn't support to change brightness of LCD");
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

/**************************************************************************************************
 *
 * Button Funciton
 *
 **************************************************************************************************/
esp_err_t bsp_button_init(const bsp_button_t btn)
{
    const gpio_config_t button_io_config = {
        .pin_bit_mask = BIT64(btn),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&button_io_config);
}

bool bsp_button_get(const bsp_button_t btn)
{
    return !(bool)gpio_get_level(btn);
}
