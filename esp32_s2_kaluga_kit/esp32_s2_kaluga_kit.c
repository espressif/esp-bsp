/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_timer.h"
#include "bsp/esp32_s2_kaluga_kit.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

static const char *TAG = "Kaluga";

static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;      // contains callback functions
static SemaphoreHandle_t lvgl_mux;  // LVGL mutex

static const touch_pad_t bsp_touch_button[TOUCH_BUTTON_NUM] = {
    TOUCH_BUTTON_PHOTO,      /*!< 'PHOTO' button */
    TOUCH_BUTTON_PLAY,       /*!< 'PLAY/PAUSE' button */
    TOUCH_BUTTON_NETWORK,    /*!< 'NETWORK' button */
    TOUCH_BUTTON_RECORD,     /*!< 'RECORD' button */
    TOUCH_BUTTON_VOLUP,      /*!< 'VOL_UP' button */
    TOUCH_BUTTON_VOLDOWN,    /*!< 'VOL_DOWN' button */
    TOUCH_BUTTON_GUARD,      /*!< Guard ring for waterproof design. If this pad is touched, other pads no response.*/
};

const button_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC1_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .adc_button_config.button_index = BSP_BUTTON_REC,
        .adc_button_config.min = 2310, // middle is 2410mV
        .adc_button_config.max = 2510
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC1_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .adc_button_config.button_index = BSP_BUTTON_MODE,
        .adc_button_config.min = 1880, // middle is 1980mV
        .adc_button_config.max = 2080
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC1_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .adc_button_config.button_index = BSP_BUTTON_PLAY,
        .adc_button_config.min = 1550, // middle is 1650mV
        .adc_button_config.max = 1750
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC1_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .adc_button_config.button_index = BSP_BUTTON_SET,
        .adc_button_config.min = 1010, // middle is 1110mV
        .adc_button_config.max = 1210
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC1_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .adc_button_config.button_index = BSP_BUTTON_VOLDOWN,
        .adc_button_config.min = 720, // middle is 820mV
        .adc_button_config.max = 920
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC1_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .adc_button_config.button_index = BSP_BUTTON_VOLUP,
        .adc_button_config.min = 280, // middle is 380mV
        .adc_button_config.max = 480
    }
};

void bsp_i2c_init(void)
{
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = BSP_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CONFIG_BSP_I2C_CLK_SPEED_HZ
    };
    ESP_ERROR_CHECK(i2c_param_config(BSP_I2C_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(BSP_I2C_NUM, i2c_conf.mode, 0, 0, 0));
}

void bsp_i2c_deinit(void)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_driver_delete(BSP_I2C_NUM));
}

void bsp_audio_init(const i2s_std_config_t *i2s_config, i2s_chan_handle_t *tx_channel, i2s_chan_handle_t *rx_channel)
{
    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, tx_channel, rx_channel));

    /* Setup I2S channels */
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_MONO_CFG(22050);
    const i2s_std_config_t *p_i2s_cfg = &std_cfg_default;
    if (i2s_config != NULL) {
        p_i2s_cfg = i2s_config;
    }

    if (tx_channel != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(*tx_channel, p_i2s_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(*tx_channel));
    }
    if (rx_channel != NULL) {
        ESP_ERROR_CHECK(i2s_channel_init_std_mode(*rx_channel, p_i2s_cfg));
        ESP_ERROR_CHECK(i2s_channel_enable(*rx_channel));
    }

    /* Setup power amplifier enable pin */
    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(BSP_POWER_AMP_IO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

void bsp_audio_poweramp_enable(bool enable)
{
    ESP_ERROR_CHECK(gpio_set_level(BSP_POWER_AMP_IO, enable ? 1 : 0));
}

void bsp_touchpad_init(intr_handler_t fn)
{
    /*!< Initialize touch pad peripheral, it will start a timer to run a filter */
    ESP_ERROR_CHECK(touch_pad_init());

    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        ESP_ERROR_CHECK(touch_pad_config(bsp_touch_button[i]));
    }

    /*!< Denoise setting at TouchSensor 0. */
    touch_pad_denoise_t denoise = {
        /*!< The bits to be cancelled are determined according to the noise level. */
        .grade = TOUCH_PAD_DENOISE_BIT4,
        .cap_level = TOUCH_PAD_DENOISE_CAP_L4,
    };
    ESP_ERROR_CHECK(touch_pad_denoise_set_config(&denoise));
    ESP_ERROR_CHECK(touch_pad_denoise_enable());

    /*!< Waterproof function */
    touch_pad_waterproof_t waterproof = {
        .guard_ring_pad = TOUCH_BUTTON_GUARD,   /*!< If no ring pad, set 0; */
        /*!< It depends on the number of the parasitic capacitance of the shield pad. */
        .shield_driver = TOUCH_PAD_SHIELD_DRV_L0,   /*!< 40pf */
    };
    ESP_ERROR_CHECK(touch_pad_waterproof_set_config(&waterproof));
    ESP_ERROR_CHECK(touch_pad_waterproof_enable());

    /*!< Filter setting */
    touch_filter_config_t filter_info = {
        .mode = TOUCH_PAD_FILTER_IIR_8,           /*!< Test jitter and filter 1/4. */
        .debounce_cnt = 1,      /*!< 1 time count. */
        .noise_thr = 0,         /*!< 50% */
        .jitter_step = 4,       /*!< use for jitter mode. */
    };
    ESP_ERROR_CHECK(touch_pad_filter_set_config(&filter_info));
    ESP_ERROR_CHECK(touch_pad_filter_enable());
    /*!< Register touch interrupt ISR, enable intr type. */
    ESP_ERROR_CHECK(touch_pad_isr_register(fn, NULL, TOUCH_PAD_INTR_MASK_ALL));
    ESP_ERROR_CHECK(touch_pad_intr_enable(TOUCH_PAD_INTR_MASK_ACTIVE | TOUCH_PAD_INTR_MASK_INACTIVE));

    /*!< Enable touch sensor clock. Work mode is "timer trigger". */
    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    ESP_ERROR_CHECK(touch_pad_fsm_start());

    /*!< Wait touch sensor init done and calibrate */
    vTaskDelay(100 / portTICK_PERIOD_MS);
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        bsp_touchpad_calibrate(bsp_touch_button[i], 0.1f);
    }
}

void bsp_touchpad_calibrate(bsp_touchpad_button_t tch_pad, float tch_threshold)
{
    /*!< read baseline value */
    uint32_t touch_value = 0;
    ESP_ERROR_CHECK(touch_pad_read_benchmark(tch_pad, &touch_value));
    /*!< set interrupt threshold. */
    ESP_ERROR_CHECK(touch_pad_set_thresh(tch_pad, (uint32_t)((float)touch_value * tch_threshold)));
}

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           (8)
#define LCD_PARAM_BITS         (8)
#define LVGL_TICK_PERIOD_MS    (CONFIG_BSP_DISPLAY_LVGL_TICK)
#define LVGL_BUFF_SIZE_PIX     (BSP_LCD_H_RES * BSP_LCD_V_RES)

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
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
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
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    return esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000);
}

static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        bsp_display_lock(0);
        lv_timer_handler();
        bsp_display_unlock();
    }
}

static lv_disp_t *lvgl_port_display_init(void)
{
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num     = BSP_LCD_SPI_CLK,
        .mosi_io_num     = BSP_LCD_SPI_MOSI,
        .miso_io_num     = GPIO_NUM_NC,
        .quadwp_io_num   = GPIO_NUM_NC,
        .quadhd_io_num   = GPIO_NUM_NC,
        .max_transfer_sz = LVGL_BUFF_SIZE_PIX * sizeof(lv_color_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGD(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_SPI_CS,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = lvgl_port_flush_ready,
        .user_ctx = &disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, &io_handle));

    ESP_LOGD(TAG, "Install LCD driver for ST7789");
    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_mirror(panel_handle, true, false);
    esp_lcd_panel_swap_xy(panel_handle, true);
    esp_lcd_panel_invert_color(panel_handle, false);
    esp_lcd_panel_disp_on_off(panel_handle, true);

    // Alloc draw buffers used by LVGL
    // It's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(LVGL_BUFF_SIZE_PIX * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_disp_draw_buf_init(&disp_buf, buf1, NULL, LVGL_BUFF_SIZE_PIX);

    ESP_LOGI(TAG, "Registering display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = BSP_LCD_H_RES;
    disp_drv.ver_res = BSP_LCD_V_RES;
    disp_drv.flush_cb = lvgl_port_flush_callback;
    disp_drv.drv_update_cb = lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    return lv_disp_drv_register(&disp_drv);
}

lv_disp_t *bsp_display_start(void)
{
    lv_init();
    lv_disp_t *disp = lvgl_port_display_init();
    lvgl_port_tick_init();
    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);
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


/* Backlight functions are not implemented - Kaluga board doesn't provide backlight control
   These functions are here to provide consistent API with other Board Support Packages */
void bsp_display_backlight_off(void) {}
void bsp_display_backlight_on(void) {}
