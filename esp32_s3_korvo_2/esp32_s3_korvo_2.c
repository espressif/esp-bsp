/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

//#include "esp_adc/adc_oneshot.h"
//#include "esp_adc/adc_cali.h"
//#include "esp_adc/adc_cali_scheme.h"

#include "bsp/esp32_s3_korvo_2.h"
#include "esp_io_expander_tca9554.h"
#include "esp_lcd_touch_tt21100.h"
#include "esp_vfs_fat.h"
#include "lvgl.h"
#include "bsp_err_check.h"

/* Display is not tested yet, because HW is not in stock. */
#define BSP_DISPLAY_ENABLED 0
/* Battery voltage measurement is disabled, because it waits for update button component. */
#define BSP_BATTERY_ENABLED 0

static const char *TAG = "S3-KORVO-2";

#if BSP_DISPLAY_ENABLED
static lv_disp_draw_buf_t disp_buf; // Contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;      // Contains callback functions
static esp_lcd_touch_handle_t tp;   // LCD touch handle
static SemaphoreHandle_t lvgl_mux;  // LVGL mutex
#endif
static esp_io_expander_handle_t io_expander = NULL; // IO expander tca9554 handle
//static adc_oneshot_unit_handle_t adc1_handle; // ADC1 handle; for USB voltage measurement
//static adc_cali_handle_t adc1_cali_handle; // ADC1 calibration handle
sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler

const button_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_4, // ADC1 channel 4 is GPIO5
        .adc_button_config.button_index = BSP_BUTTON_REC,
        .adc_button_config.min = 2310, // middle is 2410mV
        .adc_button_config.max = 2510
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_4, // ADC1 channel 4 is GPIO5
        .adc_button_config.button_index = BSP_BUTTON_MUTE,
        .adc_button_config.min = 1880, // middle is 1980mV
        .adc_button_config.max = 2080
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_4, // ADC1 channel 4 is GPIO5
        .adc_button_config.button_index = BSP_BUTTON_PLAY,
        .adc_button_config.min = 1550, // middle is 1650mV
        .adc_button_config.max = 1750
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_4, // ADC1 channel 4 is GPIO5
        .adc_button_config.button_index = BSP_BUTTON_SET,
        .adc_button_config.min = 1010, // middle is 1110mV
        .adc_button_config.max = 1210
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_4, // ADC1 channel 4 is GPIO5
        .adc_button_config.button_index = BSP_BUTTON_VOLDOWN,
        .adc_button_config.min = 720, // middle is 820mV
        .adc_button_config.max = 920
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_4, // ADC1 channel 4 is GPIO5
        .adc_button_config.button_index = BSP_BUTTON_VOLUP,
        .adc_button_config.min = 280, // middle is 380mV
        .adc_button_config.max = 480
    }
};

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


esp_err_t bsp_sdcard_mount(void)
{
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    const sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    const sdmmc_slot_config_t slot_config = {
        .clk = BSP_SD_CLK,
        .cmd = BSP_SD_CMD,
        .d0 = BSP_SD_D0,
        .d1 = GPIO_NUM_NC,
        .d2 = GPIO_NUM_NC,
        .d3 = GPIO_NUM_NC,
        .d4 = GPIO_NUM_NC,
        .d5 = GPIO_NUM_NC,
        .d6 = GPIO_NUM_NC,
        .d7 = GPIO_NUM_NC,
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP,
        .width = 1,
        .flags = 0,
    };

    return esp_vfs_fat_sdmmc_mount(BSP_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(BSP_MOUNT_POINT, bsp_sdcard);
}

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config, i2s_chan_handle_t *tx_channel, i2s_chan_handle_t *rx_channel)
{
    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(CONFIG_BSP_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    BSP_ERROR_CHECK_RETURN_ERR(i2s_new_channel(&chan_cfg, tx_channel, rx_channel));

    /* Setup I2S channels */
    const i2s_std_config_t std_cfg_default = BSP_I2S_DUPLEX_STEREO_CFG(22050);
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
    const gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(BSP_POWER_AMP_IO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
    };
    BSP_ERROR_CHECK_RETURN_ERR(gpio_config(&io_conf));

    return ESP_OK;
}

esp_err_t bsp_audio_poweramp_enable(bool enable)
{
    BSP_ERROR_CHECK_RETURN_ERR(gpio_set_level(BSP_POWER_AMP_IO, enable ? 1 : 0));

    return ESP_OK;
}

esp_io_expander_handle_t bsp_io_expander_init(void)
{
    if (io_expander) {
        ESP_LOGW(TAG, "io_expander is initialized");
    } else {
        BSP_ERROR_CHECK_RETURN_NULL(esp_io_expander_new_i2c_tca9554(BSP_I2C_NUM, BSP_IO_EXPANDER_I2C_ADDRESS, &io_expander));
    }

    return io_expander;
}


#if BSP_DISPLAY_ENABLED

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8
#define LVGL_TICK_PERIOD_MS    CONFIG_BSP_DISPLAY_LVGL_TICK
#define LCD_LEDC_CH            CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

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
        esp_lcd_panel_mirror(panel_handle, true, true);
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, true);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        // Rotate LCD touch
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, true);
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
    BSP_NULL_CHECK(bsp_io_expander_init(), NULL);
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(io_expander, BSP_LCD_IO_CS, IO_EXPANDER_OUTPUT));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(io_expander, BSP_LCD_IO_RST, IO_EXPANDER_OUTPUT));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(io_expander, BSP_LCD_IO_BACKLIGHT, IO_EXPANDER_OUTPUT));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, BSP_LCD_IO_CS, 0));

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_PCLK,
        .mosi_io_num = BSP_LCD_DATA0,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = BSP_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    BSP_ERROR_CHECK_RETURN_NULL(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGD(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_CS,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = lvgl_port_flush_ready,
        .user_ctx = &disp_drv,
    };
    // Attach the LCD to the SPI bus
    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, &io_handle));

    ESP_LOGD(TAG, "Install LCD driver of st7789");
    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST, // Shared with Touch reset
        .color_space = ESP_LCD_COLOR_SPACE_BGR,
        .bits_per_pixel = 16,
    };
    BSP_ERROR_CHECK_RETURN_NULL(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    // Reset LCD
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, BSP_LCD_IO_RST, 1));
    vTaskDelay(pdMS_TO_TICKS(10));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, BSP_LCD_IO_RST, 0));
    vTaskDelay(pdMS_TO_TICKS(10));

    // Enable display
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, BSP_LCD_IO_CS, 1));

    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_mirror(panel_handle, true, true);
    esp_lcd_panel_disp_on_off(panel_handle, true);

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
        .int_gpio_num = BSP_LCD_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 1,
            .mirror_y = 0,
        },
    };
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_TT21100_CONFIG();
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)BSP_I2C_NUM, &tp_io_config, &tp_io_handle));
    BSP_ERROR_CHECK_RETURN_ERR(esp_lcd_touch_new_i2c_tt21100(tp_io_handle, &tp_cfg, &tp));
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
#endif //BSP_DISPLAY_ENABLED

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

lv_disp_t *bsp_display_start(void)
{
#if BSP_DISPLAY_ENABLED
    lv_init();
    lv_disp_t *disp = lvgl_port_display_init();
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_indev_init());
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_tick_init());
    lvgl_mux = xSemaphoreCreateMutex();
    BSP_NULL_CHECK(lvgl_mux, NULL);
    xTaskCreate(lvgl_port_task, "LVGL task", 4096, NULL, CONFIG_BSP_DISPLAY_LVGL_TASK_PRIORITY, NULL);
    return disp;
#else
    ESP_LOGW(TAG, "Display is disabled, because it is not tested yet!");
    return NULL;
#endif
}

void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation)
{
#if BSP_DISPLAY_ENABLED
    lv_disp_set_rotation(disp, rotation);
#else
    ESP_LOGW(TAG, "Display is disabled, because it is not tested yet!");
#endif
}

bool bsp_display_lock(uint32_t timeout_ms)
{
#if BSP_DISPLAY_ENABLED
    assert(lvgl_mux && "bsp_display_start must be called first");

    const TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
#else
    return pdFALSE;
#endif
}

void bsp_display_unlock(void)
{
#if BSP_DISPLAY_ENABLED
    assert(lvgl_mux && "bsp_display_start must be called first");
    xSemaphoreGive(lvgl_mux);
#endif
}

esp_err_t bsp_voltage_init(void)
{
#if BSP_BATTERY_ENABLED
    // Init ADC1
    const adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    BSP_ERROR_CHECK_RETURN_ERR(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // Init ADC1 channels
    const adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    BSP_ERROR_CHECK_RETURN_ERR(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config));

    // ESP32-S3 supports Curve Fitting calibration scheme
    const adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_11,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    BSP_ERROR_CHECK_RETURN_ERR(adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle));
    return ESP_OK;
#else
    ESP_LOGW(TAG, "Battery voltage measurement is disabled, because it waits for update button component.");
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

int bsp_voltage_battery_get(void)
{
#if BSP_BATTERY_ENABLED
    int voltage, adc_raw;

    assert(adc1_handle);
    BSP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &adc_raw), -1);
    BSP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_handle, adc_raw, &voltage), -1);
    return voltage * BSP_BATTERY_VOLTAGE_DIV;
#else
    ESP_LOGW(TAG, "Battery voltage measurement is disabled, because it waits for update button component.");
    return -1;
#endif
}

esp_err_t bsp_leds_init(void)
{
    BSP_NULL_CHECK(bsp_io_expander_init(), NULL);
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(io_expander, BSP_LED_RED, IO_EXPANDER_OUTPUT));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_dir(io_expander, BSP_LED_BLUE, IO_EXPANDER_OUTPUT));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, BSP_LED_RED, true));
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, BSP_LED_BLUE, true));
    return ESP_OK;
}

esp_err_t bsp_led_set(const bsp_led_t led_io, const bool on)
{
    BSP_ERROR_CHECK_RETURN_ERR(esp_io_expander_set_level(io_expander, led_io, !on));
    return ESP_OK;
}
