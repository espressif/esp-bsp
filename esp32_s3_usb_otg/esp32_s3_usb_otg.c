/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "bsp/esp32_s3_usb_otg.h"
#include "esp_vfs_fat.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_adc_cal.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/adc.h"

#include "usb/usb_host.h"

#define TAG "USB-OTG"

static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;      // contains callback functions
static SemaphoreHandle_t lvgl_mux;  // LVGL mutex
static TaskHandle_t usb_host_task;  // USB Host Library task
static esp_adc_cal_characteristics_t adc1_chars; // ADC1 calibration data
sdmmc_card_t *bsp_sdcard = NULL;    // Global uSD card handler

void bsp_leds_init(void)
{
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BIT64(BSP_LED_YELLOW) | BIT64(BSP_LED_GREEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&led_io_config));
}

void bsp_led_set(const bsp_led_t led_io, const bool on)
{
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t) led_io, (uint32_t) on));
}

esp_err_t bsp_sdcard_mount(void)
{
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_uSD_FORMAT_ON_MOUNT_FAIL
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
        .d1 = BSP_SD_D1,
        .d2 = BSP_SD_D2,
        .d3 = BSP_SD_D3,
        .d4 = GPIO_NUM_NC,
        .d5 = GPIO_NUM_NC,
        .d6 = GPIO_NUM_NC,
        .d7 = GPIO_NUM_NC,
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP,
        .width = 4,
        .flags = 0,
    };

    return esp_vfs_fat_sdmmc_mount(CONFIG_BSP_uSD_MOUNT_POINT, &host, &slot_config, &mount_config, &bsp_sdcard);
}

esp_err_t bsp_sdcard_unmount(void)
{
    return esp_vfs_fat_sdcard_unmount(CONFIG_BSP_uSD_MOUNT_POINT, bsp_sdcard);
}

void bsp_button_init(void)
{
    const gpio_config_t btn_io_config = {
        .pin_bit_mask = BIT64(BSP_BUTTON_DW) | BIT64(BSP_BUTTON_UP) | BIT64(BSP_BUTTON_OK) | BIT64(BSP_BUTTON_MENU) | BIT64(BSP_USB_OVERCURRENT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&btn_io_config));
}

bool bsp_button_get(const bsp_button_t btn)
{
    return !(bool)gpio_get_level(btn);
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
        esp_lcd_panel_set_gap(panel_handle, 0, 0);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        esp_lcd_panel_set_gap(panel_handle, 80, 0);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        esp_lcd_panel_set_gap(panel_handle, 0, 80);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        esp_lcd_panel_set_gap(panel_handle, 0, 0);
        break;
    }
}

#define LCD_CMD_BITS         (8)
#define LCD_PARAM_BITS       (8)
#define LCD_LEDC_CH          (CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH)
#define LVGL_TICK_PERIOD_MS  (CONFIG_BSP_DISPLAY_LVGL_TICK)

static void bsp_display_brightness_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BACKLIGHT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0
    };
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&LCD_backlight_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&LCD_backlight_channel));
}

void bsp_display_brightness_set(int brightness_percent)
{
    if (brightness_percent > 100) {
        brightness_percent = 100;
    } else if (brightness_percent < 0) {
        brightness_percent = 0;
    }

    ESP_LOGI(TAG, "Setting LCD backlight: %d%%", brightness_percent);
    // LEDC resolution set to 10bits, thus: 100% = 1023
    uint32_t duty_cycle = (1023 * brightness_percent) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LCD_LEDC_CH));
}

void bsp_display_backlight_off(void)
{
    bsp_display_brightness_set(0);
}

void bsp_display_backlight_on(void)
{
    bsp_display_brightness_set(100);
}

static lv_disp_t *lvgl_port_display_init(void)
{
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_SPI_CLK,
        .mosi_io_num = BSP_LCD_SPI_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = BSP_LCD_BUFF_SIZE * sizeof(lv_color_t),
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

    ESP_LOGD(TAG, "Install LCD driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_disp_off(panel_handle, false);

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(BSP_LCD_BUFF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(BSP_LCD_BUFF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, BSP_LCD_BUFF_SIZE);

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

static void lvgl_port_tick_increment(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static esp_err_t lvgl_port_tick_init(void)
{
    // Tick interface for LVGL (using esp_timer to generate periodic event)
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
        bsp_display_lock(0);
        uint32_t task_delay_ms = lv_timer_handler();
        bsp_display_unlock();
        if (task_delay_ms > 500) {
            task_delay_ms = 500;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

lv_disp_t *bsp_display_start(void)
{
    lv_init();
    bsp_display_brightness_init();
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

void bsp_usb_mode_select_device(void)
{
    // Make sure the pin is configured
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BIT64(BSP_USB_MODE_SEL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&led_io_config));
    ESP_ERROR_CHECK(gpio_set_level(BSP_USB_MODE_SEL, 0));
}


void bsp_usb_mode_select_host(void)
{
    // Make sure the pin is configured
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BIT64(BSP_USB_MODE_SEL),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&led_io_config));
    ESP_ERROR_CHECK(gpio_set_level(BSP_USB_MODE_SEL, 1));
}

void bsp_usb_host_power_mode(bsp_usb_host_power_mode_t mode, bool limit_500mA)
{
    // Make sure the pins are configured
    const gpio_config_t led_io_config = {
        .pin_bit_mask = BIT64(BSP_USB_LIMIT_EN) | BIT64(BSP_USB_DEV_VBUS_EN) | BIT64(BSP_BATTERY_BOOST_EN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&led_io_config));

    // 1. Configure the limiter
    ESP_ERROR_CHECK(gpio_set_level(BSP_USB_LIMIT_EN, limit_500mA));

    // 2. Turn power off and wait 10ms
    ESP_ERROR_CHECK(gpio_set_level(BSP_USB_DEV_VBUS_EN, 0));
    ESP_ERROR_CHECK(gpio_set_level(BSP_BATTERY_BOOST_EN, 0));
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. Turn on requested mode
    if (mode == BSP_USB_HOST_POWER_MODE_BATTERY) {
        ESP_ERROR_CHECK(gpio_set_level(BSP_BATTERY_BOOST_EN, 1));
    } else if (mode == BSP_USB_HOST_POWER_MODE_USB_DEV) {
        ESP_ERROR_CHECK(gpio_set_level(BSP_USB_DEV_VBUS_EN, 1));
    }
}

static void usb_lib_task(void *arg)
{
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
            // The only way this task can be stopped is by calling bsp_usb_host_stop()
        }
    }
}

void bsp_usb_host_start(bsp_usb_host_power_mode_t mode, bool limit_500mA)
{
    // Configure board for host mode
    bsp_usb_mode_select_host();
    bsp_usb_host_power_mode(mode, limit_500mA);

    //Install USB Host driver. Should only be called once in entire application
    ESP_LOGI(TAG, "Installing USB Host");
    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    // Create a task that will handle USB library events
    if (xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, &usb_host_task) != pdTRUE) {
        ESP_LOGE(TAG, "Creating USB host lib task failed");
        abort();
    }
}

void bsp_usb_host_stop(void)
{
    usb_host_uninstall();
    if (usb_host_task) {
        vTaskSuspend(usb_host_task);
        vTaskDelete(usb_host_task);
    }
    bsp_usb_host_power_mode(BSP_USB_HOST_POWER_MODE_OFF, false);
}

bool bsp_voltage_init(void)
{
    // Calibration
    ESP_ERROR_CHECK(esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP_FIT));
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);

    //ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11));
    return true;
}

uint32_t bsp_voltage_battery_get(void)
{
    uint32_t voltage;
    esp_adc_cal_get_voltage(ADC1_CHANNEL_1, &adc1_chars, &voltage);
    return voltage * BSP_BATTERY_VOLTAGE_DIV;
}

uint32_t bsp_voltage_usb_get(void)
{
    uint32_t voltage;
    esp_adc_cal_get_voltage(ADC1_CHANNEL_0, &adc1_chars, &voltage);
    return (float)voltage * BSP_USB_HOST_VOLTAGE_DIV;
}
