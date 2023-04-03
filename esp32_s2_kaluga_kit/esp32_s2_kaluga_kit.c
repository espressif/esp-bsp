/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "bsp/esp32_s2_kaluga_kit.h"
#include "bsp/display.h"
#include "esp_lvgl_port.h"
#include "bsp_err_check.h"

static const char *TAG = "Kaluga";

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
        .adc_button_config.adc_channel = ADC_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .adc_button_config.button_index = BSP_BUTTON_REC,
        .adc_button_config.min = 2310, // middle is 2410mV
        .adc_button_config.max = 2510
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .adc_button_config.button_index = BSP_BUTTON_MODE,
        .adc_button_config.min = 1880, // middle is 1980mV
        .adc_button_config.max = 2080
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .adc_button_config.button_index = BSP_BUTTON_PLAY,
        .adc_button_config.min = 1550, // middle is 1650mV
        .adc_button_config.max = 1750
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .adc_button_config.button_index = BSP_BUTTON_SET,
        .adc_button_config.min = 1010, // middle is 1110mV
        .adc_button_config.max = 1210
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .adc_button_config.button_index = BSP_BUTTON_VOLDOWN,
        .adc_button_config.min = 720, // middle is 820mV
        .adc_button_config.max = 920
    },
    {
        .type = BUTTON_TYPE_ADC,
        .adc_button_config.adc_channel = ADC_CHANNEL_5, // ADC1 channel 5 is GPIO6
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
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = BSP_I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
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

esp_err_t bsp_audio_init(const i2s_std_config_t *i2s_config, i2s_chan_handle_t *tx_channel, i2s_chan_handle_t *rx_channel)
{
    /* Setup I2S peripheral */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
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

    /* Setup power amplifier enable pin */
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

esp_err_t bsp_touchpad_init(intr_handler_t fn)
{
    /*!< Initialize touch pad peripheral, it will start a timer to run a filter */
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_init());

    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        BSP_ERROR_CHECK_RETURN_ERR(touch_pad_config(bsp_touch_button[i]));
    }

    /*!< Denoise setting at TouchSensor 0. */
    touch_pad_denoise_t denoise = {
        /*!< The bits to be cancelled are determined according to the noise level. */
        .grade = TOUCH_PAD_DENOISE_BIT4,
        .cap_level = TOUCH_PAD_DENOISE_CAP_L4,
    };
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_denoise_set_config(&denoise));
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_denoise_enable());

    /*!< Waterproof function */
    touch_pad_waterproof_t waterproof = {
        .guard_ring_pad = TOUCH_BUTTON_GUARD,   /*!< If no ring pad, set 0; */
        /*!< It depends on the number of the parasitic capacitance of the shield pad. */
        .shield_driver = TOUCH_PAD_SHIELD_DRV_L0,   /*!< 40pf */
    };
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_waterproof_set_config(&waterproof));
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_waterproof_enable());

    /*!< Filter setting */
    touch_filter_config_t filter_info = {
        .mode = TOUCH_PAD_FILTER_IIR_8,           /*!< Test jitter and filter 1/4. */
        .debounce_cnt = 1,      /*!< 1 time count. */
        .noise_thr = 0,         /*!< 50% */
        .jitter_step = 4,       /*!< use for jitter mode. */
    };
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_filter_set_config(&filter_info));
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_filter_enable());
    /*!< Register touch interrupt ISR, enable intr type. */
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_isr_register(fn, NULL, TOUCH_PAD_INTR_MASK_ALL));
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_intr_enable(TOUCH_PAD_INTR_MASK_ACTIVE | TOUCH_PAD_INTR_MASK_INACTIVE));

    /*!< Enable touch sensor clock. Work mode is "timer trigger". */
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_fsm_start());

    /*!< Wait touch sensor init done and calibrate */
    vTaskDelay(100 / portTICK_PERIOD_MS);
    for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        bsp_touchpad_calibrate(bsp_touch_button[i], 0.1f);
    }

    return ESP_OK;
}

esp_err_t bsp_touchpad_calibrate(bsp_touchpad_button_t tch_pad, float tch_threshold)
{
    /*!< read baseline value */
    uint32_t touch_value = 0;
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_read_benchmark(tch_pad, &touch_value));
    /*!< set interrupt threshold. */
    BSP_ERROR_CHECK_RETURN_ERR(touch_pad_set_thresh(tch_pad, (uint32_t)((float)touch_value * tch_threshold)));

    return ESP_OK;
}

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           (8)
#define LCD_PARAM_BITS         (8)

esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io)
{
    esp_err_t ret = ESP_OK;
    assert(config != NULL && config->max_transfer_sz > 0);

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num     = BSP_LCD_SPI_CLK,
        .mosi_io_num     = BSP_LCD_SPI_MOSI,
        .miso_io_num     = GPIO_NUM_NC,
        .quadwp_io_num   = GPIO_NUM_NC,
        .quadhd_io_num   = GPIO_NUM_NC,
        .max_transfer_sz = config->max_transfer_sz,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BSP_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC,
        .cs_gpio_num = BSP_LCD_SPI_CS,
        .pclk_hz = BSP_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BSP_LCD_SPI_NUM, &io_config, ret_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST,
        .color_space = BSP_LCD_COLOR_SPACE,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(*ret_io, &panel_config, ret_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(*ret_panel);
    esp_lcd_panel_init(*ret_panel);
    esp_lcd_panel_mirror(*ret_panel, true, false);
    esp_lcd_panel_swap_xy(*ret_panel, true);
    esp_lcd_panel_invert_color(*ret_panel, false);
    return ret;

err:
    if (*ret_panel) {
        esp_lcd_panel_del(*ret_panel);
    }
    if (*ret_io) {
        esp_lcd_panel_io_del(*ret_io);
    }
    spi_bus_free(BSP_LCD_SPI_NUM);
    return ret;
}

static lv_disp_t *bsp_display_lcd_init(void)
{
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle = NULL;
    const bsp_display_config_t bsp_disp_cfg = {
        .max_transfer_sz = BSP_LCD_DRAW_BUFF_SIZE * sizeof(uint16_t),
    };
    BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));

    esp_lcd_panel_disp_on_off(panel_handle, true);

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = true,
            .mirror_x = true,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}

lv_disp_t *bsp_display_start(void)
{
    lv_disp_t *disp = NULL;
    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&lvgl_cfg));
    BSP_NULL_CHECK(disp = bsp_display_lcd_init(), NULL);
    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return NULL;
}

void bsp_display_rotate(lv_disp_t *disp, lv_disp_rot_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}


/* Backlight functions are not implemented - Kaluga board doesn't provide backlight control
   These functions are here to provide consistent API with other Board Support Packages */
esp_err_t bsp_display_backlight_off(void)
{
    return ESP_OK;
}
esp_err_t bsp_display_backlight_on(void)
{
    return ESP_OK;
}
