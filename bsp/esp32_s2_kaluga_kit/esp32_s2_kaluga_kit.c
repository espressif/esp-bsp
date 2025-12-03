/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_spiffs.h"

#include "bsp/esp32_s2_kaluga_kit.h"
#include "bsp/display.h"
#include "esp_lvgl_port.h"
#include "esp_codec_dev_defaults.h"
#include "bsp_err_check.h"
#include "button_adc.h"

static const char *TAG = "Kaluga";

/**
 * @brief I2C handle for BSP usage
 *
 * In IDF v5.4 you can call i2c_master_get_bus_handle(BSP_I2C_NUM, i2c_master_bus_handle_t *ret_handle)
 * from #include "esp_private/i2c_platform.h" to get this handle
 *
 * For IDF 5.2 and 5.3 you must call bsp_i2c_get_handle()
 */
static i2c_master_bus_handle_t i2c_handle = NULL;
static bool i2c_initialized = false;
static adc_oneshot_unit_handle_t bsp_adc_handle = NULL;

static const touch_pad_t bsp_touch_button[TOUCH_BUTTON_NUM] = {
    TOUCH_BUTTON_PHOTO,      /*!< 'PHOTO' button */
    TOUCH_BUTTON_PLAY,       /*!< 'PLAY/PAUSE' button */
    TOUCH_BUTTON_NETWORK,    /*!< 'NETWORK' button */
    TOUCH_BUTTON_RECORD,     /*!< 'RECORD' button */
    TOUCH_BUTTON_VOLUP,      /*!< 'VOL_UP' button */
    TOUCH_BUTTON_VOLDOWN,    /*!< 'VOL_DOWN' button */
    TOUCH_BUTTON_GUARD,      /*!< Guard ring for waterproof design. If this pad is touched, other pads no response.*/
};

static const button_adc_config_t bsp_button_config[BSP_BUTTON_NUM] = {
    {
        .adc_handle = &bsp_adc_handle,
        .adc_channel = ADC_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .button_index = BSP_BUTTON_REC,
        .min = 2310, // middle is 2410mV
        .max = 2510
    },
    {
        .adc_handle = &bsp_adc_handle,
        .adc_channel = ADC_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .button_index = BSP_BUTTON_MODE,
        .min = 1880, // middle is 1980mV
        .max = 2080
    },
    {
        .adc_handle = &bsp_adc_handle,
        .adc_channel = ADC_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .button_index = BSP_BUTTON_PLAY,
        .min = 1550, // middle is 1650mV
        .max = 1750
    },
    {
        .adc_handle = &bsp_adc_handle,
        .adc_channel = ADC_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .button_index = BSP_BUTTON_SET,
        .min = 1010, // middle is 1110mV
        .max = 1210
    },
    {
        .adc_handle = &bsp_adc_handle,
        .adc_channel = ADC_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .button_index = BSP_BUTTON_VOLDOWN,
        .min = 720, // middle is 820mV
        .max = 920
    },
    {
        .adc_handle = &bsp_adc_handle,
        .adc_channel = ADC_CHANNEL_5, // ADC1 channel 5 is GPIO6
        .button_index = BSP_BUTTON_VOLUP,
        .min = 280, // middle is 380mV
        .max = 480
    }
};

esp_err_t bsp_iot_button_create(button_handle_t btn_array[], int *btn_cnt, int btn_array_size)
{
    esp_err_t ret = ESP_OK;
    const button_config_t btn_config = {0};
    if ((btn_array_size < BSP_BUTTON_NUM) ||
            (btn_array == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Initialize ADC and get ADC handle */
    BSP_ERROR_CHECK_RETURN_NULL(bsp_adc_initialize());
    bsp_adc_handle = bsp_adc_get_handle();

    if (btn_cnt) {
        *btn_cnt = 0;
    }
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        ret |= iot_button_new_adc_device(&btn_config, &bsp_button_config[i], &btn_array[i]);
        if (btn_cnt) {
            (*btn_cnt)++;
        }
    }
    return ret;
}

esp_err_t bsp_i2c_init(void)
{
    /* I2C was initialized before */
    if (i2c_initialized) {
        return ESP_OK;
    }

    const i2c_master_bus_config_t i2c_config = {
        .i2c_port = BSP_I2C_NUM,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    BSP_ERROR_CHECK_RETURN_ERR(i2c_new_master_bus(&i2c_config, &i2c_handle));

    i2c_initialized = true;
    return ESP_OK;
}

esp_err_t bsp_i2c_deinit(void)
{
    BSP_ERROR_CHECK_RETURN_ERR(i2c_del_master_bus(i2c_handle));
    i2c_initialized = false;
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_get_handle(void)
{
    bsp_i2c_init();
    return i2c_handle;
}

/**
 * @brief Common codec init
 *
 * Kaluga kit uses one codec for both audio playback and recording
 *
 * @return esp_codec_dev_t
 */
static esp_codec_dev_handle_t bsp_audio_codec_init(void)
{
    static esp_codec_dev_handle_t codec = NULL;
    static const audio_codec_data_if_t *i2s_data_if = NULL;  /* Codec data interface */

    // This function can be called only once
    if (NULL != codec) {
        return codec;
    }

    /* Initialize I2S: IDF-version dependant implementation */
    BSP_ERROR_CHECK_RETURN_NULL(bsp_audio_init(NULL));
    i2s_data_if = bsp_audio_get_codec_itf();
    BSP_NULL_CHECK(i2s_data_if, NULL);

    /* Initialize I2C */
    BSP_ERROR_CHECK_RETURN_ERR(bsp_i2c_init());
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = BSP_I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_handle,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    BSP_NULL_CHECK(i2c_ctrl_if, NULL);

    /* Create new ES8311 codec */
    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage = 5.0,
        .codec_dac_voltage = 3.3,
    };

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = i2c_ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = BSP_POWER_AMP_IO,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain = gain,
    };
    const audio_codec_if_t *es8311_dev = es8311_codec_new(&es8311_cfg);
    BSP_NULL_CHECK(es8311_dev, NULL);

    esp_codec_dev_cfg_t codec_dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = es8311_dev,
        .data_if = i2s_data_if,
    };
    codec = esp_codec_dev_new(&codec_dev_cfg);
    BSP_NULL_CHECK(codec, NULL);
    return codec;
}

esp_codec_dev_handle_t bsp_audio_codec_speaker_init(void)
{
    return bsp_audio_codec_init();
}

esp_codec_dev_handle_t bsp_audio_codec_microphone_init(void)
{
    return bsp_audio_codec_init();
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
        .rgb_ele_order = BSP_LCD_COLOR_SPACE,
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

static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
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
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
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
            .buff_dma = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
        }
    };

    return lvgl_port_add_disp(&disp_cfg);
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
        }
    };
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    lv_display_t *disp = NULL;
    BSP_NULL_CHECK(cfg, NULL);

    BSP_ERROR_CHECK_RETURN_NULL(lvgl_port_init(&cfg->lvgl_port_cfg));
    BSP_NULL_CHECK(disp = bsp_display_lcd_init(cfg), NULL);
    return disp;
}

lv_indev_t *bsp_display_get_input_dev(void)
{
    return NULL;
}

void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation)
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
esp_err_t bsp_display_brightness_init(void)
{
    return ESP_OK;
}
esp_err_t bsp_display_backlight_off(void)
{
    return ESP_OK;
}
esp_err_t bsp_display_backlight_on(void)
{
    return ESP_OK;
}
esp_err_t bsp_display_brightness_set(int brightness_percent)
{
    return ESP_OK;
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    BSP_ERROR_CHECK_RETURN_ERR(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}
