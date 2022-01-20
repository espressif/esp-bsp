// Copyright 2015-2021 Espressif Systems (Shanghai) CO LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "esp32_s2_kaluga_kit.h"
#include "lvgl.h"
#include "lvgl_helpers.h"
#include "esp_rom_gpio.h"   // needed for MCLK output config

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
    ESP_ERROR_CHECK(i2c_driver_delete(BSP_I2C_NUM));
}

void bsp_audio_init(const i2s_config_t *i2s_config)
{
    /* Setup I2S peripheral */
    const i2s_pin_config_t i2s_pin_config = {
        .bck_io_num = BSP_I2S_SCLK,
        .ws_io_num = BSP_I2S_LCLK,
        .data_out_num = BSP_I2S_DOUT,
        .data_in_num = BSP_I2S_DSIN
    };

    ESP_ERROR_CHECK(i2s_driver_install(BSP_I2S_NUM, i2s_config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_pin(BSP_I2S_NUM, &i2s_pin_config));

    /* Setup power amplifier and I2S MCLK pin */
    const gpio_config_t io_conf = {
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(BSP_POWER_AMP_IO) | BIT64(BSP_I2S_MCLK),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    /* Note: In ESP32-S2 Technical Reference Manual in Table 22: GPIO Matrix, there is a definition of
       I2S MCLK = 251. This definition is missing in I2S driver, so it must be hardcoded here */
    esp_rom_gpio_connect_out_signal(BSP_I2S_MCLK, 251, false, false);
}

void bsp_audio_deinit(void)
{
    ESP_ERROR_CHECK_WITHOUT_ABORT(i2s_driver_uninstall(BSP_I2S_NUM));
    const gpio_config_t io_conf = {
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = BIT64(BSP_POWER_AMP_IO) | BIT64(BSP_I2S_MCLK),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_config(&io_conf));
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

#define LV_TICK_PERIOD_MS 1
static void lv_tick_task(void *arg)
{
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}

void bsp_display_task(void *pvParameter)
{
    lv_init();

    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);

    /* Use double buffered when not working with monochrome displays */
    lv_color_t *buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);

    static lv_disp_buf_t disp_buf;
    lv_disp_buf_init(&disp_buf, buf1, buf2, DISP_BUF_SIZE);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;
    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
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
