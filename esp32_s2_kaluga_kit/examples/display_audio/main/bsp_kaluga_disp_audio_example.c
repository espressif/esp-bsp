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

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "esp32_s2_kaluga_kit.h"
#include "es8311.h"
#include "driver/i2s.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "string.h"
#include <unistd.h>

#include "button.h"
#include "led_strip.h"
#include "lvgl.h"
#include "lvgl_helpers.h"

#define TAG "Kaluga"

/* Buffer for reading/writing to I2S driver. Same length as SPIFFS buffer and I2S buffer, for optimal read/write performance.
   Recording audio data path:
   I2S peripheral -> I2S buffer (DMA) -> App buffer (RAM) -> SPIFFS buffer -> External SPI Flash.
   Vice versa for playback. */
#define BUFFER_SIZE     (1024)
#define SAMPLE_RATE     (22050)
#define DEFAULT_VOLUME  (50)
/* The recording will be RECORDING_LENGTH * BUFFER_SIZE long (in bytes)
   With sampling frequency 22050 Hz and 16bit mono resolution it equals to ~3.715 seconds */
#define RECORDING_LENGTH (160)

/* Globals */
static button_handle_t audio_button[BSP_BUTTON_NUM] = {};
static QueueHandle_t audio_button_q = NULL;
static led_strip_t *rgb_led = NULL;
static lv_obj_t *recording_checkbox = NULL;
static lv_obj_t *playing_checkbox = NULL;
static lv_obj_t *volume_arc = NULL;

void btn_handler(void *arg)
{
    for (uint8_t i = 0; i < BSP_BUTTON_NUM; i++) {
        if ((button_handle_t)arg == audio_button[i]) {
            xQueueSend(audio_button_q, &i, 0);
            break;
        }
    }
}

// Very simple WAV header, ignores most fields
typedef struct __attribute__((packed))
{
    uint8_t ignore_0[22];
    uint16_t num_channels;
    uint32_t sample_rate;
    uint8_t ignore_1[6];
    uint16_t bits_per_sample;
    uint8_t ignore_2[4];
    uint32_t data_size;
    uint8_t data[];
} dumb_wav_header_t;

esp_err_t spiffs_init(void)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Initializing SPIFFS");

    const esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    /*!< Use settings defined above to initialize and mount SPIFFS filesystem. */
    /*!< Note: esp_vfs_spiffs_register is an all-in-one convenience function. */
    ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }

        return ret;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret;
}

static void audio_task(void *arg)
{
    const i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // ES8311 is a mono codec, only left is sufficient
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_buf_count = 3,
        .dma_buf_len = 1024, // 3x1024 bytes for buffering
        .use_apll = true,
        .tx_desc_auto_clear = true, /*!< I2S auto clear tx descriptor if there is underflow condition (helps in avoiding
                                       noise in case of data unavailability) */
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM,
    };

    const es8311_clock_config_t clk_cfg = {
        .mclk_from_mclk_pin = false,
        .mclk_inverted = false,
        .sclk_inverted = false,
        .sample_frequency = SAMPLE_RATE
    };

    /* Create and configure ES8311 I2C driver */
    es8311_handle_t es8311_dev = es8311_create(BSP_I2C_NUM, ES8311_ADDRRES_0);
    es8311_init(es8311_dev, &clk_cfg, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
    es8311_voice_volume_set(es8311_dev, DEFAULT_VOLUME, NULL);

    /* Microphone settings */
    es8311_microphone_config(es8311_dev, false);
    es8311_microphone_gain_set(es8311_dev, ES8311_MIC_GAIN_42DB);

    bsp_audio_init(&i2s_config);
    bsp_audio_poweramp_enable(true);

    /* Pointer to a file that is going to be played */
    const char music_filename[] = "/spiffs/16bit_mono_22_05khz.wav";
    const char recording_filename[] = "/spiffs/recording.wav";
    const char *play_filename = music_filename;

    while (1) {
        uint8_t btn_index = 0;
        if (xQueueReceive(audio_button_q, &btn_index, portMAX_DELAY) == pdTRUE) {
            switch (btn_index) {
            case BSP_BUTTON_REC: {
                /* Writing to SPIFFS from internal RAM is ~15% faster than from external SPI RAM */
                int16_t *recording_buffer = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_INTERNAL);
                assert(recording_buffer != NULL);

                /* Open file for recording */
                FILE *record_file = fopen(recording_filename, "wb");
                assert(record_file != NULL);

                /* Write WAV file header */
                const dumb_wav_header_t recording_header = {
                    .bits_per_sample = 16,
                    .data_size = RECORDING_LENGTH * BUFFER_SIZE,
                    .num_channels = 1,
                    .sample_rate = SAMPLE_RATE
                };
                if (fwrite((void *)&recording_header, 1, sizeof(dumb_wav_header_t), record_file) != sizeof(dumb_wav_header_t)) {
                    ESP_LOGW(TAG, "Error in writting to file");
                    continue;
                }

                lv_checkbox_set_checked(recording_checkbox, true);
                ESP_ERROR_CHECK(i2s_zero_dma_buffer(BSP_I2S_NUM)); // Reset RX buffers before i2s_read
                ESP_LOGI(TAG, "Recording start");

                size_t bytes_written_to_spiffs = 0;
                while (bytes_written_to_spiffs < RECORDING_LENGTH * BUFFER_SIZE) {
                    size_t bytes_received_from_i2s;

                    ESP_ERROR_CHECK(i2s_read(BSP_I2S_NUM, recording_buffer, BUFFER_SIZE, &bytes_received_from_i2s,
                                             5000 / portTICK_RATE_MS));

                    /* Write WAV file data */
                    size_t data_written = fwrite(recording_buffer, 1, bytes_received_from_i2s, record_file);
                    bytes_written_to_spiffs += data_written;
                }

                ESP_LOGI(TAG, "Recording stop, length: %i bytes", bytes_written_to_spiffs);
                lv_checkbox_set_checked(recording_checkbox, false);

                fclose(record_file);
                free(recording_buffer);
                break;
            }
            case BSP_BUTTON_MODE: {
                static bool play_recording = true;

                /* Switch between saved and recorded wav file */
                play_filename = play_recording ? recording_filename : music_filename;
                play_recording = !play_recording;
                ESP_LOGI(TAG, "Playback file changed to %s", play_filename);
                break;
            }
            case BSP_BUTTON_PLAY: {
                int16_t *wav_bytes = malloc(BUFFER_SIZE);
                assert(wav_bytes != NULL);

                /* Open WAV file */
                ESP_LOGI(TAG, "Playing file %s", play_filename);
                FILE *play_file = fopen(play_filename, "rb");
                assert(play_file != NULL);

                /* Read WAV header file */
                dumb_wav_header_t wav_header;
                if (fread((void *)&wav_header, 1, sizeof(wav_header), play_file) != sizeof(wav_header)) {
                    ESP_LOGW(TAG, "Error in reading file");
                    continue;
                }
                ESP_LOGI(TAG, "Number of channels: %d", wav_header.num_channels);
                ESP_LOGI(TAG, "Bits per sample: %d", wav_header.bits_per_sample);
                ESP_LOGI(TAG, "Sample rate: %d", wav_header.sample_rate);
                ESP_LOGI(TAG, "Data size: %d", wav_header.data_size);

                lv_checkbox_set_checked(playing_checkbox, true);

                uint32_t bytes_send_to_i2s = 0;
                while (bytes_send_to_i2s < wav_header.data_size) {
                    /* Get data from SPIFFS */
                    size_t bytes_read_from_spiffs = fread(wav_bytes, 1, BUFFER_SIZE, play_file);

                    /* Send it to I2S */
                    size_t i2s_bytes_written;
                    ESP_ERROR_CHECK(i2s_write(BSP_I2S_NUM, wav_bytes, bytes_read_from_spiffs, &i2s_bytes_written,
                                              500 / portTICK_RATE_MS));
                    bytes_send_to_i2s += i2s_bytes_written;
                }

                lv_checkbox_set_checked(playing_checkbox, false);
                fclose(play_file);
                free(wav_bytes);
                break;
            }
            case BSP_BUTTON_SET: {
                /* Wit each button SET press, toggle RGB led on/off and set random color */
                static bool rgb_off = false;
                if (rgb_off) {
                    rgb_led->set_pixel(rgb_led, 0, 0, 0, 0);
                } else {
                    rgb_led->set_pixel(rgb_led, 0, rand() % 100, rand() % 100, rand() % 100);
                }

                rgb_led->refresh(rgb_led, 100);
                rgb_off = !rgb_off;
                break;
            }
            case BSP_BUTTON_VOLDOWN: {
                int vol, vol_real;
                es8311_voice_volume_get(es8311_dev, &vol);
                vol -= 5;
                es8311_voice_volume_set(es8311_dev, vol, &vol_real);
                lv_arc_set_value(volume_arc, vol);
                ESP_LOGI(TAG, "Volume Down: %i", vol_real);
                break;
            }
            case BSP_BUTTON_VOLUP: {
                int vol, vol_real;
                es8311_voice_volume_get(es8311_dev, &vol);
                vol += 5;
                es8311_voice_volume_set(es8311_dev, vol, &vol_real);
                lv_arc_set_value(volume_arc, vol);
                ESP_LOGI(TAG, "Volume Up: %i", vol_real);
                break;
            }
            default:
                ESP_LOGW(TAG, "Button index out of range");
            }
        }
    }
}

void app_main(void)
{
    /* Init board peripherals */
    bsp_i2c_init(); // Used by ES8311 driver
    ESP_ERROR_CHECK(spiffs_init());
    rgb_led = led_strip_init(0, BSP_LEDSTRIP_IO, 1);
    assert(rgb_led != NULL);
    ESP_ERROR_CHECK(rgb_led->clear(rgb_led, 100));

    /* Needed from random RGB LED color generation */
    time_t t;
    srand((unsigned) time(&t));

    /* Create FreeRTOS tasks and queues */
    audio_button_q = xQueueCreate(10, sizeof(uint8_t));
    assert (audio_button_q != NULL);

    BaseType_t ret = xTaskCreate(audio_task, "audio_task", 4096, NULL, 6, NULL);
    assert(ret == pdPASS);

    TaskHandle_t bsp_display_task_handle;
    ret = xTaskCreate(bsp_display_task, "bsp_display", 4096 * 2, xTaskGetCurrentTaskHandle(), 3, &bsp_display_task_handle);
    assert(ret == pdPASS);

    /* Init audio buttons */
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        audio_button[i] = button_create(&bsp_button_config[i]);
        assert(audio_button[i] != NULL);
        ESP_ERROR_CHECK(button_register_cb(audio_button[i], BUTTON_PRESS_DOWN, btn_handler));
    }

    /* Wait until LVGL is ready */
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000))) {
        vTaskSuspend(bsp_display_task_handle);
        /*Create a window*/
        lv_obj_t *win = lv_win_create(lv_scr_act(), NULL);
        lv_win_set_title(win, "ESP32-S2-Kaluga-Kit:\nBoard Support Package example");
        lv_win_set_scrollbar_mode(win, LV_SCROLLBAR_MODE_OFF);

        /* Container */
        lv_obj_t *cont = lv_cont_create(win, NULL);
        lv_cont_set_fit(cont, LV_FIT_TIGHT);
        lv_cont_set_layout(cont, LV_LAYOUT_COLUMN_LEFT);

        /* Checkboxes */
        recording_checkbox = lv_checkbox_create(cont, NULL);
        lv_checkbox_set_text(recording_checkbox, "Recording");
        playing_checkbox = lv_checkbox_create(cont, NULL);
        lv_checkbox_set_text(playing_checkbox, "Playing");

        /* Volume arc */
        volume_arc = lv_arc_create(win, NULL);
        lv_arc_set_end_angle(volume_arc, 200);
        lv_obj_set_size(volume_arc, 130, 130);
        lv_obj_align(volume_arc, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 0);
        lv_arc_set_value(volume_arc, DEFAULT_VOLUME);
        lv_obj_t *volume_label = lv_label_create(volume_arc, NULL);
        lv_label_set_text_static(volume_label, "Volume");
        lv_obj_align(volume_label, volume_arc, LV_ALIGN_IN_BOTTOM_MID, 0, 0);
        vTaskResume(bsp_display_task_handle);
    } else {
        ESP_LOGW(TAG, "LVGL init timed out");
    }
}
