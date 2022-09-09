/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <string.h>
#include "esp_log.h"
#include "esp_spiffs.h"

#include "bsp/esp32_s2_kaluga_kit.h"
#include "es8311.h"
#include "led_strip.h"
#include "lvgl.h"
#include "disp_example.h"

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
static const char *TAG = "Kaluga";
static button_handle_t audio_button[BSP_BUTTON_NUM] = {};
static QueueHandle_t audio_button_q = NULL;
static led_strip_handle_t rgb_led = NULL;
static i2s_chan_handle_t i2s_tx_chan;
static i2s_chan_handle_t i2s_rx_chan;

static void btn_handler(void *arg, void *arg2)
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

static esp_err_t spiffs_init(void)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Initializing SPIFFS");

    const esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false
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
    /* Create and configure ES8311 I2C driver */
    es8311_handle_t es8311_dev = es8311_create(BSP_I2C_NUM, ES8311_ADDRRES_0);
    const es8311_clock_config_t clk_cfg = BSP_ES8311_SCLK_CONFIG(SAMPLE_RATE);
    es8311_init(es8311_dev, &clk_cfg, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
    es8311_voice_volume_set(es8311_dev, DEFAULT_VOLUME, NULL);

    /* Microphone settings */
    es8311_microphone_config(es8311_dev, false);
    es8311_microphone_gain_set(es8311_dev, ES8311_MIC_GAIN_42DB);

    /* Configure I2S peripheral and Power Amplifier */
    bsp_audio_init(NULL, &i2s_tx_chan, &i2s_rx_chan);
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

                disp_set_recording(true);
                ESP_LOGI(TAG, "Recording start");

                size_t bytes_written_to_spiffs = 0;
                while (bytes_written_to_spiffs < RECORDING_LENGTH * BUFFER_SIZE) {
                    size_t bytes_received_from_i2s;
                    ESP_ERROR_CHECK(i2s_channel_read(i2s_rx_chan, recording_buffer, BUFFER_SIZE, &bytes_received_from_i2s, pdMS_TO_TICKS(5000)));

                    /* Write WAV file data */
                    size_t data_written = fwrite(recording_buffer, 1, bytes_received_from_i2s, record_file);
                    bytes_written_to_spiffs += data_written;
                }

                ESP_LOGI(TAG, "Recording stop, length: %i bytes", bytes_written_to_spiffs);
                disp_set_recording(false);

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
                if (play_file == NULL) {
                    ESP_LOGW(TAG, "%s file does not exist!", play_filename);
                    break;
                }

                /* Read WAV header file */
                dumb_wav_header_t wav_header;
                if (fread((void *)&wav_header, 1, sizeof(wav_header), play_file) != sizeof(wav_header)) {
                    ESP_LOGW(TAG, "Error in reading file");
                    break;
                }
                ESP_LOGI(TAG, "Number of channels: %d", wav_header.num_channels);
                ESP_LOGI(TAG, "Bits per sample: %d", wav_header.bits_per_sample);
                ESP_LOGI(TAG, "Sample rate: %d", wav_header.sample_rate);
                ESP_LOGI(TAG, "Data size: %d", wav_header.data_size);

                disp_set_playing(true);

                uint32_t bytes_send_to_i2s = 0;
                while (bytes_send_to_i2s < wav_header.data_size) {
                    /* Get data from SPIFFS */
                    size_t bytes_read_from_spiffs = fread(wav_bytes, 1, BUFFER_SIZE, play_file);

                    /* Send it to I2S */
                    size_t i2s_bytes_written;
                    ESP_ERROR_CHECK(i2s_channel_write(i2s_tx_chan, wav_bytes, bytes_read_from_spiffs, &i2s_bytes_written, pdMS_TO_TICKS(500)));
                    bytes_send_to_i2s += i2s_bytes_written;
                }

                disp_set_playing(false);
                fclose(play_file);
                free(wav_bytes);
                break;
            }
            case BSP_BUTTON_SET: {
                /* Wit each button SET press, toggle RGB led on/off and set random color */
                static bool rgb_off = false;
                if (rgb_off) {
                    led_strip_set_pixel(rgb_led, 0, 0, 0, 0);
                } else {
                    led_strip_set_pixel(rgb_led, 0, rand() % 100, rand() % 100, rand() % 100);
                }

                led_strip_refresh(rgb_led);
                rgb_off = !rgb_off;
                break;
            }
            case BSP_BUTTON_VOLDOWN: {
                int vol, vol_real;
                es8311_voice_volume_get(es8311_dev, &vol);
                vol -= 5;
                es8311_voice_volume_set(es8311_dev, vol, &vol_real);
                disp_set_volume(vol);
                ESP_LOGI(TAG, "Volume Down: %i", vol_real);
                break;
            }
            case BSP_BUTTON_VOLUP: {
                int vol, vol_real;
                es8311_voice_volume_get(es8311_dev, &vol);
                vol += 5;
                es8311_voice_volume_set(es8311_dev, vol, &vol_real);
                disp_set_volume(vol);
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

    /* Configure RGB LED */
    const led_strip_config_t rgb_config = {
        .strip_gpio_num = BSP_LEDSTRIP_IO,
        .max_leds = 1,
    };
    const led_strip_rmt_config_t rmt_config = {0};
    led_strip_new_rmt_device(&rgb_config, &rmt_config, &rgb_led);
    assert(rgb_led != NULL);
    ESP_ERROR_CHECK(led_strip_clear(rgb_led));

    /* Needed from random RGB LED color generation */
    time_t t;
    srand((unsigned) time(&t));

    /* Create FreeRTOS tasks and queues */
    audio_button_q = xQueueCreate(10, sizeof(uint8_t));
    assert (audio_button_q != NULL);

    BaseType_t ret = xTaskCreate(audio_task, "audio_task", 4096, NULL, 6, NULL);
    assert(ret == pdPASS);

    /* Init audio buttons */
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        audio_button[i] = iot_button_create(&bsp_button_config[i]);
        assert(audio_button[i] != NULL);
        ESP_ERROR_CHECK(iot_button_register_cb(audio_button[i], BUTTON_PRESS_DOWN, btn_handler, NULL));
    }

    bsp_display_start(); // Start LVGL and LCD driver
    disp_init();         // Create LVGL screen and widgets
    disp_set_volume(DEFAULT_VOLUME);
}
