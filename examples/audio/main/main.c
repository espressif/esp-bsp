/*
 * SPDX-FileCopyrightText: 2021-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/**
 * @file
 * @brief BSP Audio Example
 * @details Play and record WAV file
 * @example https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=audio-
 */

#include <stdio.h>
#include <inttypes.h>
#include <sdkconfig.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "bsp/esp-bsp.h"

/* Buffer for reading/writing to I2S driver. Same length as SPIFFS buffer and I2S buffer, for optimal read/write performance.
   Recording audio data path:
   I2S peripheral -> I2S buffer (DMA) -> App buffer (RAM) -> SPIFFS buffer -> External SPI Flash.
   Vice versa for playback. */
#define BUFFER_SIZE     (1024)
#define SAMPLE_RATE     (16000) // For recording
#define DEFAULT_VOLUME  (50)
/* The recording will be RECORDING_LENGTH * BUFFER_SIZE long (in bytes)
   With sampling frequency 16000 Hz and 16bit mono resolution it equals to ~5.12 seconds */
#define RECORDING_LENGTH (160)

/* Globals */
static const char *TAG = "example";
static QueueHandle_t audio_button_q = NULL;

static void btn_handler(void *button_handle, void *usr_data)
{
    int button_pressed = (int)usr_data;
    xQueueSend(audio_button_q, &button_pressed, 0);
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

static void audio_task(void *arg)
{
    esp_codec_dev_handle_t spk_codec_dev = bsp_audio_codec_speaker_init();
    assert(spk_codec_dev);
    esp_codec_dev_set_out_vol(spk_codec_dev, DEFAULT_VOLUME);
    esp_codec_dev_handle_t mic_codec_dev = bsp_audio_codec_microphone_init();

    /* Pointer to a file that is going to be played */
    const char music_filename[] = BSP_SPIFFS_MOUNT_POINT"/16bit_mono_22_05khz.wav";
    const char recording_filename[] = BSP_SPIFFS_MOUNT_POINT"/recording.wav";
    const char *play_filename = music_filename;

    while (1) {
        int btn_index = 0;
        if (xQueueReceive(audio_button_q, &btn_index, portMAX_DELAY) == pdTRUE) {
            switch (btn_index) {
            case BSP_BUTTON_REC: {
                if (mic_codec_dev == NULL) {
                    ESP_LOGW(TAG, "This board does not support microphone recording!");
                    break;
                }
                int16_t *recording_buffer = heap_caps_malloc(BUFFER_SIZE, MALLOC_CAP_DEFAULT);
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
                    ESP_LOGW(TAG, "Error in writing to file");
                    continue;
                }
                setvbuf(record_file, NULL, _IOFBF, BUFFER_SIZE);

                esp_codec_dev_sample_info_t fs = {
                    .sample_rate = SAMPLE_RATE,
                    .channel = 1,
                    .bits_per_sample = 16,
                };
                esp_codec_dev_set_in_gain(mic_codec_dev, 42.0);
                esp_codec_dev_open(mic_codec_dev, &fs);

                ESP_LOGI(TAG, "Recording start");
                size_t bytes_written_to_spiffs = 0;
                while (bytes_written_to_spiffs < RECORDING_LENGTH * BUFFER_SIZE) {
                    /* Read data from codec and write it to SPIFFS */
                    ESP_ERROR_CHECK(esp_codec_dev_read(mic_codec_dev, recording_buffer, BUFFER_SIZE));
                    size_t data_written = fwrite(recording_buffer, 1, BUFFER_SIZE, record_file);
                    bytes_written_to_spiffs += data_written;
                }

                ESP_LOGI(TAG, "Recording stop, length: %i bytes", bytes_written_to_spiffs);
                fclose(record_file);
                free(recording_buffer);
                esp_codec_dev_close(mic_codec_dev);
                break;
            }
            case BSP_BUTTON_SET: {
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
                ESP_LOGI(TAG, "Number of channels: %" PRIu16 "", wav_header.num_channels);
                ESP_LOGI(TAG, "Bits per sample: %" PRIu16 "", wav_header.bits_per_sample);
                ESP_LOGI(TAG, "Sample rate: %" PRIu32 "", wav_header.sample_rate);
                ESP_LOGI(TAG, "Data size: %" PRIu32 "", wav_header.data_size);

                esp_codec_dev_sample_info_t fs = {
                    .sample_rate = wav_header.sample_rate,
                    .channel = wav_header.num_channels,
                    .bits_per_sample = wav_header.bits_per_sample,
                };
                esp_codec_dev_open(spk_codec_dev, &fs);

                uint32_t bytes_send_to_i2s = 0;
                while (bytes_send_to_i2s < wav_header.data_size) {
                    /* Get data from SPIFFS and send it to codec */
                    size_t bytes_read_from_spiffs = fread(wav_bytes, 1, BUFFER_SIZE, play_file);
                    esp_codec_dev_write(spk_codec_dev, wav_bytes, bytes_read_from_spiffs);
                    bytes_send_to_i2s += bytes_read_from_spiffs;
                }
                fclose(play_file);
                free(wav_bytes);
                esp_codec_dev_close(spk_codec_dev);
                break;
            }
            case BSP_BUTTON_VOLDOWN: {
                int vol;
                esp_codec_dev_get_out_vol(spk_codec_dev, &vol);
                vol = (vol - 5 < 0) ? 0 : vol - 5;
                esp_codec_dev_set_out_vol(spk_codec_dev, vol);
                ESP_LOGI(TAG, "Volume Down: %i", vol);
                break;
            }
            case BSP_BUTTON_VOLUP: {
                int vol;
                esp_codec_dev_get_out_vol(spk_codec_dev, &vol);
                vol = (vol + 5 > 100) ? 100 : vol + 5;
                esp_codec_dev_set_out_vol(spk_codec_dev, vol);
                ESP_LOGI(TAG, "Volume Up: %i", vol);
                break;
            }
            default:
                ESP_LOGI(TAG, "No function for this button");
                break;
            }
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(bsp_spiffs_mount());

    /* Create FreeRTOS tasks and queues */
    audio_button_q = xQueueCreate(10, sizeof(int));
    assert (audio_button_q != NULL);

    BaseType_t ret = xTaskCreate(audio_task, "audio_task", 4096, NULL, 6, NULL);
    assert(ret == pdPASS);

    /* Init audio buttons */
    button_handle_t btns[BSP_BUTTON_NUM];
    ESP_ERROR_CHECK(bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM));
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
#if BUTTON_VER_MAJOR >= 4
        ESP_ERROR_CHECK(iot_button_register_cb(btns[i], BUTTON_PRESS_DOWN, NULL, btn_handler, (void *) i));
#else
        ESP_ERROR_CHECK(iot_button_register_cb(btns[i], BUTTON_PRESS_DOWN, btn_handler, (void *) i));
#endif
    }
}
