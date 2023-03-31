# Display LVGL Demos

This example shows LVGL internal demos with RGB LCD.

### Configurations

To improve display performance (FPS), please set the following configurations:

* System:
    * `ESP_DEFAULT_CPU_FREQ_MHZ_240`
    * `FREERTOS_HZ` = 1000
    * `COMPILER_OPTIMIZATION_PERF`
* Flash:
    * `ESPTOOLPY_FLASHMODE_QIO`
    * `ESPTOOLPY_FLASHFREQ_120M`
* PSRAM:
    * `SPIRAM_MODE_OCT`
    * `SPIRAM_SPEED_120M` (See [here](https://github.com/espressif/esp-dev-kits/tree/master/esp32-s3-lcd-ev-board/factory#idf-patch) to enbale this feature of ESP-IDF)
    * `SPIRAM_FETCH_INSTRUCTIONS`
    * `SPIRAM_RODATA`
* Cache:
    * `ESP32S3_DATA_CACHE_LINE_64B` (It can be enabled only when using bounce buffer or PSRAM with Octal 120M. Otherwise it will cause screen drift.)
* LVGL
    * `LV_MEM_CUSTOM`
    * `LV_MEMCPY_MEMSET_STD`
    * `LV_ATTRIBUTE_FAST_MEM_USE_IRAM`

### Hardware Required

ESP32-S3-LCD-EV-BOARD with 800x480 or 480x480 LCD sub-board.
