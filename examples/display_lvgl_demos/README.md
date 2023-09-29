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
    * `SPIRAM_SPEED_120M` (See [here](https://github.com/espressif/esp-dev-kits/tree/master/esp32-s3-lcd-ev-board#psram-120m-ddr) to enbale this feature of ESP-IDF)
    * `SPIRAM_FETCH_INSTRUCTIONS`
    * `SPIRAM_RODATA`
* Cache:
    * `ESP32S3_DATA_CACHE_LINE_64B` (It can be enabled only when using bounce buffer or PSRAM with Octal 120M. Otherwise it will cause screen drift.)
* LVGL
    * `LV_MEM_CUSTOM`
    * `LV_MEMCPY_MEMSET_STD`
    * `LV_ATTRIBUTE_FAST_MEM_USE_IRAM`

### Hardware Required

ESP32-S3-LCD-EV-Board or ESP32-S3-LCD-EV-Board-2

<a href="https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_lvgl_demos">
    <img alt="Try it with ESP Launchpad" src="https://espressif.github.io/esp-launchpad/assets/try_with_launchpad.png" width="250" height="70">
</a>
