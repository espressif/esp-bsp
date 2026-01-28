# BSP: ESP32-S3-EYE

| [HW Reference](https://www.espressif.com/en/products/devkits/esp-eye/overview) | [HOW TO USE API](API.md) | [EXAMPLES](#compatible-bsp-examples) | [![Component Registry](https://components.espressif.com/components/espressif/esp32_s3_eye/badge.svg)](https://components.espressif.com/components/espressif/esp32_s3_eye) | ![maintenance-status](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg) |
| --- | --- | --- | --- | -- |

## Overview

<table>
<tr><td>

The ESP32-S3-EYE board consists of two parts: the main board (ESP32-S3-EYE-MB) that integrates the ESP32-S3-WROOM-1 module, camera, uSD card slot, digital microphone, USB port, and function buttons; and the sub board (ESP32-S3-EYE-SUB) that contains an LCD display. The main board and sub board are connected through pin headers.

**The development board has the following features:**
* ESP32-S3 module with built-in 8 MB flash and 8 MB octal RAM
* 1.3-inch 240x240 LCD color screen
* Onboard uSD card slot
* Digital microphone
* Accelerometer
* OV2640 camera

</td><td width="200">
  <img src="doc/esp32_s3_eye.webp">
</td></tr>
</table>

![](https://raw.githubusercontent.com/espressif/esp-who/master/docs/_static/get-started/ESP32-S3-EYE_MB-annotated-photo.png)


## Capabilities and dependencies

<div align="center">
<!-- START_DEPENDENCIES -->

|     Available    |       Capability       |Controller/Codec|                                               Component                                              |   Version  |
|------------------|------------------------|----------------|------------------------------------------------------------------------------------------------------|------------|
|:heavy_check_mark:|     :pager: DISPLAY    |     st7789     |                                                  idf                                                 |    >=5.4   |
|:heavy_check_mark:|:black_circle: LVGL_PORT|                |    [espressif/esp_lvgl_port](https://components.espressif.com/components/espressif/esp_lvgl_port)    |     ^2     |
|        :x:       |    :point_up: TOUCH    |                |                                                                                                      |            |
|:heavy_check_mark:| :radio_button: BUTTONS |                |           [espressif/button](https://components.espressif.com/components/espressif/button)           |     ^4     |
|        :x:       |   :white_circle: KNOB  |                |                                                                                                      |            |
|:heavy_check_mark:|  :musical_note: AUDIO  |                |    [espressif/esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)    |    ~1.5    |
|        :x:       | :speaker: AUDIO_SPEAKER|                |                                                                                                      |            |
|:heavy_check_mark:| :microphone: AUDIO_MIC |                |                                                                                                      |            |
|:heavy_check_mark:|  :floppy_disk: SDCARD  |                |                                                  idf                                                 |    >=5.4   |
|:heavy_check_mark:|       :bulb: LED       |                |idf<br/>[espressif/led_indicator](https://components.espressif.com/components/espressif/led_indicator)|>=5.4<br/>^2|
|:heavy_check_mark:|     :camera: CAMERA    |     OV2640     |        [espressif/esp_video](https://components.espressif.com/components/espressif/esp_video)        |    ~1.4    |
|        :x:       |      :battery: BAT     |                |                                                                                                      |            |
|        :x:       |    :video_game: IMU    |                |                                                                                                      |            |

<!-- END_DEPENDENCIES -->
</div>

## Compatible BSP Examples

<div align="center">
<!-- START_EXAMPLES -->

| Example | Description | Try with ESP Launchpad |
| ------- | ----------- | ---------------------- |
| [Display Example](https://github.com/espressif/esp-bsp/tree/master/examples/display) | Show an image on the screen with a simple startup animation (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display-) |
| [Camera Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_camera_video) | Stream camera output to display (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_camera_video) |
| [LVGL Benchmark Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_lvgl_benchmark) | Run LVGL benchmark tests | - |
| [LVGL Demos Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_lvgl_demos) | Run the LVGL demo player - all LVGL examples are included (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_lvgl_demos-) |

<!-- END_EXAMPLES -->
</div>
