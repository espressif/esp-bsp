# BSP: ESP32-S3-EYE

| [HW Reference](https://www.espressif.com/en/products/devkits/esp-eye/overview) | [HOW TO USE API](https://github.com/espressif/esp-bsp/blob/master/docu/how_to_use.md) | [EXAMPLES](#compatible-bsp-examples) | [![Component Registry](https://components.espressif.com/components/espressif/esp32_s3_eye/badge.svg)](https://components.espressif.com/components/espressif/esp32_s3_eye) | ![maintenance-status](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg) |
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

|     Available    |       Capability       |Controller/Codec|                                           Component                                          |Version|
|------------------|------------------------|----------------|----------------------------------------------------------------------------------------------|-------|
|:heavy_check_mark:|     :pager: DISPLAY    |     st7789     |                                              idf                                             | >=5.4 |
|:heavy_check_mark:|:black_circle: LVGL_PORT|                |[espressif/esp_lvgl_port](https://components.espressif.com/components/espressif/esp_lvgl_port)|   ^2  |
|        :x:       |    :point_up: TOUCH    |                |                                                                                              |       |
|:heavy_check_mark:| :radio_button: BUTTONS |                |       [espressif/button](https://components.espressif.com/components/espressif/button)       |   ^4  |
|:heavy_check_mark:|  :musical_note: AUDIO  |                |[espressif/esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)| ~1.3.1|
|        :x:       | :speaker: AUDIO_SPEAKER|                |                                                                                              |       |
|:heavy_check_mark:| :microphone: AUDIO_MIC |                |                                                                                              |       |
|:heavy_check_mark:|  :floppy_disk: SDCARD  |                |                                              idf                                             | >=5.4 |
|:heavy_check_mark:|    :video_game: IMU    |                |               [qma6100p](https://components.espressif.com/components/qma6100p)               |   ^2  |
|:heavy_check_mark:|     :camera: CAMERA    |                | [espressif/esp32-camera](https://components.espressif.com/components/espressif/esp32-camera) |^2.0.13|

<!-- END_DEPENDENCIES -->
</div>

## Compatible BSP Examples

<div align="center">
<!-- START_EXAMPLES -->

| Example | Description | Try with ESP Launchpad |
| ------- | ----------- | ---------------------- |
| [Display Example](https://github.com/espressif/esp-bsp/tree/master/examples/display) | Show an image on the screen with a simple startup animation (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display) |
| [Camera Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_camera) | Stream camera output to display (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_camera) |
| [LVGL Benchmark Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_lvgl_benchmark) | Run LVGL benchmark tests | - |
| [LVGL Demos Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_lvgl_demos) | Run the LVGL demo player - all LVGL examples are included (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_lvgl_demo) |

<!-- END_EXAMPLES -->
</div>

<!-- START_BENCHMARK -->

# Benchmark for BOARD esp32_s3_eye

**DATE:** 06.05.2025 14:51

**LVGL version:** 9.2.2

| Name | Avg. CPU | Avg. FPS | Avg. time | render time | flush time |
| ---- | :------: | :------: | :-------: | :---------: | :--------: |
| Empty screen | 92%  | 60  | 13 *<span style="color:green"><sub>(-1)</sub></span>* | 2 *<span style="color:green"><sub>(-1)</sub></span>* | 11  |
| Moving wallpaper | 61% *<span style="color:green"><sub>(-1)</sub></span>* | 76  | 9  | 4  | 5  |
| Single rectangle | 17% *<span style="color:red"><sub>(+3)</sub></span>* | 92  | 0  | 0  | 0  |
| Multiple rectangles | 83%  | 74  | 11  | 4  | 7  |
| Multiple RGB images | 17% *<span style="color:green"><sub>(-1)</sub></span>* | 90  | 0  | 0  | 0  |
| Multiple ARGB images | 17% *<span style="color:green"><sub>(-1)</sub></span>* | 88 *<span style="color:red"><sub>(-2)</sub></span>* | 3  | 3  | 0  |
| Rotated ARGB images | 71% *<span style="color:red"><sub>(+1)</sub></span>* | 58 *<span style="color:red"><sub>(-5)</sub></span>* | 16 *<span style="color:red"><sub>(+2)</sub></span>* | 14 *<span style="color:red"><sub>(+1)</sub></span>* | 2 *<span style="color:red"><sub>(+1)</sub></span>* |
| Multiple labels | 21% *<span style="color:red"><sub>(+4)</sub></span>* | 93 *<span style="color:green"><sub>(+4)</sub></span>* | 1  | 1 *<span style="color:red"><sub>(+1)</sub></span>* | 0 *<span style="color:green"><sub>(-1)</sub></span>* |
| Screen sized text | 92%  | 35  | 25  | 14  | 11  |
| Multiple arcs | 11% *<span style="color:green"><sub>(-1)</sub></span>* | 89 *<span style="color:red"><sub>(-2)</sub></span>* | 0  | 0  | 0  |
| Containers | 12% *<span style="color:green"><sub>(-1)</sub></span>* | 91 *<span style="color:red"><sub>(-2)</sub></span>* | 4  | 4  | 0  |
| Containers with overlay | 91% *<span style="color:red"><sub>(+1)</sub></span>* | 40 *<span style="color:red"><sub>(-1)</sub></span>* | 21  | 11  | 10  |
| Containers with opa | 25% *<span style="color:green"><sub>(-3)</sub></span>* | 91 *<span style="color:green"><sub>(+1)</sub></span>* | 7 *<span style="color:red"><sub>(+2)</sub></span>* | 6 *<span style="color:red"><sub>(+1)</sub></span>* | 1 *<span style="color:red"><sub>(+1)</sub></span>* |
| Containers with opa_layer | 29%  | 82 *<span style="color:green"><sub>(+1)</sub></span>* | 12 *<span style="color:red"><sub>(+1)</sub></span>* | 11 *<span style="color:red"><sub>(+1)</sub></span>* | 1  |
| Containers with scrolling | 94% *<span style="color:green"><sub>(-1)</sub></span>* | 39 *<span style="color:red"><sub>(-1)</sub></span>* | 22 *<span style="color:green"><sub>(-1)</sub></span>* | 11 *<span style="color:green"><sub>(-1)</sub></span>* | 11  |
| Widgets demo | 96%  | 44  | 8 *<span style="color:green"><sub>(-1)</sub></span>* | 8  | 0 *<span style="color:green"><sub>(-1)</sub></span>* |
| All scenes avg. | 51%  | 71  | 8  | 5  | 3  |

***



<!-- END_BENCHMARK -->
