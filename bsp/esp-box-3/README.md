# BSP: ESP-BOX-3

| [HW Reference](https://github.com/espressif/esp-box/tree/master/hardware) | [HOW TO USE API](API.md) | [EXAMPLES](#compatible-bsp-examples) | [![Component Registry](https://components.espressif.com/components/espressif/esp-box-3/badge.svg)](https://components.espressif.com/components/espressif/esp-box-3) | ![maintenance-status](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg) |
| --- | --- | --- | --- | --- |

## Overview

<table>
<tr><td>

ESP32-S3-BOX-3 is an AI voice development kit that is based on Espressif’s ESP32-S3 Wi-Fi + Bluetooth 5 (LE) SoC, with AI capabilities. In addition to ESP32-S3’s 512KB SRAM,

ESP32-S3-BOX-3 comes with 16MB of QSPI flash and 16MB of Octal PSRAM. ESP32-S3-BOX-3 is also equipped with a variety of peripherals, such as a 2.4-inch display with a 320x240 resolution, a capacitive touch screen, a dual microphone, a speaker, and two Pmod™-compatible headers which allow for the extensibility of the hardware.

ESP32-S3-BOX-3 also uses a Type-C USB connector that provides 5 V of power input, while also supporting serial and JTAG debugging, as well as a programming interface; all through the same connector.

</td><td width="200" valign="top">
  <img src="doc/esp-box-3.webp">
</td></tr>
</table>

![image](doc/pic.png)

## Capabilities and dependencies

<div align="center">
<!-- START_DEPENDENCIES -->

|     Available    |       Capability       |Controller/Codec|                                                                                                          Component                                                                                                          |     Version    |
|------------------|------------------------|----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------|
|:heavy_check_mark:|     :pager: DISPLAY    | st7789, ili9341|                                                          [espressif/esp_lcd_ili9341](https://components.espressif.com/components/espressif/esp_lcd_ili9341)<br/>idf                                                         |^2.0.1<br/>>=5.3|
|:heavy_check_mark:|:black_circle: LVGL_PORT|                |                                                                [espressif/esp_lvgl_port](https://components.espressif.com/components/espressif/esp_lvgl_port)                                                               |       ^2       |
|:heavy_check_mark:|    :point_up: TOUCH    | tt21100, gt911 |[espressif/esp_lcd_touch_gt911](https://components.espressif.com/components/espressif/esp_lcd_touch_gt911)<br/>[espressif/esp_lcd_touch_tt21100](https://components.espressif.com/components/espressif/esp_lcd_touch_tt21100)|    ^1<br/>^1   |
|:heavy_check_mark:| :radio_button: BUTTONS |                |                                                                       [espressif/button](https://components.espressif.com/components/espressif/button)                                                                      |       ^4       |
|:heavy_check_mark:|  :musical_note: AUDIO  |                |                                                                [espressif/esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)                                                               |     ~1.3.1     |
|:heavy_check_mark:| :speaker: AUDIO_SPEAKER|     es8311     |                                                                                                                                                                                                                             |                |
|:heavy_check_mark:| :microphone: AUDIO_MIC |     es7210     |                                                                                                                                                                                                                             |                |
|:heavy_check_mark:|  :floppy_disk: SDCARD  |                |                                                                                                             idf                                                                                                             |      >=5.3     |
|:heavy_check_mark:|    :video_game: IMU    |                |                                                                     [espressif/icm42670](https://components.espressif.com/components/espressif/icm42670)                                                                    |     ^2.0.1     |

<!-- END_DEPENDENCIES -->
</div>

## Compatible BSP Examples

<div align="center">
<!-- START_EXAMPLES -->

| Example | Description | Try with ESP Launchpad |
| ------- | ----------- | ---------------------- |
| [Display Example](https://github.com/espressif/esp-bsp/tree/master/examples/display) | Show an image on the screen with a simple startup animation (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display-) |
| [Display, Audio and Photo Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_audio_photo) | Complex demo: browse files from filesystem and play/display JPEG, WAV, or TXT files (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_audio_photo-) |
| [LVGL Benchmark Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_lvgl_benchmark) | Run LVGL benchmark tests | - |
| [LVGL Demos Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_lvgl_demos) | Run the LVGL demo player - all LVGL examples are included (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_lvgl_demos-) |
| [Display Rotation Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_rotation) | Rotate screen using buttons or an accelerometer (`BSP_CAPS_IMU`, if available) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_rotation-) |

<!-- END_EXAMPLES -->
</div>

<!-- START_BENCHMARK -->

## LVGL Benchmark

**DATE:** 19.08.2025 02:45

**LVGL version:** 9.3.0

| Name | Avg. CPU | Avg. FPS | Avg. time | render time | flush time |
| ---- | :------: | :------: | :-------: | :---------: | :--------: |
| Empty screen | 93%  | 26  | 33  | 3  | 30  |
| Moving wallpaper | 95%  | 26  | 35  | 5  | 30  |
| Single rectangle | 10%  | 92  | 0  | 0  | 0  |
| Multiple rectangles | 90%  | 52  | 15  | 2  | 13  |
| Multiple RGB images | 7%  | 88  | 0  | 0  | 0  |
| Multiple ARGB images | 21%  | 87  | 3  | 2  | 1  |
| Rotated ARGB images | 81%  | 57  | 17  | 12  | 5  |
| Multiple labels | 87%  | 79  | 9  | 3  | 6  |
| Screen sized text | 98%  | 15  | 59  | 28  | 31  |
| Multiple arcs | 9%  | 87  | 0  | 0  | 0  |
| Containers | 37%  | 74  | 15  | 4  | 11  |
| Containers with overlay | 87%  | 20  | 44  | 14  | 30  |
| Containers with opa | 49%  | 68  | 19  | 7  | 12  |
| Containers with opa_layer | 59%  | 54  | 27  | 15  | 12  |
| Containers with scrolling | 95%  | 21  | 44  | 13  | 31  |
| Widgets demo | 97%  | 25  | 25  | 11  | 14  |
| All scenes avg. | 63%  | 54  | 21  | 7  | 14  |



<!-- END_BENCHMARK -->
