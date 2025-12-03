# BSP: ESP32-P4 Function EV Board

| [HW Reference](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/user_guide.html) | [HOW TO USE API](API.md) | [EXAMPLES](#compatible-bsp-examples) | [![Component Registry](https://components.espressif.com/components/espressif/esp32_p4_function_ev_board/badge.svg)](https://components.espressif.com/components/espressif/esp32_p4_function_ev_board) | ![maintenance-status](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg) |
| --- | --- | --- | --- | -- |

## Overview

<table>
<tr><td>

ESP32-P4-Function-EV-Board is a multimedia development board based on the ESP32-P4 chip. ESP32-P4 chip features a dual-core 400 MHz RISC-V processor and supports up to 32 MB PSRAM. In addition, ESP32-P4 supports USB 2.0 specification, MIPI-CSI/DSI, H264 Encoder, and various other peripherals. With all of its outstanding features, the board is an ideal choice for developing low-cost, high-performance, low-power network-connected audio and video products.

</td><td width="200">
  <img src="doc/esp32_p4_function_ev_board.webp">
</td></tr>
</table>

![](doc/esp32-p4-function-ev-board-annotated-photo-front.png)

## HW Version

| HW version | BSP Version |
| :--------: | :---------: |
|    V1.0    |      ^1     |
|    V1.2    |      ^2     |
|    [V1.4](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/user_guide_v1.4.html)    |      ^3     |
|    [V1.5](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/user_guide.html)    |      ^3     |

> [!NOTE]
> For using boards with ESP32-P4 ECO5 (Board v1.5), change `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=n` in `menuconfig`.

## Configuration

Configuration in `menuconfig`.

Selection LCD display `Board Support Package(ESP32-P4) --> Display --> Select LCD type`
- LCD 1280x800 - ili9881c (default)
- LCD 1024x600 - ek79007
- HDMI - lt8912b
    - 800x600@60HZ
    - 1280x720@60HZ
    - 1280x800@60HZ
    - 1920x1080@30HZ

Selection color format `Board Support Package(ESP32-P4) --> Display --> Select LCD color format`
- RGB565 (default)
- RGB888

## HDMI Support

This BSP supports HDMI converter Lontium LT8912B. Follow these rules for using it with HDMI:
- Use ESP-IDF 5.4 or older (from commit [93fdbf2](https://github.com/espressif/esp-idf/commit/93fdbf25b3ea7e44d1f519ed61050847dcc8a076))
- Only RGB888 is supported with HDMI
- Use MIPI-DSI to HDMI converter Lontium LT8912B

## Capabilities and dependencies

<div align="center">
<!-- START_DEPENDENCIES -->

|     Available    |       Capability       |     Controller/Codec     |                                                                                                                                                         Component                                                                                                                                                        |                            Version                           |
|------------------|------------------------|--------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------|
|:heavy_check_mark:|     :pager: DISPLAY    |ek79007, ili9881c, lt8912b|idf<br/>[espressif/esp_lcd_ek79007](https://components.espressif.com/components/espressif/esp_lcd_ek79007)<br/>[espressif/esp_lcd_ili9881c](https://components.espressif.com/components/espressif/esp_lcd_ili9881c)<br/>[espressif/esp_lcd_lt8912b](https://components.espressif.com/components/espressif/esp_lcd_lt8912b)|>=5.4<br/>>=2.0.1,<3.0.0<br/>>=1.0.3,<2.0.0<br/>>=0.1.3,<1.0.0|
|:heavy_check_mark:|:black_circle: LVGL_PORT|                          |                                                                                                              [espressif/esp_lvgl_port](https://components.espressif.com/components/espressif/esp_lvgl_port)                                                                                                              |                              ^2                              |
|:heavy_check_mark:|    :point_up: TOUCH    |           gt911          |                                                                                                        [espressif/esp_lcd_touch_gt911](https://components.espressif.com/components/espressif/esp_lcd_touch_gt911)                                                                                                        |                              ^1                              |
|        :x:       | :radio_button: BUTTONS |                          |                                                                                                                                                                                                                                                                                                                          |                                                              |
|:heavy_check_mark:|  :musical_note: AUDIO  |                          |                                                                                                              [espressif/esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)                                                                                                              |                             ~1.5                             |
|:heavy_check_mark:| :speaker: AUDIO_SPEAKER|          es8311          |                                                                                                                                                                                                                                                                                                                          |                                                              |
|:heavy_check_mark:| :microphone: AUDIO_MIC |          es8311          |                                                                                                                                                                                                                                                                                                                          |                                                              |
|:heavy_check_mark:|  :floppy_disk: SDCARD  |                          |                                                                                                                                                            idf                                                                                                                                                           |                             >=5.4                            |
|        :x:       |    :video_game: IMU    |                          |                                                                                                                                                                                                                                                                                                                          |                                                              |
|:heavy_check_mark:|     :camera: CAMERA    |      OV5647, SC2336      |                                                                                                                                                                                                                                                                                                                          |                                                              |

<!-- END_DEPENDENCIES -->
</div>

## Compatible BSP Examples

<div align="center">
<!-- START_EXAMPLES -->

| Example | Description | Try with ESP Launchpad |
| ------- | ----------- | ---------------------- |
| [Display Example](https://github.com/espressif/esp-bsp/tree/master/examples/display) | Show an image on the screen with a simple startup animation (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display-) |
| [Camera Example (MIPI-CSI)](https://github.com/espressif/esp-bsp/tree/master/examples/display_camera_csi) | Stream camera (MIPI-CSI) output to display (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_camera-) |
| [LVGL Benchmark Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_lvgl_benchmark) | Run LVGL benchmark tests | - |
| [LVGL Demos Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_lvgl_demos) | Run the LVGL demo player - all LVGL examples are included (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_lvgl_demos-) |
| [Display Rotation Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_rotation) | Rotate screen using buttons or an accelerometer (`BSP_CAPS_IMU`, if available) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_rotation-) |
| [Display SD card Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_sdcard) | Example of mounting an SD card using SD-MMC/SPI with display interaction. This example is also supported on boards without a display. | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_sdcard) |
| [USB HID Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_usb_hid) | USB HID demo (keyboard, mouse, or gamepad visualization using LVGL) | - |

<!-- END_EXAMPLES -->
</div>

<!-- START_BENCHMARK -->

## LVGL Benchmark

**DATE:** 07.11.2025 09:03

**LVGL version:** 9.4.0

| Name | Avg. CPU | Avg. FPS | Avg. time | render time | flush time |
| ---- | :------: | :------: | :-------: | :---------: | :--------: |
| Empty screen | 61%  | 88  | 5  | 3  | 2  |
| Moving wallpaper | 89%  | 71  | 10  | 7  | 3  |
| Single rectangle | 23%  | 88  | 1  | 1  | 0  |
| Multiple rectangles | 39%  | 90  | 2  | 2  | 0  |
| Multiple RGB images | 31%  | 96  | 2  | 2  | 0  |
| Multiple ARGB images | 58%  | 93  | 6  | 6  | 0  |
| Rotated ARGB images | 87%  | 61  | 14  | 14  | 0  |
| Multiple labels | 92%  | 61  | 12  | 12  | 0  |
| Screen sized text | 99%  | 13  | 70  | 67  | 3  |
| Multiple arcs | 98%  | 47  | 18  | 16  | 2  |
| Containers | 29%  | 87  | 2  | 2  | 0  |
| Containers with overlay | 88%  | 28  | 32  | 30  | 2  |
| Containers with opa | 41%  | 89  | 4  | 4  | 0  |
| Containers with opa_layer | 62%  | 72  | 12  | 12  | 0  |
| Containers with scrolling | 96%  | 29  | 30  | 28  | 2  |
| Widgets demo | 99%  | 17  | 49  | 48  | 1  |
| All scenes avg. | 68%  | 64  | 15  | 15  | 0  |



<!-- END_BENCHMARK -->
