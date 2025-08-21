# BSP: M5Stack Core

| [HW Reference](https://docs.m5stack.com/en/core/basic_v2.7) | [HOW TO USE API](API.md) | [EXAMPLES](#compatible-bsp-examples) | [![Component Registry](https://components.espressif.com/components/espressif/m5stack_core/badge.svg)](https://components.espressif.com/components/espressif/m5stack_core) | ![maintenance-status](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg) |
| --- | --- | --- | --- | -- |

> [!WARNING]
> The SD card is not working simultaneously with the LCD screen. We are working on a fix.

## Overview

<table>
<tr><td>

This BSP supports the following M5Stack Core v2.7 series devices:
- M5Stack Basic v2.7
- M5Stack Fire v2.7
- Other M5Stack Core based devices

M5Stack Core is a development board based on ESP32 microcontroller. It features:

- **MCU**: ESP32 dual-core Xtensa® 32-bit LX6 microprocessor, up to 240MHz
- **Memory**: 4MB Flash + 520KB SRAM
- **Interface**: USB Type-C for programming and debugging
- **Display**: 2.0" TFT LCD screen (320x240 resolution) with ILI9341 controller
  - If you have a v2.2 or above, you can enable the IPS display by setting `CONFIG_BSP_M5STACK_CORE_LCD_INVERT_COLOR=y` in the SDK configuration. Vice versa.
- **Power Management**: IP5306 power management chip for battery charging and power control
- **Audio**: Single GPIO controlled speaker
- **Storage**: microSD card slot (SPI interface)
- **Input**: Three physical buttons (ButtonA, ButtonB, ButtonC)
- **Expansion**: Bottom expansion headers for additional modules

</td><td width="200" valign="top">
  <img src="doc/m5stack_core.webp">
</td></tr>
</table>

<p align="center">
<img src="https://static-cdn.m5stack.com/resource/docs/products/core/basic_v2.7/basic_v2.7_01.webp" alt="basic" width="350" height="350">
<img src="https://static-cdn.m5stack.com/resource/docs/products/core/fire_v2.7/fire_v2.6_01.webp" alt="fire" width="350" height="350">
</p>

## Capabilities and dependencies

<div align="center">
<!-- START_DEPENDENCIES -->

|     Available    |       Capability       |Controller/Codec|                                                 Component                                                |     Version    |
|------------------|------------------------|----------------|----------------------------------------------------------------------------------------------------------|----------------|
|:heavy_check_mark:|     :pager: DISPLAY    |     ili9341    |[espressif/esp_lcd_ili9341](https://components.espressif.com/components/espressif/esp_lcd_ili9341)<br/>idf|^2.0.1<br/>>=5.2|
|:heavy_check_mark:|:black_circle: LVGL_PORT|                |      [espressif/esp_lvgl_port](https://components.espressif.com/components/espressif/esp_lvgl_port)      |       ^2       |
|        :x:       |    :point_up: TOUCH    |                |                                                                                                          |                |
|:heavy_check_mark:| :radio_button: BUTTONS |                |             [espressif/button](https://components.espressif.com/components/espressif/button)             |       ^4       |
|        :x:       |  :musical_note: AUDIO  |                |                                                                                                          |                |
|:heavy_check_mark:| :speaker: AUDIO_SPEAKER|                |                                                                                                          |                |
|        :x:       | :microphone: AUDIO_MIC |                |                                                                                                          |                |
|:heavy_check_mark:|  :floppy_disk: SDCARD  |                |                                                    idf                                                   |      >=5.2     |
|        :x:       |    :video_game: IMU    |                |                                                                                                          |                |

<!-- END_DEPENDENCIES -->
</div>

## Compatible BSP Examples

<div align="center">
<!-- START_EXAMPLES -->

| Example | Description | Try with ESP Launchpad |
| ------- | ----------- | ---------------------- |
| [Display Example](https://github.com/espressif/esp-bsp/tree/master/examples/display) | Show an image on the screen with a simple startup animation (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display-) |
| [Display Rotation Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_rotation) | Rotate screen using buttons or an accelerometer (`BSP_CAPS_IMU`, if available) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_rotation-) |

<!-- END_EXAMPLES -->
</div>

<!-- START_BENCHMARK -->
<!-- END_BENCHMARK -->
