# BSP: M5Stack Core2

| [HW Reference](https://docs.m5stack.com/en/core/Core2) | [HOW TO USE API](API.md) | [EXAMPLES](#compatible-bsp-examples) | [![Component Registry](https://components.espressif.com/components/espressif/m5stack_core_2/badge.svg)](https://components.espressif.com/components/espressif/m5stack_core_2) | ![maintenance-status](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg) |
| --- | --- | --- | --- | -- |

> [!WARNING]
> The SD card is not working simultaneously with the LCD screen. We are working on a fix.

## Overview

<table>
<tr><td>

M5Core2 is the second generation core device in the M5Stack development kit series, which further enhances the functions of the original generation of cores. It is a powerful and user-friendly development board with the following features:

- **MCU**: Equipped with an ESP32-D0WDQ6-V3, featuring dual core Xtensa® 32-bit 240Mhz LX6 processors that can be controlled separately.
- **Memory**: Onboard 16MB Flash and 8MB PSRAM.
- **Interface**: USB TYPE-C interface for charging, program downloading, and serial communication.
- **Display**: 2.0-inch integrated capacitive touch screen with three programmable capacitive buttons on the front.
- **Power Management:** Managed by an AXP192 power management chip (upgraded to AXP2101 in Core2 V1.1), which effectively controls power consumption. It includes a built-in green LED power indicator for battery level notification. The battery capacity is 390mAh, providing longer power duration than the previous model.
  - For Core2 V1.0: Configure `CONFIG_BSP_PMU_AXP192`
  - For Core2 V1.1: Configure `CONFIG_BSP_PMU_AXP2101`
- **Audio**: I2S digital audio interface power amplifier chip to prevent signal distortion, along with a built-in speaker.
- **Expansion**: Retains a TF-card (microSD) slot, and an expansion board on the back with a 6-axis IMU sensor and a microphone.
- **RTC Module**: Built-in RTC module for accurate timing, with a dedicated battery for RTC power supply in Core2 V1.1.
Additional Features: Built-in vibration motor for haptic feedback, independent power, and reset buttons on the left side and bottom of the base.

</td><td width="200" valign="top">
  <img src="doc/m5stack_core_2.webp">
</td></tr>
</table>

<p align="center">
<img src="https://static-cdn.m5stack.com/resource/docs/products/core/core2/core2_01.webp" alt="basic" width="350" height="350"><img src="https://static-cdn.m5stack.com/resource/docs/products/core/Core2%20v1.1/img-1a949091-da2c-4fbb-bf4f-bce108cb43ec.webp" alt="gray" width="350" height="350">
</p>

#### Core2 V1.1 Enhancements
Core2 V1.1 is an iterative version of Core2 with the following upgrades and additional features:

- **Power Management**: Uses the AXP2101 power management chip for enhanced power control.
- **Indicators**: Built-in blue power indicator light for specific functions or status indications.
- RTC Battery: Dedicated battery for RTC power supply for accurate timing.
- **User Interaction**: Enhanced touch screen experience with programmable virtual buttons for diverse human-machine interaction.

## Capabilities and dependencies

<div align="center">
<!-- START_DEPENDENCIES -->

|     Available    |       Capability       |Controller/Codec|                                                  Component                                                 |     Version    |
|------------------|------------------------|----------------|------------------------------------------------------------------------------------------------------------|----------------|
|:heavy_check_mark:|     :pager: DISPLAY    |     ili9341    | [espressif/esp_lcd_ili9341](https://components.espressif.com/components/espressif/esp_lcd_ili9341)<br/>idf |^2.0.1<br/>>=5.0|
|:heavy_check_mark:|:black_circle: LVGL_PORT|                |       [espressif/esp_lvgl_port](https://components.espressif.com/components/espressif/esp_lvgl_port)       |       ^2       |
|:heavy_check_mark:|    :point_up: TOUCH    |     ft5x06     |[espressif/esp_lcd_touch_ft5x06](https://components.espressif.com/components/espressif/esp_lcd_touch_ft5x06)|       ^1       |
|        :x:       | :radio_button: BUTTONS |                |                                                                                                            |                |
|:heavy_check_mark:|  :musical_note: AUDIO  |                |       [espressif/esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)       |      ~1.1      |
|:heavy_check_mark:| :speaker: AUDIO_SPEAKER|                |                                                                                                            |                |
|        :x:       | :microphone: AUDIO_MIC |                |                                                                                                            |                |
|:heavy_check_mark:|  :floppy_disk: SDCARD  |                |                                                     idf                                                    |      >=5.0     |
|        :x:       |    :video_game: IMU    |                |                                                                                                            |                |

<!-- END_DEPENDENCIES -->
</div>

## Compatible BSP Examples

<div align="center">
<!-- START_EXAMPLES -->

| Example | Description | Try with ESP Launchpad |
| ------- | ----------- | ---------------------- |
| [Display Example](https://github.com/espressif/esp-bsp/tree/master/examples/display) | Show an image on the screen with a simple startup animation (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display-) |
| [Display, Audio and Photo Example](https://github.com/espressif/esp-bsp/tree/master/examples/display_audio_photo) | Complex demo: browse files from filesystem and play/display JPEG, WAV, or TXT files (LVGL) | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_audio_photo-) |

<!-- END_EXAMPLES -->
</div>

<!-- START_BENCHMARK -->
<!-- END_BENCHMARK -->
