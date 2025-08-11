# BSP: ESP32-LyraT

| [HW Reference](https://www.espressif.com/en/products/devkits/esp32-lyrat) | [HOW TO USE API](API.md) | [EXAMPLES](#compatible-bsp-examples) | [![Component Registry](https://components.espressif.com/components/espressif/esp32_lyrat/badge.svg)](https://components.espressif.com/components/espressif/esp32_lyrat) | ![maintenance-status](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg) |
| --- | --- | --- | --- | -- |

## Overview

<table>
<tr><td>

The ESP32-LyraT is a hardware platform designed for the dual-core ESP32 audio applications, e.g., Wi-Fi or Bluetooth audio speakers, speech-based remote controllers, connected smart-home appliances with one or more audio functionality, etc.

The ESP32-LyraT is a stereo audio board. If you are looking for a mono audio board, intended for lower end applications, check ESP32-LyraT-Mini.

</td><td width="200">
  <img src="doc/esp32_lyrat.webp">
</td></tr>
</table>

![image](pic.jpg)

## Capabilities and dependencies

<div align="center">
<!-- START_DEPENDENCIES -->

|     Available    |       Capability      |Controller/Codec|                                           Component                                          |  Version  |
|------------------|-----------------------|----------------|----------------------------------------------------------------------------------------------|-----------|
|        :x:       |    :pager: DISPLAY    |                |                                                                                              |           |
|        :x:       |    :point_up: TOUCH   |                |                                                                                              |           |
|:heavy_check_mark:| :radio_button: BUTTONS|                |       [espressif/button](https://components.espressif.com/components/espressif/button)       |     ^4    |
|:heavy_check_mark:|  :musical_note: AUDIO |                |[espressif/esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)|^1.0.3,<1.2|
|:heavy_check_mark:|:speaker: AUDIO_SPEAKER|     es8388     |                                                                                              |           |
|:heavy_check_mark:| :microphone: AUDIO_MIC|     es8388     |                                                                                              |           |
|:heavy_check_mark:|       :bulb: LED      |                |                                              idf                                             |   >=4.4   |
|:heavy_check_mark:|  :floppy_disk: SDCARD |                |                                              idf                                             |   >=4.4   |
|        :x:       |    :video_game: IMU   |                |                                                                                              |           |

<!-- END_DEPENDENCIES -->
</div>

## Compatible BSP Examples

<div align="center">
<!-- START_EXAMPLES -->

| Example | Description | Try with ESP Launchpad |
| ------- | ----------- | ---------------------- |
| [Audio Example](https://github.com/espressif/esp-bsp/tree/master/examples/audio) | Play and record WAV file | [Flash Example](https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=audio-) |

<!-- END_EXAMPLES -->
</div>
