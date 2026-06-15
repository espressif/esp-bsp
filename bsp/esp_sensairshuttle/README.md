# BSP: ESP-SensairShuttle

| [HW Reference](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32c5/esp-sensairshuttle/user_guide_v1.0.html) | [HOW TO USE API](API.md) |
| --- | --- |

> This board support package was **generated with the [ESP-BSP Generator](https://bsp-generator.espressif.tools)**. Keep the saved configuration JSON from your run so you can reload it, adjust pins or features, and regenerate when ESP-IDF or BSP templates change — instead of editing generated files by hand.



## Overview

<table>
<tr><td>

ESP-SensairShuttle is a development board jointly launched by Espressif and Bosch Sensortec for motion sensing and large language model human-computer interaction scenarios, dedicated to promoting the deep integration of multimodal sensing and intelligent interaction technologies. The platform covers typical application scenarios such as AI toys, smart homes, sports health, and smart offices, supporting a complete technical chain from environmental sensing, behavior understanding to intelligent feedback, providing a more natural, real-time, and intelligent interaction experience for next-generation intelligent terminals.

</td><td width="200" valign="top">
</td></tr>
</table>

## Capabilities and dependencies

<div align="center">
<!-- START_DEPENDENCIES -->

|     Available    |       Capability       |Controller/Codec|                                                   Component                                                  |   Version  |
|------------------|------------------------|----------------|--------------------------------------------------------------------------------------------------------------|------------|
|:heavy_check_mark:|     :pager: DISPLAY    |     st7789     |                                                      idf                                                     |    >=5.2   |
|:heavy_check_mark:|:black_circle: LVGL_PORT|                |        [espressif/esp_lvgl_port](https://components.espressif.com/components/espressif/esp_lvgl_port)        |     ^2     |
|:heavy_check_mark:|    :point_up: TOUCH    |     cst816s    |[espressif/esp_lcd_touch_cst816s](https://components.espressif.com/components/espressif/esp_lcd_touch_cst816s)|      *     |
|:heavy_check_mark:| :radio_button: BUTTONS |                |               [espressif/button](https://components.espressif.com/components/espressif/button)               |     ^4     |
|        :x:       |   :white_circle: KNOB  |                |                                                                                                              |            |
|:heavy_check_mark:|  :musical_note: AUDIO  |                |        [espressif/esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)        |    ~1.5    |
|:heavy_check_mark:| :speaker: AUDIO_SPEAKER|                |                                                                                                              |            |
|:heavy_check_mark:| :microphone: AUDIO_MIC |      dummy     |                                                                                                              |            |
|        :x:       |  :floppy_disk: SDCARD  |                |                                                                                                              |            |
|:heavy_check_mark:|       :bulb: LED       |                |    idf<br/>[espressif/led_indicator](https://components.espressif.com/components/espressif/led_indicator)    |>=5.2<br/>^2|
|        :x:       |     :camera: CAMERA    |                |                                                                                                              |            |
|        :x:       |      :battery: BAT     |                |                                                                                                              |            |
|:heavy_check_mark:|    :video_game: IMU    |                |                                                                                                              |            |
|        :x:       | :thermometer: HUMITURE |                |                                                                                                              |            |

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
