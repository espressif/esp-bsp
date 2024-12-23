# BSP: ESP32-P4 Function EV Board

[![Component Registry](https://components.espressif.com/components/espressif/esp32_p4_function_ev_board/badge.svg)](https://components.espressif.com/components/espressif/esp32_p4_function_ev_board)

ESP32-P4 Function EV Board is internal Espressif board for testing features on ESP32P4 chip.

| HW version | BSP Version |
| :--------: | :---------: |
|    V1.0    |      ^1     |
|    V1.2    |      ^2     |
|    [V1.4](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32p4/esp32-p4-function-ev-board/user_guide.html)    |      ^3     |

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

<!-- Autogenerated start: Dependencies -->
### Capabilities and dependencies
|  Capability |     Available    |                                                 Component                                                |Version|
|-------------|------------------|----------------------------------------------------------------------------------------------------------|-------|
|   DISPLAY   |:heavy_check_mark:|    [espressif/esp_lcd_ek79007](https://components.espressif.com/components/espressif/esp_lcd_ek79007)    |  1.*  |
|  LVGL_PORT  |:heavy_check_mark:|      [espressif/esp_lvgl_port](https://components.espressif.com/components/espressif/esp_lvgl_port)      |   ^2  |
|    TOUCH    |:heavy_check_mark:|[espressif/esp_lcd_touch_gt911](https://components.espressif.com/components/espressif/esp_lcd_touch_gt911)|   ^1  |
|   BUTTONS   |        :x:       |                                                                                                          |       |
|    AUDIO    |:heavy_check_mark:|      [espressif/esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)      | 1.2.* |
|AUDIO_SPEAKER|:heavy_check_mark:|                                                                                                          |       |
|  AUDIO_MIC  |:heavy_check_mark:|                                                                                                          |       |
|    SDCARD   |:heavy_check_mark:|                                                    idf                                                   | >=5.3 |
|     IMU     |        :x:       |                                                                                                          |       |
<!-- Autogenerated end: Dependencies -->
