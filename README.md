[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://github.com/pre-commit/pre-commit)

# ESP-BSP: Espressif's Board Support Packages
Board support packages for development boards using Espressif's SoCs, written in C.

## Supported boards
| Board name | SoC | Features | Photo |
|---|---|---|---|
| [ESP-WROVER-KIT](esp_wrover_kit) | ESP32 | LCD display, uSD card slot | <img src="docu/pics/wrover.png" width="150"> |
| [ESP-BOX](esp-box) | ESP32-S3 | LCD display with touch, audio codec + power amplifier,<br>accelerometer and gyroscope | <img src="docu/pics/box.webp" width="150"> |
| [ESP32-Azure IoT Kit](esp32_azure_iot_kit) | ESP32 | OLED display, uSD card slot, accelerometer,<br>magnetometer, humidity, pressure, light<br>and temperature sensors | <img src="docu/pics/azure.png" width="150"> |
| [ESP32-S2-Kaluga Kit](esp32_s2_kaluga_kit) | ESP32-S2 | LCD display, audio codec + power amplifier,<br>smart LED and camera | <img src="docu/pics/kaluga.png" width="150">  |
| [ESP32-S3-USB-OTG](esp32_s3_usb_otg) | ESP32-S3 | LCD display, uSD card slot, USB-OTG | <img src="docu/pics/esp32_s3_otg.png" width="150">  |
| [ESP32-S3-EYE](esp32_s3_eye) | ESP32-S3 | LCD display, camera, uSD card slot, microphone and accelerometer | <img src="docu/pics/s3-eye.webp" width="150"> |
| [ESP32-S3-LCD-EV-BOARD](esp32_s3_lcd_ev_board) | ESP32-S3 | LCD display with touch, audio codec + power amplifier | <img src="docu/pics/esp32-s3-lcd-ev-board_800x480.png" width="150"> |

## LCD and TOUCH drivers

The BSP repository includes lot of LCD and Touch driver components. The list of available and planned LCDs is [here](LCD.md).

## How to use

### Examples

Best way to start with ESP-BSP is trying one of the [examples](examples) on your board. Every example contains `README.md` with a list of supported boards. Here is the summary of the available examples:

| Example | Supported boards |
|---|---|
| [display](examples/display) | WROVER-KIT |
| [display_audio](examples/display_audio) | Kaluga-kit |
| [display_camera](examples/display_camera) | Kaluga-kit |
| [display_audio_photo](examples/display_audio_photo) | ESP-BOX |
| [display_rotation](examples/display_rotation) | ESP-BOX |
| [display_lvgl_demos](examples/display_lvgl_demos) | ESP32-S3-LCD-EV-BOARD |
| [touchscreen_colorwheel](examples/touchscreen_colorwheel) | ESP-BOX |
| [mqtt_example](examples/mqtt_example) | Azure-IoT-kit |
| [sensors_example](examples/sensors_example) | Azure-IoT-kit |

### BSP headers
Each BSP provides its header file in 'bsp' subfolder, so it can be included as follows: `#include "bsp/name-of-the-bsp.h"`.

For you convenience, each BSP also provides a wrapper header, which has the same name for all BSPs: `#include "bsp/esp-bsp.h"`.

> **_NOTE:_** There can be only one BSP in a single esp-idf project.

### In a custom project
Packages from this repository are uploaded to the [IDF component registry](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependency`, e.g.
```
    idf.py add-dependency esp_wrover_kit==1.0.0
```

Alternatively, you can create `idf_component.yml` file manually, such as in [this example](examples/display/main/idf_component.yml).

### Recommendation for custom projects

When you want to use a BSP in a real project, it is highly recommended to disable configuration option `CONFIG_BSP_ERROR_CHECK` in menuconfig. You should check all returned error codes from all BSP functions you call. Otherwise, if the option `CONFIG_BSP_ERROR_CHECK` is enabled, any error encountered in a BSP will abort the program.

## Migration to ESP-IDF v5.0
ESP-IDF v5.0 brings a lot of new features, but, as the bump in major version suggests, also a lot of breaking changes.

ESP-BSP is kept up-to-date with the latest ESP-IDF version, but some breaking changes in ESP-BSP API are inevitable.
Usually, BSPs compatible with IDF v5.0 are version 2. If you want to use BSP with IDF v4.4 you can still use version 1 of the particular BSP.

More information about ESP-IDF breaking changes can be found in the [official migration guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/migration-guides/release-5.x/index.html).

## Additional information
More information about idf-component-manager can be found in [Espressif API guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html)
or [PyPi registry](https://pypi.org/project/idf-component-manager/).

## Contributing to ESP-BSP

Please check [CONTRIBUTING.md](CONTRIBUTING.md) if you'd like to contribute to ESP-BSP.

## Copyrights and License

All original source code in this repository is Copyright (c) Espressif Systems (Shanghai) Co. Ltd., and is licensed under the Apache 2.0 license.
