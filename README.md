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

## How to use

### Examples

Best way to start with ESP-BSP is trying one of the [examples](examples) on your board. Every example contains `README.md` with list of supported boards. Here is a examples' summary:

| Example | Supported boards |
|---|---|
| [display](examples/display) | WROVER-KIT |
| [display_audio](examples/display_audio) | Kaluga-kit |
| [mqtt_example](examples/mqtt_example) | Azure-IoT-kit |
| [rainmaker_example](examples/rainmaker_example) | Azure-IoT-kit |
| [sensors_example](examples/sensors_example) | Azure-IoT-kit |

### In custom project 

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g. 
```
    idf.py add-dependency esp_wrover_kit==1.0.0
```

Alternatively, you can create `idf_component.yml` file manually, such as in [this example](examples/display/main/idf_component.yml).

## Additional information
More information about idf-component-manager can be found in [Espressif API guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html)
or [PyPi registry](https://pypi.org/project/idf-component-manager/).
