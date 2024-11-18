# ESP LCD LT8912B

[![Component Registry](https://components.espressif.com/components/espressif/esp_lcd_lt8912b/badge.svg)](https://components.espressif.com/components/espressif/esp_lcd_lt8912b)

Implementation of the LT8912B MIPI-DSI to HDMI bridge with esp_lcd component.

| LCD controller | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| LT8912B        | I2C                     | esp_lcd_lt8912b | [Specification](http://www.lontiumsemi.com/UploadFiles/2022-03/LT8912B_Brief_R1.3.pdf) |

> [!WARNING]
> This controller suports only RGB888 color mode.

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependency`, e.g.

```text
idf.py add-dependency esp_lcd_lt8912b
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

