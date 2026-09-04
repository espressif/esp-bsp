# ESP LCD LT9611

[![Component Registry](https://components.espressif.com/components/espressif/esp_lcd_lt9611/badge.svg)](https://components.espressif.com/components/espressif/esp_lcd_lt9611)

Implementation of the Lontium LT9611 MIPI-DSI to HDMI bridge with the `esp_lcd` component.

| LCD controller | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| LT9611         | I2C                     | esp_lcd_lt9611 | [Lontium](https://www.lontiumsemi.com/uploadfile/202606/6994d40a8853a9c2c896158d894c1df8.pdf) |

> [!WARNING]
> On ESP32-P4 (maximum two DSI lanes at 1.5 Gbps per lane), 1920x1080@60 Hz only supports RGB565.
> RGB888 at 1080p60 exceeds the available DSI bandwidth.

Supported configuration macros:

- 1920x1080@60 Hz RGB565: `LT9611_1920x1080_60HZ_RGB16_LANE_BITRATE_MBPS`,
  `LT9611_1920x1080_60HZ_RGB16_DPI_PANEL_CONFIG_WITH_FBS(num_fbs)`
- 1280x720@60 Hz RGB888: `LT9611_1280x720_60HZ_BGR24_LANE_BITRATE_MBPS`,
  `LT9611_1280x720_60HZ_BGR24_DPI_PANEL_CONFIG_WITH_FBS(num_fbs)`

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add this component to your project with:

```text
idf.py add-dependency esp_lcd_lt9611
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).
