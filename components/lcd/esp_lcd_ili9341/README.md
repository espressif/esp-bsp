# ESP LCD ILI9341

Implementation of the ILI9341 LCD controller with esp_lcd component. 

| LCD controller | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| ILI9341        | SPI                     | esp_lcd_ili9341     | [Specification](https://cdn-shop.adafruit.com/datasheets/ILI9341.pdf) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g. 
```
    idf.py add-dependency esp_lcd_ili9341==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use

There is an example in ESP-IDF with this LCD controller. Please follow this [link](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/lcd/spi). 
