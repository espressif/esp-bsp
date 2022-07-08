# ESP LCD GC9A01

Implementation of the GC9A01 LCD controller with esp_lcd component. 

| LCD controller | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| GC9A01         | SPI                     | esp_lcd_gc9a01     | [WIKI](https://www.waveshare.com/wiki/1.28inch_LCD_Module) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g. 
```
    idf.py add-dependency esp_lcd_gc9a01==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use

There is an example in ESP-IDF with this LCD controller. Please follow this [link](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/lcd/spi). 
