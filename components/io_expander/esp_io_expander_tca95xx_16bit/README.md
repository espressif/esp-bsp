# ESP IO Expander Chip TCA9539 and TCA9555

[![Component Registry](https://components.espressif.com/components/espressif/esp_io_expander_tca95xx_16bit/badge.svg)](https://components.espressif.com/components/espressif/esp_io_expander_tca95xx_16bit)

Implementation of the TCA9539 and TCA9555 io expander chip with esp_io_expander component.

| Chip             | Communication interface | Component name                | Link to datasheet |
| :--------------: | :---------------------: | :---------------------------: | :---------------: |
| TCA9539          | I2C                     | esp_io_expander_tca95xx_16bit | [datasheet](https://www.ti.com/lit/gpn/tca9539) |
| TCA9555          | I2C                     | esp_io_expander_tca95xx_16bit | [datasheet](https://www.ti.com/lit/gpn/tca9555) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependency`, e.g.
```
    idf.py add-dependency esp_io_expander_tca95xx_16bit==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use

Creation of the component.

```
    esp_io_expander_handle_t io_expander = NULL;
    esp_io_expander_new_i2c_tca95xx_16bit(1, ESP_IO_EXPANDER_I2C_TCA9539_ADDRESS_00, &io_expander);
```

Set pin 0 and pin 1 with output dircetion and low level:

```
    esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, 0);
```

Print all pins's status to the log:

```
    esp_io_expander_print_state(io_expander);
```
