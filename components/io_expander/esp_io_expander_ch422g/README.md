# ESP IO Expander Chip CH422G

Implementation of the CH422G io expander chip with esp_io_expander component.

| Chip             | Communication interface | Component name | Link to datasheet |
| :--------------: | :---------------------: | :------------: | :---------------: |
| CH422G           | I2C                     | esp_io_expander_ch422g | [datasheet](https://files.waveshare.com/wiki/common/CH422DS1_EN.pdf) |

Currently only support bidirectional I/O pins. General purposes pins OC3-OC0 are not supported.
Chip limitations dictates that all bidirectional I/O pins are either input or output, they cannot be controlled individually.

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependency`, e.g.
```
    idf.py add-dependency esp_io_expander_ch422g==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use

Creation of the i2c bus.

```c
    i2c_master_bus_handle_t i2c_handle = NULL;
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 47,
        .scl_io_num = 48,
        .clk_source = I2C_CLK_SRC_DEFAULT,
    };
    i2c_new_master_bus(&bus_config, &i2c_handle);
```

Creation of the component.

```c
    esp_io_expander_handle_t io_expander = NULL;
    esp_io_expander_new_i2c_ch422g(i2c_handle, &io_expander);
```

Print all pins's status to the log:

```c
    esp_io_expander_print_state(io_expander);
```

Set pin 0 and pin 1 with output direction and low level:

```c
    esp_io_expander_set_dir(io_expander, IO_EXPANDER_ALL_PINS, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, 0);
```

Set pin 2 and pin 3 with input direction:

```c
    uint32_t pin_levels = 0;
    esp_io_expander_set_dir(io_expander, IO_EXPANDER_ALL_PINS, IO_EXPANDER_INPUT);
    esp_io_expander_get_level(io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, &pin_levels);
```
