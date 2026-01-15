# ESP IO Expander Chip TCA9554(A)

[![Component Registry](https://components.espressif.com/components/espressif/esp_io_expander_tca9554/badge.svg)](https://components.espressif.com/components/espressif/esp_io_expander_tca9554)

Implementation of the TCA9554 io expander chip with esp_io_expander component.

| Chip             | Communication interface | Component name | Link to datasheet |
| :--------------: | :---------------------: | :------------: | :---------------: |
| TCA9554(A)       | I2C                     | esp_io_expander_tca9554 | [datasheet](https://www.ti.com/lit/gpn/tca9554) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependency`, e.g.
```
    idf.py add-dependency esp_io_expander_tca9554==2.0.3
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
    esp_io_expander_new_i2c_tca9554(i2c_handle, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &io_expander);
```

Print all pins's status to the log:

```c
    esp_io_expander_print_state(io_expander);
```

Set pin 0 and pin 1 with output direction and low level:

```c
    esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, 0);
```

Set pin 2 and pin 3 with input direction:

```c
    uint32_t pin_levels = 0;
    esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, IO_EXPANDER_INPUT);
    esp_io_expander_get_level(io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, &pin_levels);
```

## Example use with GPIO API

1. Enable `CONFIG_IO_EXPANDER_ENABLE_GPIO_API_WRAPPER=y` in `menuconfig`
2. Include `#include "esp_io_expander_gpio_wrapper.h"` to your project.

3. Creation of the i2c bus.

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

4. Creation of the component.

```c
    esp_io_expander_handle_t io_expander = NULL;
    esp_io_expander_new_i2c_tca9554(i2c_handle, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000, &io_expander);
```

5. Append IO Expander to GPIO wrapper.

```c
esp_io_expander_gpio_wrapper_append_handler(io_expander, GPIO_NUM_MAX);
```

6. Use it as standard GPIO.

```c
const uint8_t io_led1 = GPIO_NUM_MAX + 6; // IO Expander pin 6
gpio_set_direction(io_led1, GPIO_MODE_OUTPUT);

ESP_LOGI(TAG, "LED1 set ON");
gpio_set_level(io_led1, 1);

ESP_LOGI(TAG, "LED1 set OFF");
gpio_set_level(io_led1, 0);
```
