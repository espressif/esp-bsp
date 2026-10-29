# ESP IO Expander Chip M5IOE1

[![Component Registry](https://components.espressif.com/components/espressif/esp_io_expander_m5ioe1/badge.svg)](https://components.espressif.com/components/espressif/esp_io_expander_m5ioe1)

Implementation of the M5IOE1 io expander chip with esp_io_expander component.

| Chip             | Communication interface | Component name | Link to datasheet |
| :--------------: | :---------------------: | :------------: | :---------------: |
| M5IOE1           | I2C                     | esp_io_expander_m5ioe1 | [datasheet](https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1210/IO_Expander_Datasheet_EN.pdf) |

> [!NOTE]
> The chip names its pins IO1..IO14. `IO_EXPANDER_PIN_NUM_0` is IO1 and `IO_EXPANDER_PIN_NUM_13` is IO14.
>
> Initialization puts every pin in input mode, push-pull, with pull-up and pull-down disabled. Use `esp_io_expander_set_output_mode()` for open-drain.
>
> ADC, PWM, NeoPixel, the temperature sensor and GPIO interrupts are not implemented. The driver talks to the chip at 100 kHz, which is the M5IOE1 power-on I2C speed. Valid 7-bit addresses are 0x6F..0x76 (IO7 strap) and 0x4F/0x50 on some M5Stack products.

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependency`, e.g.
```
    idf.py add-dependency esp_io_expander_m5ioe1==1.0.0
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
    esp_io_expander_new_i2c_m5ioe1(i2c_handle, ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_6F, &io_expander);
```

Print all pins's status to the log:

```c
    esp_io_expander_print_state(io_expander);
```

Set pin IO1 and IO2 as outputs, low level:

```c
    esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(io_expander, IO_EXPANDER_PIN_NUM_0 | IO_EXPANDER_PIN_NUM_1, 0);
```

Set pin IO3 and IO4 as inputs with pull-up:

```c
    uint32_t pin_levels = 0;
    esp_io_expander_set_dir(io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, IO_EXPANDER_INPUT);
    esp_io_expander_set_pullupdown(io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, IO_EXPANDER_PULL_UP);
    esp_io_expander_get_level(io_expander, IO_EXPANDER_PIN_NUM_2 | IO_EXPANDER_PIN_NUM_3, &pin_levels);
```

## Example use with GPIO API

1. Enable `CONFIG_IO_EXPANDER_ENABLE_GPIO_API_WRAPPER=y` in `menuconfig` (enabled by default)
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
    esp_io_expander_new_i2c_m5ioe1(i2c_handle, ESP_IO_EXPANDER_I2C_M5IOE1_ADDRESS_6F, &io_expander);
```

5. Append IO Expander to GPIO wrapper.

```c
esp_io_expander_gpio_wrapper_append_handler(io_expander, GPIO_NUM_MAX);
```

6. Use it as standard GPIO.

```c
const uint8_t io_led1 = GPIO_NUM_MAX + 0; // M5IOE1 IO1
gpio_set_direction(io_led1, GPIO_MODE_OUTPUT);

ESP_LOGI(TAG, "LED1 set ON");
gpio_set_level(io_led1, 1);

ESP_LOGI(TAG, "LED1 set OFF");
gpio_set_level(io_led1, 0);
```
