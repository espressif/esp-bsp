# ESP IO Expander Component

[![Component Registry](https://components.espressif.com/components/espressif/esp_io_expander/badge.svg)](https://components.espressif.com/components/espressif/esp_io_expander)

This component is main esp_io_expander component which defines main functions and types for easy adding specific IO expander chip component.

## Supported features

### From v1.0

- Set an IO's direction
- Get an IO's direction
- Set an IO's output level
- Get an IO's input level
- Show all IOs' status

### From v1.1

- Set an IO's output mode (Push Pull / Open Drain)
- Set an IO's Pull-up / Pull-down state

### From v1.2

- GPIO API wrapper to control IO expander pins via ESP-IDF `gpio_*` APIs

## GPIO API wrapper

When enabled, IO expander pins can be used like regular GPIOs through ESP-IDF's `gpio_*` APIs. This allows existing GPIO-based code to work with IO expanders with minimal changes.

- Enable in `menuconfig`: `IO_EXPANDER_ENABLE_GPIO_API_WRAPPER` (under `ESP IO Expander`).
- Append a handler with a virtual GPIO start index using `esp_io_expander_gpio_wrapper_append_handler(handler, start_io_num)`. The `start_io_num` must be greater than or equal to `GPIO_NUM_MAX`. The handler's `io_count` pins will be mapped to the range `[start_io_num, start_io_num + io_count)`.
- Remove a previously appended mapping with `esp_io_expander_gpio_wrapper_remove_handler(handler)`.

Supported wrapped APIs:
- `gpio_set_level`
- `gpio_get_level`
- `gpio_set_direction` (supports `GPIO_MODE_INPUT`, `GPIO_MODE_OUTPUT`, `GPIO_MODE_OUTPUT_OD` if the chip supports high-Z write)
- `gpio_set_pull_mode` (supports `GPIO_PULLUP_ONLY`, `GPIO_PULLDOWN_ONLY` if the chip supports pulldown select, and `GPIO_FLOATING`)

Example:

```c
// 1) Enable IO_EXPANDER_ENABLE_GPIO_API_WRAPPER in menuconfig
// 2) Create the IO expander handler via esp_io_expander_new_i2c_xxx(..., &ioexp)
esp_io_expander_handle_t ioexp;
ESP_ERROR_CHECK(esp_io_expander_new_i2c_xxx(/* i2c_port, i2c_addr, optional cfg */, &ioexp));
// 3) Initialize your IO expander and obtain `esp_io_expander_handle_t ioexp`
ESP_ERROR_CHECK(esp_io_expander_gpio_wrapper_append_handler(ioexp, GPIO_NUM_MAX));

gpio_num_t vgpio0 = (gpio_num_t)(GPIO_NUM_MAX + 0);
gpio_set_direction(vgpio0, GPIO_MODE_OUTPUT);
gpio_set_level(vgpio0, 1);

// When no longer needed:
ESP_ERROR_CHECK(esp_io_expander_gpio_wrapper_remove_handler(ioexp));
```

### Future

- Interrupt mode



