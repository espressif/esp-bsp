# ESP LCD Touch CST820 Controller

[![Component Registry](https://components.espressif.com/components/espressif/esp_lcd_touch_cst820/badge.svg)](https://components.espressif.com/components/espressif/esp_lcd_touch_cst820)

ESP-IDF driver for CST820 capacitive touch controllers. It uses the common
`esp_lcd_touch` API and communicates through I2C.

| Touch controller | Communication interface | Component name | Default address | Maximum points | Datasheet |
| :--------------: | :---------------------: | :------------: | :-------------: | :------------: | :-------: |
| CST820 | I2C | `esp_lcd_touch_cst820` | `0x15` | 2 | [datasheet](https://raw.githubusercontent.com/kodediy/esp_lcd_touch_cst820/main/CST820_Datasheet_V1.2.pdf) |

> [!NOTE]
> * The public CST820 datasheet describes the electrical interface but does not
>   publish the complete touch report register map. This driver follows the
>   15-byte report supplied with the display module: count in byte 2 and
>   coordinate slots beginning at bytes 3 and 9. It does not probe
>   CST816-family addresses, alias CST816 registers, or fall back to a CST816
>   compatibility path. Driver initialization is limited to reset and an
>   optional ID read; it does not change auto-sleep or interrupt-mode registers
>   and sends no command writes.
> * If the controller firmware does not expose register `0xA7`, enable
>   `CONFIG_ESP_LCD_TOUCH_CST820_DISABLE_READ_ID`. Touch data can still be read
>   after the controller returns to dynamic mode on a touch event.

## Add to project

Once the component is published to the ESP Component Registry, add it to an
ESP-IDF project with:

```sh
idf.py add-dependency "espressif/esp_lcd_touch_cst820^1.2.0"
```

This command is available after registry publication. Until then, reference a
local checkout with `override_path` in `idf_component.yml` instead. After
publication, the Component Manager writes the dependency to
`main/idf_component.yml` and downloads it during the next configure or build.

## Initialize the controller

Create the I2C bus and panel IO first. The example below uses the current
ESP-IDF I2C master driver:

```c
#include "driver/i2c_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_cst820.h"

i2c_master_bus_handle_t i2c_bus = NULL;
esp_lcd_panel_io_handle_t touch_io = NULL;
esp_lcd_touch_handle_t touch = NULL;

const i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_8,
    .scl_io_num = GPIO_NUM_18,
    .clk_source = I2C_CLK_SRC_DEFAULT,
};
ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

const esp_lcd_panel_io_i2c_config_t io_config =
    ESP_LCD_TOUCH_IO_I2C_CST820_CONFIG();
ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &touch_io));

const esp_lcd_touch_config_t touch_config = {
    .x_max = 460,
    .y_max = 460,
    .rst_gpio_num = GPIO_NUM_NC,
    .int_gpio_num = GPIO_NUM_NC,
    .levels = {
        .reset = 0,
        .interrupt = 0,
    },
    .flags = {
        .swap_xy = 0,
        .mirror_x = 0,
        .mirror_y = 0,
    },
};
ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_cst820(touch_io, &touch_config, &touch));
```

GPIO numbers in the example are placeholders. Use the reset, interrupt, SDA,
and SCL pins from the target board schematic. Set either reset or interrupt to
`GPIO_NUM_NC` when that signal is not connected.

The test application exposes its own wiring and coordinate range through
`menuconfig`; the defaults are generic development-board values, not a product
board configuration.

The reset and interrupt active levels are provided by the board through
`touch_config.levels`; the driver does not assume a fixed module wiring.

## Read touch points

Call `esp_lcd_touch_read_data()` after an interrupt or from a periodic task,
then fetch the latest report:

```c
esp_lcd_touch_point_data_t points[2];
uint8_t point_count = 0;

ESP_ERROR_CHECK(esp_lcd_touch_read_data(touch));
ESP_ERROR_CHECK(esp_lcd_touch_get_data(touch, points, &point_count, 2));

for (uint8_t i = 0; i < point_count; i++) {
    ESP_LOGI("touch", "id=%u x=%u y=%u",
             points[i].track_id, points[i].x, points[i].y);
}
```

Set `CONFIG_ESP_LCD_TOUCH_MAX_POINTS` to at least `2` to retain both points.
The controller report provides coordinates and a tracking ID, but no documented
pressure value, so `strength` is returned as zero.

Polling is useful during early board bring-up because it does not depend on the
controller firmware's interrupt mode. Once the interrupt polarity has been
verified on hardware, an ISR callback can wake a task that performs the I2C
read. Do not access I2C directly from the ISR.

## Power management

The CST820 has three power modes (DS_CST_820 V1.2): dynamic (~1.9 mA),
standby (~10 uA) and deep sleep (~2 uA). The datasheet states that sleep mode
is entered through a "sleep command" but does not publish the register map;
commands reconstructed from public CST816-family sources (`0xA5 <- 0x03` and
`0xE5 <- 0x03`) were verified on hardware to leave the controller answering
I2C unchanged, so this driver provides no deep-sleep API. Monitor (standby)
mode is the supported low-power tier.

### Monitor / standby mode (`esp_lcd_touch_cst820_enter_monitor_mode()` /
`esp_lcd_touch_cst820_exit_monitor_mode()`)

Standby is the tier for "wake the product by touching the screen": the
controller keeps scanning at a low frequency and, on a touch or a
predefined standby gesture, returns to dynamic mode on its own and pulses
INT to wake the host. Per the datasheet the controller enters standby
automatically when no touch is detected for 2 s, so no undocumented
register write is needed and the driver sends none.

Typical low-power sequence (verified on hardware, INT active low):

```c
/* 1. Arm monitor mode; the controller reaches standby <= 2 s after the
 *    last touch. */
ESP_ERROR_CHECK(esp_lcd_touch_cst820_enter_monitor_mode(touch));

/* 2. Arm the INT pin as the SoC wake source (light sleep example). */
ESP_ERROR_CHECK(gpio_wakeup_enable(GPIO_NUM_3, GPIO_INTR_LOW_LEVEL));
ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

/* 3. Sleep; resumes when the panel is touched. */
esp_light_sleep_start();

/* 4. Read the wake-up report, then optionally force dynamic mode back
 *    without waiting for another touch. */
ESP_ERROR_CHECK(esp_lcd_touch_read_data(touch));
ESP_ERROR_CHECK(esp_lcd_touch_cst820_exit_monitor_mode(touch));
```

Notes:

- Allow up to 2 s after the last touch before the controller is actually in
  standby (datasheet auto-standby timeout).
- While the host is in light sleep, the controller exits standby by itself
  when touched; `esp_lcd_touch_cst820_exit_monitor_mode()` is only needed
  to force dynamic mode without a touch. It requires the reset GPIO.
- If EVT verifies a forced-standby command for this firmware, the enter
  function may start sending it; the API contract stays unchanged.

## Register access and undocumented features

The public CST820 datasheet (DS_CST_820 V1.2) describes the electrical
interface and power modes but does not publish the register map or the full
touch-report byte layout. As a result, the following datasheet feature blocks
have no structured driver API:

- Single-tap and standby gesture reports (the module report carries two
  leading bytes whose layout is not publicly documented)
- Auto-standby enable/timeout control (the datasheet states it is
  register-controlled but does not publish the register)
- Multi-key reports (the base-class `get_button_state` hook is not
  implemented because the report layout for keys is not publicly documented)

For these, use the raw register accessors, which are thin wrappers over the
panel IO channel:

```c
uint8_t buf[4] = {0};
ESP_ERROR_CHECK(esp_lcd_touch_cst820_read_reg(touch, 0x00, buf, sizeof(buf)));
ESP_ERROR_CHECK(esp_lcd_touch_cst820_write_reg(touch, 0x00, 0x00));
```

Validate any register address against your controller firmware before
shipping; addresses may differ between module firmware variants.

## Release resources

Delete resources in the reverse order in which they were created:

```c
ESP_ERROR_CHECK(esp_lcd_touch_del(touch));
ESP_ERROR_CHECK(esp_lcd_panel_io_del(touch_io));
ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus));
```
