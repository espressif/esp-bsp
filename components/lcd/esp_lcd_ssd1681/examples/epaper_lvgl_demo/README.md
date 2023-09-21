# e-Paper Example using LVGL

[esp_lcd](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd.html) allows user to add their own panel drivers in the project scope (i.e. panel driver can live outside of esp-idf), so that the upper layer code like LVGL porting code can be reused without any modifications, as long as user-implemented panel driver follows the interface defined in the `esp_lcd` component.

This example shows how to use SSD1681 e-paper display driver from Component manager in esp-idf project. This example will draw a clock widget using the LVGL library. 

## How to use the example

### Hardware Required

* An ESP development board
* An SSD1681 e-paper panel, with SPI interface
* An USB cable for power supply and programming

### Hardware Connection

The connection between ESP Board and the LCD is as follows:

```
       ESP Board                              SSD1681 e-Paper Panel
┌──────────────────────────────┐              ┌────────────────────┐
│      GND                     ├─────────────►│ GND                │
│                              │              │                    │
│      3V3                     ├─────────────►│ VCC                │
│                              │              │                    │
│      EXAMPLE_PIN_NUM_SCLK    ├─────────────►│ CLK                │
│                              │              │                    │
│      EXAMPLE_PIN_NUM_MOSI    ├─────────────►│ DIN                │
│                              │              │                    │
│      EXAMPLE_PIN_NUM_EPD_RST ├─────────────►│ RST                │
│                              │              │                    │
│      EXAMPLE_PIN_NUM_EPD_DC  ├─────────────►│ DC                 │
│                              │              │                    │
│      EXAMPLE_PIN_NUM_EPD_CS  ├─────────────►│ CS                 │
│                              │              │                    │
│      EXAMPLE_PIN_NUM_EPD_BUSY│◄─────────────│ BUSY               │
└──────────────────────────────┘              └────────────────────┘
```

The GPIO number used by this example can be changed in [main.c](main/main.c).

### Software Configuration

- Change all the `EXAMPLE_PIN` macro definition according to your hardware connection.
- If you are not using waveshare 1.54 inch V2 e-paper panel, please use the waveform lut provided by your panel vendor instead of using the demo built-in ones, or just simply comment the `epaper_panel_set_custom_lut` call and use the panel built-in waveform lut.
- You could go to `menuconfig / Component config / LVGL configuration / Feature configuration / Others` and unselect `Show CPU usage and FPS count` to hide the CPU usage and FPS count window. 

### Build and Flash

Run `idf.py -p PORT build flash monitor` to build, flash and monitor the project. A clock widget will show up on the e-paper as expected.

The first time you run `idf.py` for the example will cost extra time as the build system needs to address the component dependencies and downloads the missing components from registry into `managed_components` folder.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

### Example Output

```bash
...
I (338) example: Initialize SPI bus
I (348) example: Install panel IO
I (348) gpio: GPIO[9]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (358) gpio: GPIO[4]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (368) lcd_panel.epaper: Add handler for GPIO 18
I (378) gpio: GPIO[18]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 1| Intr:2 
I (378) example: Resetting e-Paper display...
I (508) example: Initializing e-Paper display...
I (628) example: Turning e-Paper display on...
I (748) example: Initialize LVGL library
I (748) example: Register display driver to LVGL
I (748) example: Install LVGL tick timer
I (748) example: Display LVGL Meter Widget
...
```

## Performance Notice

- LVGL library does not refresh the e-paper panel unless the content of the panel changes. However, the panel is kept refreshing when enabling `Show CPU usage and FPS count`, so do not forget to disable it to avoid unnecessary refresh.
- If you want to set a different screen rotation permanently, modifying the buffer conversion code in `example_lvgl_flush_cb` is better than using rotation API provided by LVGL. This is because converting `lv_color_t` to `uint8_t[]` needs a data copy, if using software-based rotation, we'll need another copy. Then convert the traversal sequence while converting `lv_color_t` to `uint8_t[]` might be a better idea.
- You should not set the latency of `vTaskDelay()` in the main loop too short. This demo only refresh the screen every tens of seconds, and during the interval of refreshes, the `lv_timer_handler()` is non-blocking and returns immediately, so please keep a reasonable `vTaskDelay()` latency to yield CPU for other tasks. The `vTaskDelay()` uses a busy-wait if the latency you set is too short and this may cause watchdog timeout error.

## Troubleshooting

* Why the e-paper doesn't respond?
  * Maybe your GPIO pin num is not correctly set, check in [main.c](main/main.c).
  * Maybe your waveform lut is incorrect, try stop using your custom waveform lut.
