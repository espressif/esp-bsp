# e-Paper Example

The SSD1681 e-paper display drive uses [esp_lcd](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd.html) APIs and provides some additional APIs because of the specificity of e-paper panel.

This example shows how to use SSD1681 e-paper display driver from Component manager and will draw a few bitmaps to the e-paper panel using the SSD1681 e-paper display driver.

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

### Software configuration

- Change all the `EXAMPLE_PIN` macro definition according to your hardware connection.
- If you are not using waveshare 1.54 inch V2 e-paper panel, please use the waveform lut provided by your panel vendor instead of using the demo built-in ones, or just simply comment the `epaper_panel_set_custom_lut` call and use the panel built-in waveform lut.

### Build and Flash

Run `idf.py -p PORT build flash monitor` to build, flash and monitor the project. 

The first time you run `idf.py` for the example will cost extra time as the build system needs to address the component dependencies and downloads the missing components from registry into `managed_components` folder.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

### Example Output

```bash
...
I (310) epaper_demo_plain: Initializing SPI Bus...
I (320) epaper_demo_plain: Initializing panel IO...
I (330) gpio: GPIO[9]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (330) epaper_demo_plain: Creating SSD1681 panel...
I (340) gpio: GPIO[4]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (350) lcd_panel.epaper: Add handler for GPIO 18
I (350) gpio: GPIO[18]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 0| Pulldown: 1| Intr:2 
I (360) epaper_demo_plain: Resetting e-Paper display...
I (490) epaper_demo_plain: Initializing e-Paper display...
I (610) epaper_demo_plain: Turning e-Paper display on...
I (720) epaper_demo_plain: Drawing bitmap...
...
```

## Show Custom Bitmap

As you could see from the demo, each bitmap is stored as an array. If you want to display your custom image, you need to convert your image to a bitmap array first.

Monochrome as the panel is, every bit (not byte) of the array corresponds to a pixel of the e-paper panel.

You could convert your image to bitmap array by the following steps:
- Resize your image to the size you want.
- Go to the [LVGL Online Image Converter website](https://lvgl.io/tools/imageconverter)
- Upload your resized image.
- Select the Color format to `CF_ALPHA_1_BIT`, select the Output format to `C array`.
- You could select the `Dither images (can improve quality)` to get better (gray-scale like) image quality
- Do not select the `Output in big-endian format` option.
- Click `Convert` and you get a `.c` file containing the bitmap array.

There are plenty of software alternative with such functionality, please pay attention to the scan mode if you prefer to use them.

The driver writes bitmap array to the vram in the sequence below by default:

![scan_mode](scan_mode.svg)

Please make sure that the scan mode identical to the image above. Otherwise you might have to mirror or rotate the bitmap using software implemented functions.

## Troubleshooting

* Why the e-paper is not displaying properly?
    * Maybe your GPIO pin num is not correctly set, check in [main.c](main/main.c).
    * Maybe your waveform lut is incorrect, try stop using your custom waveform lut.

