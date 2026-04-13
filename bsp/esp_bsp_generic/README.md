# BSP: Generic

| [HOW TO USE API](https://github.com/espressif/esp-bsp/blob/master/docu/how_to_use.md) | [EXAMPLES](#compatible-bsp-examples) | [![Component Registry](https://components.espressif.com/components/espressif/esp_bsp_generic/badge.svg)](https://components.espressif.com/components/espressif/esp_bsp_generic) | ![maintenance-status](https://img.shields.io/badge/maintenance-actively--developed-brightgreen.svg) |
| --- | --- | --- | --- |

## Overview

<table>
<tr><td>

This is a generic BSP, which is configurable from `menuconfig`. The generic BSP can provide consistent API for simple boards, such as Espressif's DevKits. Predefined settings for selected DevKits are in [generic_button_led](examples/generic_button_led).

</td><td width="200">
  <img src="doc/esp_bsp_generic.webp">
</td></tr>
</table>

**Supported features:**
- I2C
- SPIFFS
- SD card
- Buttons
- LEDs

# Build with predefined configuration

Predefined configurations are saved in [generic_button_led](examples/generic_button_led) example.

```
    idf.py -p COM4 -D "SDKCONFIG_DEFAULTS=sdkconfig.esp32_s3_devkitc_1" flash monitor
```

# Example usage

## I2C

1. Set GPIOs for I2C in `menuconfig`
    - `BSP_I2C_GPIO_SCL`
    - `BSP_I2C_GPIO_SDA`

2. Set I2C number in `menuconfig`
    - `BSP_I2C_NUM`

Example code:
```
    /* Initialization */
    bsp_i2c_init();

    /* Example I2C write with BSP_I2C_NUM */
    i2c_master_write_to_device(BSP_I2C_NUM, 0x10, value, sizeof(value), 1000 / portTICK_PERIOD_MS);

    ...

    bsp_i2c_deinit();
```
**Note:** The BSP automatically initialize I2C, when need it for some component (LCD touch, audio, etc.)

## SPIFFS

Example code:
```
    /* Mount SPIFFS partition */
    bsp_spiffs_mount();

    /* Use file system read/write with BSP_SPIFFS_MOUNT_POINT */
    FILE *in = fopen(BSP_SD_MOUNT_POINT"/text.txt", "rb");

    ...

    bsp_spiffs_unmount();
```

## SD card

Example code:
```
    /* Mount SD card partition */
    bsp_sdcard_mount();

    /* Use file system read/write with BSP_SD_MOUNT_POINT */
    FILE *in = fopen(BSP_SD_MOUNT_POINT"/text.txt", "rb");

    ...

    bsp_sdcard_unmount();
```

**Note:** This API is available only in MCUs, which have SD MMC peripheral.

## Buttons

1. Set count of buttons in `menuconfig`
    - `BSP_BUTTONS_NUM` (max 5)

2. Set button type and other values by type for each button in `menuconfig`
    - `BSP_BUTTON_x_TYPE`
    - `BSP_BUTTON_x_GPIO`           (for GPIO button)
    - `BSP_BUTTON_x_LEVEL`          (for GPIO button)
    - `BSP_BUTTON_x_ADC_CHANNEL`    (for ADC button)
    - `BSP_BUTTON_x_ADC_VALUE`      (for ADC button)

Example code:
```
    /* Button callback */
    static void btn_handler(void *button_handle, void *usr_data)
    {
        int button_pressed = (int)usr_data;

        ESP_LOGI(TAG, "Button pressed: %d", button_pressed);
    }

    /* Initialize all buttons and register callback for them */
    button_handle_t btns[BSP_BUTTON_NUM];
    ESP_ERROR_CHECK(bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM));
    for (int i = 0; i < BSP_BUTTON_NUM; i++) {
        ESP_ERROR_CHECK(iot_button_register_cb(btns[i], BUTTON_PRESS_DOWN, btn_handler, (void *) i));
    }
```
For button handling is used component [iot_button](https://components.espressif.com/components/espressif/button). For more information, please look into guide for this component.

## LEDS

1. Set count of LEDs in `menuconfig`
    - `BSP_LEDS_NUM` (max 5)

2. Set type for all LEDs in `menuconfig`
    - `BSP_LED_TYPE` (GPIO / Adressable RGB LED / Classic RGB)

3. For GPIO LEDs set pin and level for each LED in `menuconfig`
    - `BSP_LED_x_GPIO`
    - `BSP_LED_x_LEVEL`

3. For addressable RBG LEDs set pin and peripheral in `menuconfig`
    - `BSP_LED_RGB_GPIO`
    - `BSP_LED_RGB_BACKEND`

3. For classic RBG LEDs set pins for all colors and level in `menuconfig`
    - `BSP_LED_RGB_RED_GPIO`
    - `BSP_LED_RGB_GREEN_GPIO`
    - `BSP_LED_RGB_BLUE_GPIO`
    - `BSP_LED_RGB_CLASSIC_LEVEL`

Example code:
```
    /* Initialize all LEDs */
    led_indicator_handle_t leds[BSP_LED_NUM];
    ESP_ERROR_CHECK(bsp_led_indicator_create(leds, NULL, BSP_LED_NUM));

    /* Set LED color for first LED (only for addressable RGB LEDs) */
    led_indicator_set_rgb(leds[0], SET_IRGB(0, 0x00, 0x64, 0x64));

    /* Start effect for each LED (predefined: BSP_LED_ON, BSP_LED_OFF, BSP_LED_BLINK_FAST, BSP_LED_BLINK_SLOW, BSP_LED_BREATHE_FAST, BSP_LED_BREATHE_SLOW) */
    led_indicator_start(leds[0], BSP_LED_BREATHE_SLOW);
```
For LEDs handling is used component [led_indicator](https://components.espressif.com/components/espressif/led_indicator) with [led_strip](https://components.espressif.com/components/espressif/led_strip) component. For more information, please look into guides for these components.

## LCD Display

1. Enable display in `menuconfig`
    - `BSP_DISPLAY_ENABLED`

2. Select communication interface in `menuconfig`
    - `BSP_DISPLAY_INTERFACE_` (only SPI is supported)

3. Set communication pins in `menuconfig`
    - `BSP_DISPLAY_SCLK_GPIO`
    - `BSP_DISPLAY_MOSI_GPIO`
    - `BSP_DISPLAY_MISO_GPIO`
    - `BSP_DISPLAY_CS_GPIO`
    - `BSP_DISPLAY_DC_GPIO`
    - `BSP_DISPLAY_RST_GPIO`
    - `BSP_DISPLAY_BACKLIGHT_GPIO`

4. Select display driver in `menuconfig` (one of these)
    - `BSP_DISPLAY_DRIVER_ST7789`
    - `BSP_DISPLAY_DRIVER_ILI9341`
    - `BSP_DISPLAY_DRIVER_GC9A01`

5. Set right rotation of the screen in `menuconfig`
    - `BSP_DISPLAY_ROTATION_SWAP_XY`
    - `BSP_DISPLAY_ROTATION_MIRROR_X`
    - `BSP_DISPLAY_ROTATION_MIRROR_Y`

6. Set other display params in `menuconfig`
    - `BSP_DISPLAY_CMD_BITS`
    - `BSP_DISPLAY_PARAM_BITS`
    - `BSP_DISPLAY_PIXEL_CLOCK`
    - `BSP_DISPLAY_WIDTH`
    - `BSP_DISPLAY_HEIGHT`
    - `BSP_DISPLAY_BRIGHTNESS_LEDC_CH`
    - `BSP_LCD_DRAW_BUF_HEIGHT`
    - `BSP_LCD_DRAW_BUF_DOUBLE`

## LCD Touch

1. Enable display touch in `menuconfig`
    - `BSP_TOUCH_ENABLED`

2. Select communication interface in `menuconfig`
    - `BSP_TOUCH_INTERFACE_` (only I2C is supported)

3. Set communication pins in `menuconfig`
    - `BSP_TOUCH_RST_GPIO`
    - `BSP_TOUCH_INT_GPIO`

4. Select display driver in `menuconfig` (one of these)
    - `BSP_TOUCH_DRIVER_TT21100`
    - `BSP_TOUCH_DRIVER_GT1151`
    - `BSP_TOUCH_DRIVER_GT911`
    - `BSP_TOUCH_DRIVER_CST816S`
    - `BSP_TOUCH_DRIVER_FT5X06`

5. Set right rotation of the screen in `menuconfig`
    - `BSP_TOUCH_ROTATION_SWAP_XY`
    - `BSP_TOUCH_ROTATION_MIRROR_X`
    - `BSP_TOUCH_ROTATION_MIRROR_Y`

Example code:
```
    /* Initialize display, touch and LVGL */
    bsp_display_start();

    /* Set display brightness to 100% */
    bsp_display_backlight_on();

    bsp_display_lock(0);
    /* === Your LVGL code here === */
    bsp_display_unlock();
```
## Capabilities and dependencies

<div align="center">
<!-- START_DEPENDENCIES -->

|     Available    |       Capability       |            Controller/Codec           |                                                                                                                                                                                                                                                                                     Component                                                                                                                                                                                                                                                                                    |            Version           |
|------------------|------------------------|---------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------|
|:heavy_check_mark:|     :pager: DISPLAY    |        st7789, ili9341, gc9a01        |                                                                                                                                                                                  idf<br/>[espressif/esp_lcd_ili9341](https://components.espressif.com/components/espressif/esp_lcd_ili9341)<br/>[espressif/esp_lcd_gc9a01](https://components.espressif.com/components/espressif/esp_lcd_gc9a01)                                                                                                                                                                                 |    >=5.2<br/>^2.0.1<br/>^2   |
|:heavy_check_mark:|:black_circle: LVGL_PORT|                                       |                                                                                                                                                                                                                                          [espressif/esp_lvgl_port](https://components.espressif.com/components/espressif/esp_lvgl_port)                                                                                                                                                                                                                                          |              ^2              |
|:heavy_check_mark:|    :point_up: TOUCH    |tt21100, gt1151, gt911, cst816s, ft5x06|[espressif/esp_lcd_touch_tt21100](https://components.espressif.com/components/espressif/esp_lcd_touch_tt21100)<br/>[espressif/esp_lcd_touch_gt1151](https://components.espressif.com/components/espressif/esp_lcd_touch_gt1151)<br/>[espressif/esp_lcd_touch_gt911](https://components.espressif.com/components/espressif/esp_lcd_touch_gt911)<br/>[espressif/esp_lcd_touch_cst816s](https://components.espressif.com/components/espressif/esp_lcd_touch_cst816s)<br/>[espressif/esp_lcd_touch_ft5x06](https://components.espressif.com/components/espressif/esp_lcd_touch_ft5x06)|^1<br/>^1<br/>^1<br/>^1<br/>^1|
|:heavy_check_mark:| :radio_button: BUTTONS |                                       |                                                                                                                                                                                                                                                 [espressif/button](https://components.espressif.com/components/espressif/button)                                                                                                                                                                                                                                                 |              ^4              |
|:heavy_check_mark:|       :bulb: LED       |                                       |                                                                                                                                                                                                                                      idf<br/>[espressif/led_indicator](https://components.espressif.com/components/espressif/led_indicator)                                                                                                                                                                                                                                      |        >=5.2<br/>~2.0        |
|        :x:       |  :musical_note: AUDIO  |                                       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |                              |
|        :x:       | :speaker: AUDIO_SPEAKER|                                       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |                              |
|        :x:       | :microphone: AUDIO_MIC |                                       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |                              |
|        :x:       |  :floppy_disk: SDCARD  |                                       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |                              |
|        :x:       |    :video_game: IMU    |                                       |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  |                              |

<!-- END_DEPENDENCIES -->
</div>

## Compatible BSP Examples

<div align="center">
<!-- START_EXAMPLES -->

| Example | Description | Try with ESP Launchpad |
| ------- | ----------- | ---------------------- |
| [Generic Button and LED Example](https://github.com/espressif/esp-bsp/tree/master/examples/generic_button_led) | Minimal example using the Generic BSP: button and LED control | - |

<!-- END_EXAMPLES -->
</div>
