# How to Use the ESP-BSP API

<div align="center">

| :pager: [DISPLAY & TOUCH](#pager-display--touch) | :musical_note: [AUDIO](#musical_note-audio) | :floppy_disk: [SD & SPIFFS](#floppy_disk-sd-card--spi-flash-file-storage) | :camera: [CAMERA](#camera-camera) |
| :----------------------------------------: | :-----------------------------: | :-----------------------------: | :-------------------------: |

| :radio_button: [BUTTONS](#radio_button-buttons) | :bulb: [LEDS](#bulb-leds) | :battery: [BATTERY](#battery-battery) | :electric_plug: [USB](#electric_plug-usb) | :video_game: [IMU & Sensors](#video_game-imu--sensors) |
| :-------------------------------: | :------------------: | :--------------------------: | :------------------------: | :---------------------------: |

</div>

## Overview

This document provides a unified overview of the ESP-BSP (Board Support Package) API. It is shared across all supported BSPs. Each BSP **may not implement all APIs listed here**, but **should provide implementations for all APIs relevant to the capabilities it supports**.

The goal of this document is to make it easier for developers to understand the available APIs and how to use them consistently across different boards.

## General

### Identification

Each BSP defines an identifier macro in the form of `BSP_BOARD_*`.

### Capabilities

Each BSP defines a set of capability macros that indicate which features are supported.
The list may look like this:

```
#define BSP_CAPS_DISPLAY        1
#define BSP_CAPS_TOUCH          1
#define BSP_CAPS_BUTTONS        1
#define BSP_CAPS_AUDIO          1
#define BSP_CAPS_AUDIO_SPEAKER  1
#define BSP_CAPS_AUDIO_MIC      1
#define BSP_CAPS_SDCARD         1
#define BSP_CAPS_IMU            0
```

You can use these macros to conditionally compile code depending on feature availability.

### Pinout

Each BSP defines a set of macros for default pin assignments used by its hardware peripherals.
These macros allow users to configure or reference standard interfaces like I2C, SPI, LCD, audio, or SD cards easily.

- I2C: `BSP_I2C_*`
- Display: `BSP_LCD_*`
- Audio I2S: `BSP_I2S_*`
- USB: `BSP_USB_*`
- SD Card (MMC): `BSP_SD_*`
- SD Card (SPI): `BSP_SD_SPI_*`

> [!NOTE]
> Not all boards support all interfaces. You should always check if the related capability macro (e.g., BSP_CAPS_SDCARD) is defined.

### I2C

Some devices included in BSPs (e.g., sensors, displays, audio codecs) communicate via the I2C interface. In many cases, I2C is initialized automatically as part of the device setup. However, you can manually initialize or deinitialize the I2C peripheral using the following API:

```
/* Initialize the default I2C bus used by the BSP */
bsp_i2c_init();

...

/* Deinitialize the I2C bus */
bsp_i2c_deinit();
```

If you need direct access to the initialized I2C bus (e.g., to communicate with an external peripheral not handled by the BSP), you can retrieve the I2C bus handle:

```
i2c_master_bus_handle_t i2c = bsp_i2c_get_handle();
```

> [!NOTE]
> The BSP ensures that I2C initialization is performed only once, even if called multiple times. This helps avoid conflicts when multiple components rely on the same I2C bus.


### ADC

Some devices included in BSPs (such as buttons, battery monitoring, etc.) use the ADC peripheral. In most cases, the ADC is automatically initialized as part of the specific device setup. However, you can manually initialize the ADC using the following API:

```
/* Initialize the ADC peripheral */
bsp_adc_initialize();
```

If you need direct access to the ADC instance (e.g., for custom measurements), you can retrieve the handle:

```
adc_oneshot_unit_handle_t adc = bsp_adc_get_handle();
```

> [!NOTE]
> The BSP ensures the ADC is initialized only once, even if `bsp_adc_initialize()` is called multiple times.

### Features

Some boards support enabling or disabling specific hardware features (such as LCD, SD card, camera, etc.) to reduce power consumption or manage shared resources. The BSP provides a unified API to control these features:

```
/* Enable the LCD feature */
bsp_feature_enable(BSP_FEATURE_LCD, true);

/* Disable the speaker to reduce power usage */
bsp_feature_enable(BSP_FEATURE_SPEAKER, false);
```

Supported feature flags (may vary depending on the board):
- `BSP_FEATURE_LCD` - Display module
- `BSP_FEATURE_TOUCH` - Touch controller
- `BSP_FEATURE_SD` - SD card interface
- `BSP_FEATURE_SPEAKER`-  Audio speaker
- `BSP_FEATURE_BATTERY` - Battery monitoring
- `BSP_FEATURE_VIBRATION` - Vibration motor

> [!NOTE]
> Not all BSPs support feature toggling, and some features may not be available or controllable via this API. Always check the BSP header or documentation for supported features.

> [!TIP]
> Disabling unused features can help reduce power consumption, especially in battery-powered applications.

## :pager: Display & Touch

### Initialization

ESP-BSP provides two ways to initialize the **display**, **touch** and **LVGL**.

Simple method:

```
/* Initialize display, touch, and LVGL */
lv_display_t display = bsp_display_start();
```

Configurable method:

```
bsp_display_cfg_t cfg = {
    .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),   /* See LVGL Port for more info */
    .buffer_size = BSP_LCD_V_RES * BSP_LCD_H_RES,   /* Screen buffer size in pixels */
    .double_buffer = true,                          /* Allocate two buffers if true */
    .flags = {
        .buff_dma = true,                           /* Use DMA-capable LVGL buffer */
        .buff_spiram = false,                       /* Allocate buffer in PSRAM if true */
    }
};
cfg.lvgl_port_cfg.task_stack = 10000;   /* Example: change LVGL task stack size */
/* Initialize display, touch, and LVGL */
lv_display_t display = bsp_display_start_with_config(&cfg);
```

After initialization, you can use the [LVGL](https://docs.lvgl.io/master/) API or [LVGL Port](../components/esp_lvgl_port/README.md) API.

### Initialization without LVGL - NoGLIB BSP

To initialize the LCD without LVGL, use:

```
esp_lcd_panel_handle_t panel_handle;
esp_lcd_panel_io_handle_t io_handle;
const bsp_display_config_t bsp_disp_cfg = {
    .max_transfer_sz = (BSP_LCD_H_RES * 100) * sizeof(uint16_t),
};
BSP_ERROR_CHECK_RETURN_NULL(bsp_display_new(&bsp_disp_cfg, &panel_handle, &io_handle));
```

To initialize the LCD touch without LVGL, use:

```
esp_lcd_touch_handle_t tp;
bsp_touch_new(NULL, &tp);
```

After initialization, you can use the [ESP-LCD](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd/index.html) API and [ESP-LCD Touch](../components/lcd_touch/README.md) API.

### Set Brightness

```
/* Set display brightness to 100% */
bsp_display_backlight_on();

/* Set display brightness to 0% */
bsp_display_backlight_off();

/* Set display brightness to 50% */
bsp_display_brightness_set(50);
```

> [!NOTE]
> Some boards do not support changing brightness. They return an `ESP_ERR_NOT_SUPPORTED` error.

### LVGL API Usage (only when initialized with LVGL)

All LVGL calls must be protected using lock/unlock:

```
/* Wait until other tasks finish screen operations */
bsp_display_lock(0);
...
lv_obj_t * screen = lv_disp_get_scr_act(disp_handle);
lv_obj_t * obj = lv_label_create(screen);
...
/* Unlock after screen operations are done */
bsp_display_unlock();
```

### Screen rotation (only when initialized with LVGL)

```
bsp_display_lock(0);
/* Rotate display to 90° */
bsp_display_rotate(display, LV_DISPLAY_ROTATION_90);
bsp_display_unlock();
```

> [!NOTE]
> Some LCDs do not support hardware rotation and instead use software rotation, which consumes more memory.

### Available constants

Constants like screen resolution, pin configuration, and other options are defined in the BSP header files (`{bsp_name}.h`, `display.h`, `touch.h`).
Below are some of the most relevant predefined constants:

- `BSP_LCD_H_RES` - Horizontal resolution in pixels
- `BSP_LCD_V_RES` - Vertical resolution in pixels
- `BSP_LCD_SPI_NUM` - SPI bus used by the LCD (if applicable)

## :musical_note: Audio

### Initialization

Before using speaker or microphone features, the audio codec must be initialized.

```
/* Initialize the speaker codec */
esp_codec_dev_handle_t spk_codec_dev = bsp_audio_codec_speaker_init();

/* Initialize the microphone codec */
esp_codec_dev_handle_t mic_codec_dev = bsp_audio_codec_microphone_init();
```

After initialization, the [esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev) API can be used to control playback and recording.

> [!NOTE]
> Some BSPs may only support playback (speaker) or only input (microphone). Use the capability macros (`BSP_CAPS_AUDIO`, `BSP_CAPS_AUDIO_SPEAKER`, `BSP_CAPS_AUDIO_MIC`) to check supported features.

### Example of audio usage

#### Speaker

Below is an example of audio playback using the speaker (source data not included):

```
/* Set volume to 50% */
esp_codec_dev_set_out_vol(spk_codec_dev, 50);

/* Define audio format */
esp_codec_dev_sample_info_t fs = {
    .sample_rate = wav_header.sample_rate,
    .channel = wav_header.num_channels,
    .bits_per_sample = wav_header.bits_per_sample,
};
/* Open speaker stream */
esp_codec_dev_open(spk_codec_dev, &fs);

...
/* Play audio data */
esp_codec_dev_write(spk_codec_dev, wav_bytes, wav_bytes_len);
...

/* Close stream when done */
esp_codec_dev_close(spk_codec_dev);
```

> [!TIP]
> Audio data must be in raw PCM format. Use a decoder if playing compressed formats (e.g., WAV, MP3).

#### Microphone

Below is an example of recording audio using the microphone (destination buffer not included):

```
/* Set input gain (optional) */
esp_codec_dev_set_in_gain(mic_codec_dev, 42.0);

/* Define audio format */
esp_codec_dev_sample_info_t fs = {
    .sample_rate = 16000,
    .channel = 1,
    .bits_per_sample = 16,
};
/* Open microphone stream */
esp_codec_dev_open(mic_codec_dev, &fs);

/* Read recorded data */
esp_codec_dev_read(mic_codec_dev, recording_buffer, BUFFER_SIZE)

...

/* Close stream when done */
esp_codec_dev_close(mic_codec_dev);
```

## :floppy_disk: SD Card & SPI Flash File Storage

### SPIFFS Initialization / Deinitialization

Each BSP provides a simple API for mounting and unmounting the SPI Flash File System (SPIFFS).

```
/* Mount SPIFFS to the virtual file system */
bsp_spiffs_mount();

/* ... perform file operations ... */

/* Unmount SPIFFS from the virtual file system */
bsp_spiffs_unmount();
```

### SD Card Initialization / Deinitialization

The BSP offers a flexible API for working with SD cards. In addition to the default mount and unmount functions, you can also use a configuration structure or access preconfigured `host` and `slot` structures.

Mount with Default Configuration

```
/* Mount microSD card to the virtual file system */
bsp_sdcard_mount();

/* ... perform file operations ... */

/* Unmount microSD card */
bsp_sdcard_unmount();
```

Mount with Custom Configuration

Some BSPs allow selecting between SDMMC and SPI interfaces for the SD card. Use the appropriate API function based on your hardware:
```
bsp_sdcard_cfg_t cfg = {0};
/* Mount SD card using SDMMC interface */
bsp_sdcard_sdmmc_mount(&cfg);
```

or

```
bsp_sdcard_cfg_t cfg = {0};
/* Mount SD card using SPI interface */
bsp_sdcard_sdspi_mount(&cfg)
```

> [!NOTE]
> Not all BSPs support both SDMMC and SPI modes. Check the board documentation to see which interfaces are available.
> If an unsupported interface is used, the API will return `ESP_ERR_NOT_SUPPORTED` error.

### After Mounting

Once the SD card or SPIFFS is mounted, you can use standard file I/O functions (`fopen`, `fread`, `fwrite`, `fclose`, etc.) provided by ESP-IDF's VFS (Virtual File System).

To print basic SD card information (after mounting), you can use:
```
sdmmc_card_t *sdcard = bsp_sdcard_get_handle();
sdmmc_card_print_info(stdout, sdcard);
```

> [!TIP]
> The bsp_sdcard_get_handle() function returns a pointer to the sdmmc_card_t structure, which contains detailed information about the connected SD card.

## :camera: Camera

The BSP provides a helper function bsp_camera_start() for initializing the on-board camera module.
This function sets up the required I2C bus, video subsystem, and camera clock if necessary.

### Example Usage

Camera usage can be quite complex. For a complete example, refer to the [`display_camera_csi`](https://github.com/espressif/esp-bsp/tree/master/examples/display_camera_csi) example in the BSP repository, or to the examples provided in the [`esp_video`](https://github.com/espressif/esp-video-components/tree/master/esp_video) component.

> [!NOTE]
> Please, do not forget select right camera sensor in `menuconfig`

## :radio_button: Buttons

Most boards include one or more user buttons. The BSP provides a simple API to initialize and use them.
Internally, it utilizes the [button](https://components.espressif.com/components/espressif/button/) component for event handling and debouncing.

```
/* Initialize all available buttons */
button_handle_t btns[BSP_BUTTON_NUM] = {NULL};
bsp_iot_button_create(btns, NULL, BSP_BUTTON_NUM);

/* Register a callback for button press */
for (int i = 0; i < BSP_BUTTON_NUM; i++) {
    iot_button_register_cb(btns[i], BUTTON_PRESS_DOWN, NULL, btn_handler, (void *) i);
}

/* Called on button press */
static void btn_handler(void *button_handle, void *usr_data)
{
    int button_index = (int)usr_data;
    ESP_LOGI(TAG, "Button %d pressed", button_index);
}
```

**Notes:**
- `BSP_BUTTON_NUM` defines the number of available buttons on the board

- Instead of relying on a numeric index, you can use the `bsp_button_t` structure defined in the BSP header to reference buttons by name (e.g., BSP_BUTTON_MUTE, BSP_BUTTON_MAIN, BSP_BUTTON_BOOT, etc.)

- The callback can be registered for different button events, such as `BUTTON_PRESS_DOWN`, `BUTTON_PRESS_UP`, or `BUTTON_LONG_PRESS_START`, `BUTTON_DOUBLE_CLICK`, ...

- Button features (e.g. long-press support, active level) are configured automatically by the BSP.

## :bulb: LEDs

LEDs are handled similarly to buttons in BSP. The BSP uses the [led_indicator](https://components.espressif.com/components/espressif/led_indicator) component, which provides simple control over LED states and built-in effects such as blinking, breathing, and more. It also supports addressable RGB LEDs.

```
/* Initialize all LEDs */
bsp_led_indicator_create(leds, NULL, BSP_LED_NUM);

/* Set color of the first LED (for addressable RGB LEDs only) */
led_indicator_set_rgb(leds[0], SET_IRGB(0, 0x00, 0x64, 0x64));

/*
Start a predefined LED effect:
- BSP_LED_ON
- BSP_LED_OFF
- BSP_LED_BLINK_FAST
- BSP_LED_BLINK_SLOW
- BSP_LED_BREATHE_FAST
- BSP_LED_BREATHE_SLOW
*/
led_indicator_start(leds[0], BSP_LED_BREATHE_SLOW);
```

**Notes:**
- `BSP_LED_NUM` defines the total number of available LEDs on the board
- LEDs are automatically configured by the BSP (no need to set GPIO or direction manually)

## :battery: Battery

Some boards with battery support can measure the battery voltage using an ADC channel. BSP provides a simple API for this:

```
/* Initialize the battery voltage measurement */
bsp_voltage_init();

/* Read battery voltage in millivolts */
int voltage = bsp_voltage_battery_get();
```

## :electric_plug: USB

Boards with USB support define macros for USB pins, such as `BSP_USB_POS` and `BSP_USB_NEG`, and may also provide control APIs for enabling or disabling USB functionality.

```
/* Initialize USB in device mode and enable power */
bsp_usb_host_start(BSP_USB_HOST_POWER_MODE_USB_DEV, true);

...
/* Deinitialize and stop USB */
bsp_usb_host_stop();
```

> [!NOTE]
> Not all BSPs implement USB support or provide power control. Refer to the board's documentation and the BSP header files for available functions and supported modes.

For more USB-related APIs and configuration options, check the corresponding BSP header files.

## :video_game: IMU & Sensors

Boards with integrated sensors (e.g., IMUs, environmental sensors) are not fully abstracted by the BSP. The BSP does not provide specific APIs for sensor control or data retrieval.

Instead, the BSP includes selected sensor components via the `idf_component.yml` file and assists with peripheral initialization (typically I2C bus configuration).

> [!NOTE]
> Sensor initialization and usage are left to the user. The BSP ensures the I2C bus is properly configured but does not wrap sensor APIs.

For practical usage examples and supported sensors on your board, refer to the relevant examples in the BSP repository, such as:
- `display_rotation` (for IMU setup and orientation control)
- `display_sensors` (for reading sensor data)

