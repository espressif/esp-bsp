# API Reference

<div align="center">




| :1234: [CAPABILITIES](#1234-capabilities) | :floppy_disk: [SD CARD AND SPIFFS](#floppy_disk-sd-card-and-spiffs) | :musical_note: [AUDIO](#musical_note-audio) | :radio_button: [BUTTONS](#radio_button-buttons) | :bulb: [LEDS](#bulb-leds) | 
| :-------------------------: | :-------------------------: | :-------------------------: | :-------------------------: | :-------------------------: | 

</div>



## Overview

This document provides an overview of the ESP-BSP (Board Support Package) API as implemented by this board.

While the ESP-BSP framework defines a unified API shared across multiple boards, this documentation focuses only on the APIs supported by the current board. Any APIs not applicable to this board's hardware are excluded or may not be functional.

The goal of this document is to make it easier for developers to understand the available APIs and how to use them consistently across different boards.

## General

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




## Identification

Each BSP defines an identifier macro in the form of `BSP_BOARD_*`.

### Board Name API Reference



## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_BOARD\_ESP32\_LYRAT**](#define-bsp_board_esp32_lyrat)  <br> |









## :1234: Capabilities

Each BSP defines a set of capability macros that indicate which features are supported.
The list may look like this.
You can use these macros to conditionally compile code depending on feature availability.

### Capabilities API Reference



## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_CAPS\_AUDIO**](#define-bsp_caps_audio)  1<br> |
| define  | [**BSP\_CAPS\_AUDIO\_MIC**](#define-bsp_caps_audio_mic)  1<br> |
| define  | [**BSP\_CAPS\_AUDIO\_SPEAKER**](#define-bsp_caps_audio_speaker)  1<br> |
| define  | [**BSP\_CAPS\_BUTTONS**](#define-bsp_caps_buttons)  1<br> |
| define  | [**BSP\_CAPS\_DISPLAY**](#define-bsp_caps_display)  0<br> |
| define  | [**BSP\_CAPS\_IMU**](#define-bsp_caps_imu)  0<br> |
| define  | [**BSP\_CAPS\_LED**](#define-bsp_caps_led)  1<br> |
| define  | [**BSP\_CAPS\_SDCARD**](#define-bsp_caps_sdcard)  1<br> |
| define  | [**BSP\_CAPS\_TOUCH**](#define-bsp_caps_touch)  0<br> |











### I2C API Reference


## Functions

| Type | Name |
| ---: | :--- |
|  esp\_err\_t | [**bsp\_i2c\_deinit**](#function-bsp_i2c_deinit) (void) <br>_Deinit I2C driver and free its resources._ |
|  esp\_err\_t | [**bsp\_i2c\_init**](#function-bsp_i2c_init) (void) <br>_Init I2C driver._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_I2C\_NUM**](#define-bsp_i2c_num)  CONFIG\_BSP\_I2C\_NUM<br> |
| define  | [**BSP\_I2C\_SCL**](#define-bsp_i2c_scl)  (GPIO\_NUM\_23)<br> |
| define  | [**BSP\_I2C\_SDA**](#define-bsp_i2c_sda)  (GPIO\_NUM\_18)<br> |



## Functions Documentation

### function `bsp_i2c_deinit`

_Deinit I2C driver and free its resources._
```c
esp_err_t bsp_i2c_deinit (
    void
) 
```


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG I2C parameter error
### function `bsp_i2c_init`

_Init I2C driver._
```c
esp_err_t bsp_i2c_init (
    void
) 
```


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_INVALID\_ARG I2C parameter error
* ESP\_FAIL I2C driver installation error






## :floppy_disk: SD Card and SPIFFS

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

### SD Card and SPIFFS API Reference

## Structures and Types

| Type | Name |
| ---: | :--- |
| struct | [**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) <br>_BSP SD card configuration structure._ |

## Functions

| Type | Name |
| ---: | :--- |
|  sdmmc\_card\_t \* | [**bsp\_sdcard\_get\_handle**](#function-bsp_sdcard_get_handle) (void) <br>_Get SD card handle._ |
|  void | [**bsp\_sdcard\_get\_sdmmc\_host**](#function-bsp_sdcard_get_sdmmc_host) (const int slot, sdmmc\_host\_t \*config) <br>_Get SD card MMC host config._ |
|  void | [**bsp\_sdcard\_get\_sdspi\_host**](#function-bsp_sdcard_get_sdspi_host) (const int slot, sdmmc\_host\_t \*config) <br>_Get SD card SPI host config._ |
|  esp\_err\_t | [**bsp\_sdcard\_mount**](#function-bsp_sdcard_mount) (void) <br>_Mount microSD card to virtual file system._ |
|  void | [**bsp\_sdcard\_sdmmc\_get\_slot**](#function-bsp_sdcard_sdmmc_get_slot) (const int slot, sdmmc\_slot\_config\_t \*config) <br>_Get SD card MMC slot config._ |
|  esp\_err\_t | [**bsp\_sdcard\_sdmmc\_mount**](#function-bsp_sdcard_sdmmc_mount) ([**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) \*cfg) <br>_Mount microSD card to virtual file system (MMC mode)_ |
|  void | [**bsp\_sdcard\_sdspi\_get\_slot**](#function-bsp_sdcard_sdspi_get_slot) (const spi\_host\_device\_t spi\_host, sdspi\_device\_config\_t \*config) <br>_Get SD card SPI slot config._ |
|  esp\_err\_t | [**bsp\_sdcard\_sdspi\_mount**](#function-bsp_sdcard_sdspi_mount) ([**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) \*cfg) <br>_Mount microSD card to virtual file system (SPI mode)_ |
|  esp\_err\_t | [**bsp\_sdcard\_unmount**](#function-bsp_sdcard_unmount) (void) <br>_Unmount microSD card from virtual file system._ |
|  esp\_err\_t | [**bsp\_spiffs\_mount**](#function-bsp_spiffs_mount) (void) <br>_Mount SPIFFS to virtual file system._ |
|  esp\_err\_t | [**bsp\_spiffs\_unmount**](#function-bsp_spiffs_unmount) (void) <br>_Unmount SPIFFS from virtual file system._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_SDSPI\_HOST**](#define-bsp_sdspi_host)  (SDSPI\_DEFAULT\_HOST)<br> |
| define  | [**BSP\_SD\_CLK**](#define-bsp_sd_clk)  (GPIO\_NUM\_14)<br> |
| define  | [**BSP\_SD\_CMD**](#define-bsp_sd_cmd)  (GPIO\_NUM\_15)<br> |
| define  | [**BSP\_SD\_D0**](#define-bsp_sd_d0)  (GPIO\_NUM\_2)<br> |
| define  | [**BSP\_SD\_MOUNT\_POINT**](#define-bsp_sd_mount_point)  CONFIG\_BSP\_SD\_MOUNT\_POINT<br> |
| define  | [**BSP\_SD\_SPI\_CLK**](#define-bsp_sd_spi_clk)  (GPIO\_NUM\_14)<br> |
| define  | [**BSP\_SD\_SPI\_CS**](#define-bsp_sd_spi_cs)  (GPIO\_NUM\_13)<br> |
| define  | [**BSP\_SD\_SPI\_MISO**](#define-bsp_sd_spi_miso)  (GPIO\_NUM\_2)<br> |
| define  | [**BSP\_SD\_SPI\_MOSI**](#define-bsp_sd_spi_mosi)  (GPIO\_NUM\_15)<br> |
| define  | [**BSP\_SPIFFS\_MOUNT\_POINT**](#define-bsp_spiffs_mount_point)  CONFIG\_BSP\_SPIFFS\_MOUNT\_POINT<br> |


## Structures and Types Documentation

### struct `bsp_sdcard_cfg_t`

_BSP SD card configuration structure._

Variables:

-  sdmmc\_host\_t \* host  

-  const esp\_vfs\_fat\_sdmmc\_mount\_config\_t \* mount  

-  const sdmmc\_slot\_config\_t \* sdmmc  

-  const sdspi\_device\_config\_t \* sdspi  

-  union [**bsp\_sdcard\_cfg\_t**](#struct-bsp_sdcard_cfg_t) slot  


## Functions Documentation

### function `bsp_sdcard_get_handle`

_Get SD card handle._
```c
sdmmc_card_t * bsp_sdcard_get_handle (
    void
) 
```


**Returns:**

SD card handle
### function `bsp_sdcard_get_sdmmc_host`

_Get SD card MMC host config._
```c
void bsp_sdcard_get_sdmmc_host (
    const int slot,
    sdmmc_host_t *config
) 
```


**Parameters:**


* `slot` SD card slot 
* `config` Structure which will be filled
### function `bsp_sdcard_get_sdspi_host`

_Get SD card SPI host config._
```c
void bsp_sdcard_get_sdspi_host (
    const int slot,
    sdmmc_host_t *config
) 
```


**Parameters:**


* `slot` SD card slot 
* `config` Structure which will be filled
### function `bsp_sdcard_mount`

_Mount microSD card to virtual file system._
```c
esp_err_t bsp_sdcard_mount (
    void
) 
```


**Returns:**



* ESP\_OK on success
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_fat\_sdmmc\_mount was already called
* ESP\_ERR\_NO\_MEM if memory cannot be allocated
* ESP\_FAIL if partition cannot be mounted
* other error codes from SDMMC or SPI drivers, SDMMC protocol, or FATFS drivers
### function `bsp_sdcard_sdmmc_get_slot`

_Get SD card MMC slot config._
```c
void bsp_sdcard_sdmmc_get_slot (
    const int slot,
    sdmmc_slot_config_t *config
) 
```


**Parameters:**


* `slot` SD card slot 
* `config` Structure which will be filled
### function `bsp_sdcard_sdmmc_mount`

_Mount microSD card to virtual file system (MMC mode)_
```c
esp_err_t bsp_sdcard_sdmmc_mount (
    bsp_sdcard_cfg_t *cfg
) 
```


**Parameters:**


* `cfg` SD card configuration


**Returns:**



* ESP\_OK on success
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_fat\_sdmmc\_mount was already called
* ESP\_ERR\_NO\_MEM if memory cannot be allocated
* ESP\_FAIL if partition cannot be mounted
* other error codes from SDMMC or SPI drivers, SDMMC protocol, or FATFS drivers
### function `bsp_sdcard_sdspi_get_slot`

_Get SD card SPI slot config._
```c
void bsp_sdcard_sdspi_get_slot (
    const spi_host_device_t spi_host,
    sdspi_device_config_t *config
) 
```


**Parameters:**


* `spi_host` SPI host ID 
* `config` Structure which will be filled
### function `bsp_sdcard_sdspi_mount`

_Mount microSD card to virtual file system (SPI mode)_
```c
esp_err_t bsp_sdcard_sdspi_mount (
    bsp_sdcard_cfg_t *cfg
) 
```


**Parameters:**


* `cfg` SD card configuration


**Returns:**



* ESP\_OK on success
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_fat\_sdmmc\_mount was already called
* ESP\_ERR\_NO\_MEM if memory cannot be allocated
* ESP\_FAIL if partition cannot be mounted
* other error codes from SDMMC or SPI drivers, SDMMC protocol, or FATFS drivers
### function `bsp_sdcard_unmount`

_Unmount microSD card from virtual file system._
```c
esp_err_t bsp_sdcard_unmount (
    void
) 
```


**Returns:**



* ESP\_OK on success
* ESP\_ERR\_NOT\_FOUND if the partition table does not contain FATFS partition with given label
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_fat\_spiflash\_mount was already called
* ESP\_ERR\_NO\_MEM if memory can not be allocated
* ESP\_FAIL if partition can not be mounted
* other error codes from wear levelling library, SPI flash driver, or FATFS drivers
### function `bsp_spiffs_mount`

_Mount SPIFFS to virtual file system._
```c
esp_err_t bsp_spiffs_mount (
    void
) 
```


**Returns:**



* ESP\_OK on success
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_spiffs\_register was already called
* ESP\_ERR\_NO\_MEM if memory can not be allocated
* ESP\_FAIL if partition can not be mounted
* other error codes
### function `bsp_spiffs_unmount`

_Unmount SPIFFS from virtual file system._
```c
esp_err_t bsp_spiffs_unmount (
    void
) 
```


**Returns:**



* ESP\_OK on success
* ESP\_ERR\_NOT\_FOUND if the partition table does not contain SPIFFS partition with given label
* ESP\_ERR\_INVALID\_STATE if esp\_vfs\_spiffs\_unregister was already called
* ESP\_ERR\_NO\_MEM if memory can not be allocated
* ESP\_FAIL if partition can not be mounted
* other error codes






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

### Audio API Reference


## Functions

| Type | Name |
| ---: | :--- |
|  esp\_codec\_dev\_handle\_t | [**bsp\_audio\_codec\_microphone\_init**](#function-bsp_audio_codec_microphone_init) (void) <br>_Initialize microphone codec device._ |
|  esp\_codec\_dev\_handle\_t | [**bsp\_audio\_codec\_speaker\_init**](#function-bsp_audio_codec_speaker_init) (void) <br>_Initialize speaker codec device._ |
|  const audio\_codec\_data\_if\_t \* | [**bsp\_audio\_get\_codec\_itf**](#function-bsp_audio_get_codec_itf) (void) <br>_Get codec I2S interface (initialized in bsp\_audio\_init)_ |
|  esp\_err\_t | [**bsp\_audio\_init**](#function-bsp_audio_init) (const i2s\_std\_config\_t \*i2s\_config) <br>_Init audio._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_I2S\_DOUT**](#define-bsp_i2s_dout)  (GPIO\_NUM\_26)<br> |
| define  | [**BSP\_I2S\_DSIN**](#define-bsp_i2s_dsin)  (GPIO\_NUM\_35)<br> |
| define  | [**BSP\_I2S\_LCLK**](#define-bsp_i2s_lclk)  (GPIO\_NUM\_25)<br> |
| define  | [**BSP\_I2S\_MCLK**](#define-bsp_i2s_mclk)  (GPIO\_NUM\_0)<br> |
| define  | [**BSP\_I2S\_SCLK**](#define-bsp_i2s_sclk)  (GPIO\_NUM\_5)<br> |
| define  | [**BSP\_POWER\_AMP\_IO**](#define-bsp_power_amp_io)  (GPIO\_NUM\_21)<br> |



## Functions Documentation

### function `bsp_audio_codec_microphone_init`

_Initialize microphone codec device._
```c
esp_codec_dev_handle_t bsp_audio_codec_microphone_init (
    void
) 
```


**Returns:**

Pointer to codec device handle or NULL when error occurred
### function `bsp_audio_codec_speaker_init`

_Initialize speaker codec device._
```c
esp_codec_dev_handle_t bsp_audio_codec_speaker_init (
    void
) 
```


**Returns:**

Pointer to codec device handle or NULL when error occurred
### function `bsp_audio_get_codec_itf`

_Get codec I2S interface (initialized in bsp\_audio\_init)_
```c
const audio_codec_data_if_t * bsp_audio_get_codec_itf (
    void
) 
```


**Returns:**



* Pointer to codec I2S interface handle or NULL when error occurred
### function `bsp_audio_init`

_Init audio._
```c
esp_err_t bsp_audio_init (
    const i2s_std_config_t *i2s_config
) 
```


**Note:**

There is no deinit audio function. Users can free audio resources by calling i2s\_del\_channel() 



**Warning:**

The type of i2s\_config param is depending on IDF version. 



**Parameters:**


* `i2s_config` I2S configuration. Pass NULL to use default values (Mono, duplex, 16bit, 22050 Hz) 


**Returns:**



* ESP\_OK On success
* ESP\_ERR\_NOT\_SUPPORTED The communication mode is not supported on the current chip
* ESP\_ERR\_INVALID\_ARG NULL pointer or invalid configuration
* ESP\_ERR\_NOT\_FOUND No available I2S channel found
* ESP\_ERR\_NO\_MEM No memory for storing the channel information
* ESP\_ERR\_INVALID\_STATE This channel has not initialized or already started






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

### Buttons API Reference

## Structures and Types

| Type | Name |
| ---: | :--- |
| enum  | [**bsp\_button\_t**](#enum-bsp_button_t)  <br> |

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_err\_t | [**bsp\_iot\_button\_create**](#function-bsp_iot_button_create) (button\_handle\_t btn\_array, int \*btn\_cnt, int btn\_array\_size) <br>_Initialize all buttons._ |

## Macros

| Type | Name |
| ---: | :--- |
| define  | [**BSP\_BUTTON\_MODE\_IO**](#define-bsp_button_mode_io)  (GPIO\_NUM\_39)<br> |
| define  | [**BSP\_BUTTON\_PLAY\_TOUCH**](#define-bsp_button_play_touch)  (TOUCH\_PAD\_NUM8)<br> |
| define  | [**BSP\_BUTTON\_REC\_IO**](#define-bsp_button_rec_io)  (GPIO\_NUM\_36)<br> |
| define  | [**BSP\_BUTTON\_SET\_TOUCH**](#define-bsp_button_set_touch)  (TOUCH\_PAD\_NUM9)<br> |
| define  | [**BSP\_BUTTON\_VOLDOWN\_TOUCH**](#define-bsp_button_voldown_touch)  (TOUCH\_PAD\_NUM4)<br> |
| define  | [**BSP\_BUTTON\_VOLUP\_TOUCH**](#define-bsp_button_volup_touch)  (TOUCH\_PAD\_NUM7)<br> |


## Structures and Types Documentation

### enum `bsp_button_t`

```c
enum bsp_button_t {
    BSP_BUTTON_REC = 0,
    BSP_BUTTON_MODE,
    BSP_BUTTON_PLAY,
    BSP_BUTTON_SET,
    BSP_BUTTON_VOLUP,
    BSP_BUTTON_VOLDOWN,
    BSP_BUTTON_NUM
};
```


## Functions Documentation

### function `bsp_iot_button_create`

_Initialize all buttons._
```c
esp_err_t bsp_iot_button_create (
    button_handle_t btn_array,
    int *btn_cnt,
    int btn_array_size
) 
```


Returned button handlers must be used with espressif/button component API



**Note:**

For LCD panel button which is defined as BSP\_BUTTON\_MAIN, bsp\_display\_start should be called before call this function.



**Parameters:**


* `btn_array` Output button array 
* `btn_cnt` Number of button handlers saved to btn\_array, can be NULL 
* `btn_array_size` Size of output button array. Must be at least BSP\_BUTTON\_NUM 


**Returns:**



* ESP\_OK All buttons initialized
* ESP\_ERR\_INVALID\_ARG btn\_array is too small or NULL
* ESP\_FAIL Underlaying iot\_button\_create failed






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

### Leds API Reference

## Structures and Types

| Type | Name |
| ---: | :--- |
| enum  | [**bsp\_led\_t**](#enum-bsp_led_t)  <br> |
| typedef enum bsp\_led\_t | [**bsp\_led\_t**](#typedef-bsp_led_t)  <br> |

## Functions

| Type | Name |
| ---: | :--- |
|  esp\_err\_t | [**bsp\_led\_set**](#function-bsp_led_set) (const bsp\_led\_t led\_io, const bool on) <br>_Turn LED on/off._ |
|  esp\_err\_t | [**bsp\_leds\_init**](#function-bsp_leds_init) (void) <br>_Set LED's GPIOs as output push-pull._ |



## Structures and Types Documentation

### enum `bsp_led_t`

```c
enum bsp_led_t {
    BSP_LED_GREEN = GPIO_NUM_22
};
```

### typedef `bsp_led_t`

```c
typedef enum bsp_led_t bsp_led_t;
```


## Functions Documentation

### function `bsp_led_set`

_Turn LED on/off._
```c
esp_err_t bsp_led_set (
    const bsp_led_t led_io,
    const bool on
) 
```


**Parameters:**


* `led_io` LED io 
* `on` Switch LED on/off 


**Returns:**



* ESP\_OK Success
* ESP\_ERR\_INVALID\_ARG Parameter error
### function `bsp_leds_init`

_Set LED's GPIOs as output push-pull._
```c
esp_err_t bsp_leds_init (
    void
) 
```


**Returns:**



* ESP\_OK Success
* ESP\_ERR\_INVALID\_STATE Could not init IO expander
* ESP\_ERR\_INVALID\_ARG Parameter error



