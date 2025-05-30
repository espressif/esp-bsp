# Changelog

## [Unreleased]
### Features
- Added option to place LVGL task stack to external RAM

## 2.6.0

### Features
- Scaling feature in touch
- Added support for PPA rotation in LVGL9 (available for ESP32-P4)

## 2.5.0

### Features (Functional change for button v4 users)
- Updated LVGL port for using IoT button component v4 (LVGL port not anymore creating button, need to be created in app and included handle to LVGL port)

### Fixes
- Fixed buffer size by selected color format
- Fixed memory leak in LVGL8 in display removing https://github.com/espressif/esp-bsp/issues/462
- Fixed draw buffer alignment

## 2.4.4

### Features
- Changed queue to event group in main LVGL task for speed up https://github.com/espressif/esp-bsp/issues/492
- Reworked handling encoder (knob) https://github.com/espressif/esp-bsp/pull/450

### Fixes
- Fixed a crash when esp_lvgl_port was initialized from high priority task https://github.com/espressif/esp-bsp/issues/455
- Allow to swap bytes when used SW rotation https://github.com/espressif/esp-bsp/issues/497

## 2.4.3

### Fixes
- Fixed a display context pointer bug
- Fixed I2C example for using with LVGL9

### Features
- Support for LV_COLOR_FORMAT_I1 for monochromatic screen

## 2.4.2

### Fixes
- Fixed SW rotation in LVGL9.2
- Fixed freeing right buffers when error

## 2.4.1

### Fixes
- Fixed the issue of the DPI callback function not being initialized.

## 2.4.0

### Features
- Added support for direct mode and full refresh mode in the MIPI-DSI interface.
- Optimized avoid-tear feature and LVGL task.

## 2.3.3

### Features
- Updated RGB screen flush handling in LVGL9.

## 2.3.2

### Fixes
- Fixed rotation type compatibility with LVGL8.

## 2.3.1

### Fixes
- Fixed LVGL version resolution if LVGL is not a managed component
- Fixed link error with LVGL v9.2
- Fixed event error with LVGL v9.2

## 2.3.0

### Fixes
- Fixed LVGL port for using with LVGL9 OS FreeRTOS enabled
- Fixed bad handled touch due to synchronization timer task

### Features
- Added support for SW rotation in LVGL9

## 2.2.2

### Fixes
- Fixed missing callback in IDF4.4.3 and lower for LVGL port

## 2.2.1

### Fixes
- Added missing includes
- Fixed watchdog error in some cases in LVGL9

## 2.2.0

### Features
- Added RGB display support
- Added support for direct mode and full refresh mode

### Breaking changes
- Removed MIPI-DSI from display configuration structure - use `lvgl_port_add_disp_dsi` instead

## 2.1.0

### Features
- Added LVGL sleep feature: The esp_lvgl_port handling can sleep if the display and touch are inactive (only with LVGL9)
- Added support for different display color modes (only with LVGL9)
- Added script for generating C array images during build (depends on LVGL version)

### Fixes
- Applied initial display rotation from configuration https://github.com/espressif/esp-bsp/pull/278
- Added blocking wait for LVGL task stop during esp_lvgl_port de-initialization https://github.com/espressif/esp-bsp/issues/277
- Added missing esp_idf_version.h include

## 2.0.0

### Features

- Divided into files per feature
- Added support for LVGL9
- Added support for MIPI-DSI display

## 1.4.0

### Features

- Added support for USB HID mouse/keyboard as an input device

## 1.3.0

### Features

- Added low power interface

## 1.2.0

### Features

- Added support for encoder (knob) as an input device

## 1.1.0

### Features

- Added support for navigation buttons as an input device
