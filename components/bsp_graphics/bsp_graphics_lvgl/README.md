# BSP Graphics LVGL

This component provides an LVGL-based graphics layer for ESP-BSP. It is intentionally separated from the BSP to keep the base BSP lightweight and free of any graphical library dependencies.

If you want to use LVGL with a BSP, add this component alongside the BSP.

## Overview

The component integrates LVGL with the BSP display and input drivers and is built on top of esp_lvgl_port.

It handles:
- LVGL initialization and port configuration
- Display buffer allocation and management
- Input device registration (touch, buttons, etc.)
- LVGL task handling

The BSP itself only provides low-level display and touch initialization. This component builds on top of it and exposes a ready-to-use LVGL environment.

## Usage

There are two main ways to use this component:

### 1. Using BSP Selector

You can use the BSP selector tool to choose:

- your target BSP
- a graphical library (LVGL)

The required components (BSP + LVGL graphics layer) will be added automatically.

```yaml
dependencies:
  espressif/bsp_selector:
    version: "*"
```

### 2. Manual dependency (idf_component.yml)

Add both the BSP and this component to your project:

```yaml
dependencies:
  espressif/<your_bsp>:
    version: "*"
  espressif/bsp_graphics_lvgl:
    version: "*"
```

### Basic example

```c
lv_display_t *disp = bsp_display_start();

bsp_display_lock(portMAX_DELAY);

/* Create simple UI */
lv_obj_t *label = lv_label_create(lv_screen_active());
lv_label_set_text(label, "Hello LVGL");
lv_obj_center(label);

bsp_display_unlock();
```

## Thread Safety

LVGL is not thread-safe. All calls to LVGL APIs (lv_...) must be protected by a mutex:

```c
bsp_display_lock(portMAX_DELAY);

/* LVGL calls ... */

bsp_display_unlock();
```

Failure to lock the display before calling LVGL APIs may lead to race conditions or crashes.

## Initialization

To start the display with default settings:

```c
lv_display_t *disp = bsp_display_start();
```

For custom configuration:
```c
bsp_display_cfg_t cfg = {
    .buffer_size = ...,
    .double_buffer = true,
    .flags = {
        .buff_dma = 1,
        .buff_spiram = 1,
    },
};

lv_display_t *disp = bsp_display_start_with_config(&cfg);
```

This initializes:
- display bus (SPI, RGB, MIPI, etc.)
- display controller
- LVGL core and internal task

Note: The LCD backlight is not enabled automatically. Use BSP function `bsp_display_brightness_set()` if available.

## Input Devices

The input device (e.g. touch) is automatically initialized during display startup:

```c
lv_indev_t *indev = bsp_display_get_input_dev();
```

Returns NULL if the display has not been initialized.

## Display Control

Rotation:

```c
bsp_display_rotate(disp, LV_DISP_ROT_90);
```
