# BSP: Display USB HID Example

## Overview

This example demonstrates usage of the USB HID (keyboard and mouse) with M5Stack Tab5 Board.

## How to use the example

### Hardware Required

* Board [M5Stack Tab5](https://github.com/espressif/esp-bsp/tree/master/bsp/m5stack_tab5)
  - connect 5V USB-C power connector
  - connect USB keyboard/mouse to the right "USB-A Host" connector

## USB Stack Selection

You can select between USB stacks in menuconfig `Example Configuration`:
- Espressif's USB Stack
- CherryUSB

## Build and Flash

To build and flash the example for a specific `{board}` and `{port}`, use the following command:

```
idf.py menuconfig
idf.py flash monitor
```
