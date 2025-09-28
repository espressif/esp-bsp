# _LVGL Rotate_

(See the README.md file in the upper level 'examples' directory for more information about examples.)

This example demonstrates rotation of the screen on an eInk display. It displays a long text string
at rotations of 0, 90, 180 and 270 degrees. It will work on a Waveshare 2.7" eInk display that natively
has screen dimensions of 176W x 264H; it should also work on a Waveshare 1.54" display with dimensions
200 x 200 (not tested by me). Changing the width and height constants (search main.c for DISPLAY_X)
should work with other displays that have SSD1680 or SSD1681 controllers.

Rotating eInk displays is not the same as more dyanmic displays such as LCD.
The screen can be rotated only in multiples of 90 degrees: 0, 90, 180, 270.
An eInk display should not be updated often, in fact the manufacturers (Waveshare and GooDisplay)
state that displays should be rewritten no more often than 3 minutess. During development you can
rewrite more often, but this can shorten the lifetime of the display.
With some eInk displays it is possible to only update portions of the screen, but manufacturers
warn that this should not be done for too many cycles - eventually you should refresh the entire screen.
This demo only does full screen refreshes.

An eInk display should usually be thought of as static - for example you will display
a weather forecast and update it every 15 minutes.

Let's discuss eInk rotation.

- eInk displays handled by this example have either a SSD1680 or SSD1681 controller.

- The controller cannot perform hardware rotation (unlike some LCD).

- Some eInk display examples in ESP-IDF and ESP-BSP use a 1.54 inch square display, 200x200 pixels.
You can rotate a square screen by multiples of 90 degrees by using mirror and swap axes operations.

- If the display is not square, then most rotation schemes result in a clipped output.
Swapping X&Y axes is poorly defined.

- LVGL can do software rotation (sw_rotate=true), but it is unusable for our needs. Why?
Suppose the display is 200W x 100H. When you rotate by 90, many algorithms don't know
where to put the pixels from 101 to 200, so the screen image becomes a square measuring 100 x 100
and the screen image is clipped.

Waveshare eInk displays have a default orientation, portrait or landscape. AFAIK,
the SSD1680 controller cannot be programmed to change the orientation (sure would be nice).
Many non-square displays are portrait, even though many applications would prefer landscape.
So what to do?

- The controller does not perform hardware rotation.

- LVGL software rotation has problems (see below).

- Since we want to change orientation between portrait and landscape, we need a function that
moves the pixels from rows to columns.

## What are the problems with LVGL sw_rotate?

If **sw_rotate=true**, then **full_refresh** must be *false*. This results in the full screen image
getting rotated in pieces and written to the device by DMA. While this is fine for an LCD,
eInk displays need to refresh when each DMA transfer completes, resulting in flashing of the disdplay
multiple times until all the chunks have been transferred. Furthermore, **sw_rotate** will clip the screen
image as mentioned above.

## How can we change from portrait to landscape orientation?

The answer is to use a *canvas*. We create a canvas in the desired orientation (for example: landscape)
and write to it. If you write a landscape canvas to a portrait display, it will be clipped. If you rotate
the landscape canvas by 90 or 270 degrees, you expect that it should appear on the screen without clipping.
You should be able to rotate a canvas with **lv_canvas_transform()**, but I was unable to get it to work
with LVGL v. 8.3.0. Therefore I wrote function **rotate_buffer()** to perform a 90 degree rotation.
270 degree roatation is performed with 90 degree rotation plus mirror on both X and Y axes.

## How to use example
We encourage the users to use the example as a template for new projects.
A recommended way is to follow the instructions on a [docs page](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html#start-a-new-project).

## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── main.c
│   └── lvgl_demo_ui.c
└── README.md                  This is the file you are currently reading
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.
