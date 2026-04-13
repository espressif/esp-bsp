# Display + camera example

This example shows how you can obtain frames from camera a display them on LCD.

By default, the camera interface has following settings:
* Double-buffering (1 frame is being flushed onto the display, while another one is being fetched from the camera)
* Frames in external PSRAM: ESP32-S2 has limited internal RAM, so frames from camera are saved to external RAM.
* EDMA is used for transferring data from camera to the PSRAM
* RGB565 color and QVFGA definition. We use the same image parameters as the display has, so we don't have to convert image formats between camera and the display.

This very simple example continuously fetches image frames from camera and displays them on LCD using LVGL's canvas widget.

### Hardware Required

Kaluga kit with its camera module.

## Build and Flash

To build and flash the example for a specific `{board}` and `{port}`, use the following command:

```
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.{board} -p {port} flash monitor
```
Make sure the correct board name is set in the `main/idf_component.yml` file under the `dependencies` section.


## Launch Example

You can also try this example using ESP Launchpad:

<a href="https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_camera-">
    <img alt="Try it with ESP Launchpad" src="https://espressif.github.io/esp-launchpad/assets/try_with_launchpad.png" width="250" height="70">
</a>
