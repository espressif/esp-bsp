# Display + camera example

This example shows how you can obtain frames from MIPI-CSI camera and display them on LCD.
This very simple example continuously fetches image frames from camera and displays them on LCD using LVGL's canvas widget.

### Hardware Required

Any board with MIPI-CSI camera.

- ESP32-P4-EYE
    - Camera sensor: OV2710
- ESP32-P4-Function-EV-Board
    - Camera sensor: SC2336 (forward direction ribbon cable)
    - Camera sensor: OV5647 Raspberry Pi (reverse direction ribbon cable)

## Build and Flash

To build and flash the example for a specific `{board}` and `{port}`, use the following command:

```
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.{board} -p {port} flash monitor
```
Make sure the correct board name is set in the `main/idf_component.yml` file under the `dependencies` section.


## Launch Example

You can also try this example using ESP Launchpad:

<a href="https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_camera_new-">
    <img alt="Try it with ESP Launchpad" src="https://espressif.github.io/esp-launchpad/assets/try_with_launchpad.png" width="250" height="70">
</a>
