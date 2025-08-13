# BSP: Sensors Example

This is an example usage of Azure-IoT-Kit board.

## Sensors
All sensors are sampled and results are shown on OLED display.
User can switch between pages by pressing KEY_IO0 button.

### Magnetometer calibration
At the start of the program, magnetometer calibration is performed for 10 seconds.
Turn the board in every axis during this time to achieve best magnetometer results.

## LED and buzzer
On every press of KEY_IO0 button, the buzzer beeps and AZURE LED blinks.

## uSD card
If a uSD card is successfully mounted a hello.txt file is created and WIFI LED is turned on.

## Build and Flash

To build and flash the example for a specific `{board}` and `{port}`, use the following command:

```
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.{board} -p {port} flash monitor
```
Make sure the correct board name is set in the `main/idf_component.yml` file under the `dependencies` section.

## Launch Example

You can also try this example using ESP Launchpad:

<a href="https://espressif.github.io/esp-launchpad/?flashConfigURL=https://espressif.github.io/esp-bsp/config.toml&app=display_sensors-">
    <img alt="Try it with ESP Launchpad" src="https://espressif.github.io/esp-launchpad/assets/try_with_launchpad.png" width="250" height="70">
</a>
