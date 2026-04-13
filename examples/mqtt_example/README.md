# BSP: MQTT Example

This example collects data from sensor and publishes them to configured MQTT server.

## Configuration
In `idf.py menuconfig` -> Example configuration, please configure your WiFi SSID and password and MQTT broker URL.

## Operation
Application collects sensor data of ambient temperature, humidity, luminescence and pressure.
After successful connection to MQTT sensor, both LEDs are turned on and data are periodically published to MQTT and shown on display.

## Build and Flash

To build and flash the example for a specific `{board}` and `{port}`, use the following command:

```
idf.py -D SDKCONFIG_DEFAULTS=sdkconfig.bsp.{board} -p {port} flash monitor
```
Make sure the correct board name is set in the `main/idf_component.yml` file under the `dependencies` section.
