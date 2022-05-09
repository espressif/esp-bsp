| Supported Boards | Azure IoT kit |
| ---------------- | ------------- |

# BSP: MQTT Example

This example collects data from sensor and publishes them to configured MQTT server.

## Configuration
In `idf.py menuconfig` -> Example configuration, please configure your WiFi SSID and password and MQTT broker URL.

## Operation
Application collects sensor data of ambient temperature, humidity, luminescence and pressure.
After successful connection to MQTT sensor, both LEDs are turned on and data are periodically published to MQTT and shown on display.
