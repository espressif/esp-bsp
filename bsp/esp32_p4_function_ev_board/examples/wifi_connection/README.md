# ESP32-P4 Function Evaluation Board Wi-Fi Connection Example

This example demonstrates how to connect to a Wi-Fi network using the ESP32-P4 Function Evaluation Board, utilizing the on-board ESP32-C6 module for connectivity.

## How to use example

### Getting the example

The easiest way to get this example is to download it from ESP Component Registry:

```shell
idf.py create-project-from-example espressif/esp32_p4_function_ev_board:wifi_connection
```

Alternatively, you can clone esp-bsp repository and navigate to the directory of this example.

### Hardware Required

This example runs on the ESP32-P4 Function Evaluation Board (v1.1 or later).

### Configure the project

Run `idf.py menuconfig` and set the Wi-Fi SSID and password in the Example Connection Configuration menu (`CONFIG_EXAMPLE_WIFI_SSID` and `CONFIG_EXAMPLE_WIFI_PASSWORD`).

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```shell
idf.py flash monitor
```

(To exit the serial monitor, type `Ctrl-]`.)

## Troubleshooting

If Wi-Fi connection fails, please check that ESP32-C6 module on the ESP32-P4 Function Evaluation Board is flashed with esp-hosted firmware. See [here](https://github.com/espressif/esp-hosted/blob/feature/esp_as_mcu_host/docs/esp32_p4_function_ev_board.md#52-using-esp-prog) for instructions on how to flash the firmware to ESP32-C6.

