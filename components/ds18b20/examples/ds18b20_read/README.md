# DS18B20 sensor example

This example shows how to use the 1-Wire temperature sensor DS18B20.

### Hardware Required

* An ESP development board that supports the selected 1-Wire backend (`RMT` or `UART`)
* A DS18B20 sensor connected to the configured 1-Wire GPIO (default: GPIO0)
* An USB cable for power supply and programming

### Configuration

Run `idf.py menuconfig`, then open:

* `Example Configuration` -> `1-Wire backend` to choose `RMT` or `UART`
* `Example Configuration` -> `1-Wire bus GPIO number` to select bus pin
* `Example Configuration` -> `Enable internal pull-up resistor on bus GPIO` as needed
* `Example Configuration` -> `UART port number (for UART backend)` when UART backend is selected

### Example Output

```text
...
I (297) main_task: Calling app_main()
I (297) example: 1-Wire bus installed on GPIO0 by RMT backend
I (297) example: Device iterator created, start searching...
I (407) example: Found a DS18B20[0], address: 070822502019FC28
I (517) example: Found a DS18B20[1], address: FC0921C076034628
I (517) example: Max DS18B20 number reached, stop searching...
I (517) example: Searching done, 2 DS18B20 device(s) found
I (2327) example: temperature read from DS18B20[0]: 26.69C
I (2337) example: temperature read from DS18B20[1]: 26.25C
I (4147) example: temperature read from DS18B20[0]: 26.69C
I (4157) example: temperature read from DS18B20[1]: 26.31C
I (5967) example: temperature read from DS18B20[0]: 26.69C
I (5977) example: temperature read from DS18B20[1]: 26.31C
...
```
