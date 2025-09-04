# DS18B20 sensor example

This example shows how to use the 1-Wire temperature sensor DS18B20.

### Hardware Required

* An ESP development board with RMT peripheral (e.g ESP32, ESP32-C3, ESP32-S3, etc)
* An DS18B20 sensor connected to GPIO 18. To use a different pin, modify `EXAMPLE_ONEWIRE_BUS_GPIO` in [source file](main/ds18b20_example_main.c)
* An USB cable for power supply and programming

### Example Output

```text
...
I (297) main_task: Calling app_main()
I (297) example: 1-Wire bus installed on GPIO18
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
