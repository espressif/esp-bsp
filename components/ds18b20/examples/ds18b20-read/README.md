# DS18B20 sensor example

This example shows how to use the 1-Wire temperature sensor [DS18B20](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf) component.

## How to use the example

To add the component latest version to a project, you can use the following command:

```bash
idf.py add-dependency "espressif/ds18b20"
```

To create a new project from this example, use:

```bash
idf.py create-project-from-example "espressif/ds18b20:ds18b20-read"
```

### Hardware Required

* An ESP development board with RMT peripheral (e.g ESP32, ESP32-C3, ESP32-S3, etc)
* An DS18B20 sensor connected to GPIO 18. To use a different pin, modify `EXAMPLE_ONEWIRE_BUS_GPIO` in *ds18b20-read.c* file.
* An USB cable for power supply and programming

### Example Output

```bash
...
I (336) DS18B20: Device iterator created, start searching...
I (456) DS18B20: Found a DS18B20[0], address: 990417C1D080FF28
I (456) DS18B20: Searching done, 1 DS18B20 device(s) found
I (456) main_task: Returned from app_main()
I (1266) DS18B20: temperature read from DS18B20[0]: 27.94C
I (4076) DS18B20: temperature read from DS18B20[0]: 27.81C
I (6886) DS18B20: temperature read from DS18B20[0]: 27.75C
I (9696) DS18B20: temperature read from DS18B20[0]: 27.69C
I (12506) DS18B20: temperature read from DS18B20[0]: 27.62C
I (15316) DS18B20: temperature read from DS18B20[0]: 27.56C
I (18126) DS18B20: temperature read from DS18B20[0]: 27.44C
I (20936) DS18B20: temperature read from DS18B20[0]: 27.38C
I (23746) DS18B20: temperature read from DS18B20[0]: 27.31C
...
```
