# DS18B20 sensor example

This example shows how to use the DS18B20 sensor driver with sensor hub component.
The CONFIG_DS18B20_SENSOR_HUB configuration option must be set for this project to compile because the sensor hub component is not included by default.
This can be achieved by using the provided sdkconfig.

### Hardware Required

* An ESP development board that supports UART 1-Wire backend
* A DS18B20 sensor connected to the configured 1-Wire GPIO (default: GPIO0)
* An USB cable for power supply and programming

### Configuration

Run `idf.py menuconfig`, then open:

* `Example Configuration` -> `1-Wire bus GPIO number` to select bus pin
* `Example Configuration` -> `Enable internal pull-up resistor on bus GPIO` as needed
* `Example Configuration` -> `UART port number (for UART backend)` when UART backend is selected
* `Example Configuration` -> `Temperature measurement period` to choose a different delay between measurements

### Example Output

```text
I (266) main_task: Calling app_main()
I (266) example: 1-Wire bus installed on GPIO10 by UART backend (UART1)
I (276) SENSOR_HUB: task: sensor_default_task created!
I (276) SENSOR_LOOP: event loop created succeed
I (286) SENSOR_LOOP: register a new handler to event loop succeed
I (286) SENSOR_HUB: Sensor created, Task name = SENSOR_HUB, Type = HUMITURE, Sensor Name = sensor_hub_ds18b20, Mode = MODE_POLLING, Min Delay = 500 ms
I (306) main_task: Returned from app_main()
I (306) SENSOR_HUB: Timestamp = 72552 - sensor_hub_ds18b20_0x16 STARTED
I (1616) SENSOR_HUB: Timestamp = 1382245 - sensor_hub_ds18b20_0x16 TEMP_DATA_READY - temperature=26.56
I (2426) SENSOR_HUB: Timestamp = 2192222 - sensor_hub_ds18b20_0x16 TEMP_DATA_READY - temperature=26.56
I (3236) SENSOR_HUB: Timestamp = 3002218 - sensor_hub_ds18b20_0x16 TEMP_DATA_READY - temperature=26.56
I (4046) SENSOR_HUB: Timestamp = 3812217 - sensor_hub_ds18b20_0x16 TEMP_DATA_READY - temperature=26.56
I (4856) SENSOR_HUB: Timestamp = 4622226 - sensor_hub_ds18b20_0x16 TEMP_DATA_READY - temperature=26.62
I (5666) SENSOR_HUB: Timestamp = 5432225 - sensor_hub_ds18b20_0x16 TEMP_DATA_READY - temperature=26.62
```

### References
[Sensor hub documentation](https://docs.espressif.com/projects/esp-iot-solution/en/latest/sensors/sensor_hub.html)
