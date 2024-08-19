# QMA6100P Driver Component

[![Component Registry](https://components.espressif.com/components/espressif/qma6100p/badge.svg)](https://components.espressif.com/components/espressif/qma6100p)

C driver for QST QMA6100P 3-axis accelerometer based on I2C communication.

## Features

- Get 3-axis accelerometer data, either raw or as floating point values. 
- Configure accelerometer sensitivity.
- Support for QMA6100P interrupt generation when data ready (occurs each time a write to all sensor data registers has been completed).

## Important Notes

- Keep in mind that QMA6100P I2C address depends on the level of its AD0 pin (1) (0x12 when low, 0x13 when high).
- In order to receive QMA6100P interrupts, its INT pins (5, 6) must be connected to a GPIO on the ESP32.

## Limitations

- Only I2C communication is supported.
- If QMA6100P interrupts are used, it is recommended to not read data using I2C directly from the ISR.

## Get Started

This driver, along with many other components from this repository, can be used as a package from [Espressif's IDF Component Registry](https://components.espressif.com). To include this driver in your project, run the following idf.py from the project's root directory:

```
    idf.py add-dependency "espressif/qma6100p"
```

Another option is to manually create a `idf_component.yml` file. You can find more about using .yml files for components from [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## See Also
* [QMA6100P datasheet](https://www.qstcorp.com/upload/pdf/202203/13-52-20%20QMA6100P%20Preliminary%20Datasheet%20Rev.%20A1_SPImode03.pdf)
