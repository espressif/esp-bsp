# ICM42607/ICM42670 6-Axis MotionTracking (Accelerometer and Gyroscope)

[![Component Registry](https://components.espressif.com/components/espressif/icm42670/badge.svg)](https://components.espressif.com/components/espressif/icm42670)

C driver for Invensense ICM42607/ICM42670 6-axis gyroscope and accelerometer based on I2C communication.

## Features

- Get 3-axis accelerometer and 3-axis gyroscope data, either raw or as floating point values. 
- Read temperature from ICM42607/ICM42670 internal temperature sensor.
- Configure gyroscope and accelerometer sensitivity.
- ICM42607/ICM42670 power down mode.

## Limitations

- Only I2C communication is supported.
- Driver has not been tested with ICM42670 yet.

## Get Started

This driver, along with many other components from this repository, can be used as a package from [Espressif's IDF Component Registry](https://components.espressif.com). To include this driver in your project, run the following idf.py from the project's root directory:

```
    idf.py add-dependency "espressif/icm42670==1.0.0"
```

Another option is to manually create a `idf_component.yml` file. You can find more about using .yml files for components from [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## See Also
* [MPU6050 datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/icm-42670-p/)


