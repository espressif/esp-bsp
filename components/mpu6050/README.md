# MPU6050 Driver Component

[![Component Registry](https://components.espressif.com/components/espressif/mpu6050/badge.svg)](https://components.espressif.com/components/espressif/mpu6050)

C driver for Invensense MPU6050 6-axis gyroscope and accelerometer based on I2C communication.

## Features

- Get 3-axis accelerometer and 3-axis gyroscope data, either raw or as floating point values. 
- Read temperature from MPU6050 internal temperature sensor.
- Configure gyroscope and accelerometer sensitivity.
- MPU6050 power down mode.
- Support for MPU6050 interrupt generation when data ready (occurs each time a write to all sensor data registers has been completed).  

## Important Notes

- Keep in mind that MPU6050 I2C address depends on the level of its AD0 pin (9) (0x68 when low, 0x69 when high).
- In order to receive MPU6050 interrupts, its INT pin (12) must be conneced to a GPIO on the ESP32. 

## Limitations

- Only I2C communication is supported.
- Driver has not been tested with MPU 6000 yet.
- 9-axis support through MPU6050 I2C aux is not supported.
- If MPU6050 interrupts are used, it is recommended to not read data using I2C directly from the ISR. 

## Get Started

This driver, along with many other components from this repository, can be used as a package from [Espressif's IDF Component Registry](https://components.espressif.com). To include this driver in your project, run the following idf.py from the project's root directory:

```
    idf.py add-dependency "espressif/mpu6050^1.1.1"
```

Another option is to manually create a `idf_component.yml` file. You can find more about using .yml files for components from [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## See Also
* [Sensors example, including the MPU6050 driver](https://github.com/espressif/esp-bsp/tree/master/examples/sensors_example)
* [MPU6050 datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf)
* [MPU6000 and MPU6050 register map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)
