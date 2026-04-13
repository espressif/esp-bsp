# Bosch BMI270 inertial measurement unit driver

[![Component Registry](https://components.espressif.com/components/espressif/bmi270/badge.svg)](https://components.espressif.com/components/espressif/bmi270)
![maintenance-status](https://img.shields.io/badge/maintenance-actively--maintained-green.svg)

## Features

- Reading of the chip ID.
- Configuring output data rate and range of the onboard accelerometer and gyroscope.
- Reading normalized float values of acceleration and rotation angle.

### Limitations

- Missing implementation of the SPI interface.
- Advanced features such as gesture control, FIFO buffering, interrupt on data ready and auxilary interface are not supported.

## Integration guide

This driver, along with many other components from this repository, can be used as a package from [Espressif's IDF Component Registry](https://components.espressif.com). To include this driver in your project, run the following idf.py from the project's root directory:

``` sh
idf.py add-dependency "espressif/bmi270==*"
```

## Usage guide

### Low level driver
When using the low level driver directly then firstly initialize it by providing a valid low level driver configuration:
``` c
const bmi270_driver_config_t driver_config = {
    .addr = addr,
    .interface = BMI270_USE_I2C,
    .i2c_bus = i2c_bus_handle
};
bmi270_create(&driver_config, &bmi270_handle_pointer);
```
Start acquisition with a valid measurement configuration:
``` c
const bmi270_config_t measurement_config = {
    .acce_odr = BMI270_ACC_ODR_X,
    .acce_range = BMI270_ACC_RANGE_X,
    .gyro_odr = BMI270_GYR_ODR_X,
    .gyro_range = BMI270_GYR_RANGE_X
};
bmi270_start(bmi270_handle_pointer, &measurement_config);
```
After a completed measurement data can be read using:
``` c
bmi270_get_acce_data(bmi270_handle_pointer, &x, &y, &z);
```
Disable measurements and clean up the BMI270 driver using:
``` c
bmi270_stop(bmi270_handle_pointer);
bmi270_delete(bmi270_handle_pointer);
bmi270_handle_pointer = NULL;
```

### Espressif sensor hub

This driver can be used with [sensor-hub](https://docs.espressif.com/projects/esp-iot-solution/en/latest/sensors/sensor_hub.html) abstraction.
Create a BMI270 sensor instance using a valid configuration:
``` c
iot_sensor_create("sensor_hub_bmi270", &bmi270_sensor_hub_configuration, sensor_handle_pointer);
```

## Driver structure

```
BMI270
├── CMakeLists.txt                  # Driver component CMake file
├── README.md
├── idf_component.yml               # Driver component configuration file
├── include                         # Public include folder
│   └── bmi270.h
├── include_priv                    # Private include folder
│   └── bmi270_priv.h
├── license.txt
├── src
│   ├── bmi270.c                    # Low  level driver source file
│   └── bmi270_sensor_hub.c         # Sensor-hub wrapper
└── test_app                        # Test app for the low level driver
    ├── CMakeLists.txt
    ├── main
    │   ├── CMakeLists.txt
    │   ├── idf_component.yml
    │   └── test_app_bmi270.c
    └── sdkconfig.defaults
```

## See also
[BMI270 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf)
[ESP-IDF component registry documentation](https://docs.espressif.com/projects/idf-component-manager/en/latest/)
[Sensor hub documentation](https://docs.espressif.com/projects/esp-iot-solution/en/latest/sensors/sensor_hub.html)