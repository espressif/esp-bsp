# Component: MAG3110

[![Component Registry](https://components.espressif.com/components/espressif/mag3110/badge.svg)](https://components.espressif.com/components/espressif/mag3110)

* I2C driver and definition of MAG3110 3-axis digital magnetometer
* See [datasheet](https://www.nxp.com/docs/en/data-sheet/MAG3110.pdf)

## Instructions and details
* Interrupt mode via `INT` pin is not supported. User must periodically read the data
* Before reading new data from MAG3110 a calibration is encouraged to eliminate infulences of hard-iron and PCB
* During the calibration, user must rotate the sensor in every axis to guarantee accurate calibration

## Code snippet
```c
#include "mag3110.h"

mag3110_result_t mag_induction; // in units of 0.1[uT]

mag3110_handle_t mag3110_dev = mag3110_create(I2C_NUM_1);
mag3110_calibrate(mag3110_dev, 10000);
mag3110_start(mag3110_dev, MAG3110_DR_OS_10_128);

mag3110_get_magnetic_induction(mag3110_dev, &mag_induction);
ESP_LOGI("MAG3110 snippet", "mag_x:%i, mag_y:%i, mag_z:%i", mag_induction.x, mag_induction.y, mag_induction.z);
```
