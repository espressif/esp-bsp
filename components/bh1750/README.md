# Component: BH1750

[![Component Registry](https://components.espressif.com/components/espressif/bh1750/badge.svg)](https://components.espressif.com/components/espressif/bh1750)
![maintenance-status](https://img.shields.io/badge/maintenance-as--is-yellow.svg)

:warning: **BH1750 component is provided as-is with no further development and compatibility maintenance**

* This component will show you how to use I2C module read external i2c sensor data, here we use BH1750 light sensor (GY-30 module).
* BH1750 measurement mode:
    * one-time mode: BH1750 just measure only one time when received the one time measurement command, so you need to send this command when you want to get intensity value every time
    * continuous mode: BH1750 will measure continuously when received the continuously measurement command, so you just need to send this command once, and than call `bh1750_get_data()` to get intensity value repeatedly.
## Notice:
* BH1750 has different measurement time for each measurement mode. Measurement time can be changed by calling `bh1750_change_measure_time()`.
