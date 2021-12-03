# Component: BH1750

* This component will show you how to use I2C module read external i2c sensor data, here we use BH1750 light sensor(GY-30 module).
* BH1750 measurement mode:
    * one-time mode: bh1750 just measure only one time when receieved the one time measurement command, so you need to send this command when you want to get intensity value every time
    * continuous mode: bh1750 will measure continuously when receieved the continuously measurement command, so you just need to send this command once, and than call `bh1750_get_data()` to get intensity value repeatedly.
## Notice:
* Bh1750 has different measurement time in different measurement mode, and also, measurement time can be changed by call `bh1750_change_measure_time()`
