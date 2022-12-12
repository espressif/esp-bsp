# Component: HTS221

[![Component Registry](https://components.espressif.com/components/espressif/hts221/badge.svg)](https://components.espressif.com/components/espressif/hts221)

I2C driver and definition of HTS221 humidity and temperature sensor.

See [HTS221 datasheet](https://www.st.com/resource/en/datasheet/hts221.pdf).

## Operation modes
HTS221 supports autonomous sampling with user defined output data rate, as well as one-shot data acquisition (see `hts221_odr_t` data type).

New data from HTS221 can be obtained in two modes:

1. Polling
2. Data Ready (DRDY) interrupt driven

> Note: The user is responsible for initialization and configuration of I2C bus.

### Polling mode
After calling `hts221_create()` and `hts221_init()` the user is responsible for reading out new samples from HTS221.

If autonomous sampling was configured, it is enough to call `hts221_get_temperature()` and/or `hts221_get_humidity()` periodically.

If one-shot sampling was configured, the sampling must be first triggered by `hts221_start_oneshot()`.

### Data Ready (DRDY) interrupt driven
In this mode HTS221 asserts DRDY pin when new data of either temperature or humidity is available. The data is read out in a separate FreeRTOS task.

> Note: This mode is only available if the DRDY pin of HTS221 is connected to MCU.

After calling `hts221_create()` and `hts221_init()`, the DRDY mode is enabled by calling `hts221_drdy_enable()` which registers a user's new data function callback.


