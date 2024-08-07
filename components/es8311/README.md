# Component: ES8311

[![Component Registry](https://components.espressif.com/components/espressif/es8311/badge.svg)](https://components.espressif.com/components/espressif/es8311)
![maintenance-status](https://img.shields.io/badge/maintenance-deprecated-red.svg)

:warning: **This driver is deprecated. Please use codec drivers from [esp_codec_dev](https://components.espressif.com/components/espressif/esp_codec_dev)**

ES8311 low power mono audio codec features:
* High performance and low power multi-bit delta-sigma audio ADC and DAC
* I2S/PCM master or slave serial data port
* ADC: 24-bit, 8 to 96 kHz sampling frequency 
* ADC: 100 dB signal to noise ratio, -93 dB THD+N 
* DAC: 24-bit, 8 to 96 kHz sampling frequency 
* DAC: 110 dB signal to noise ratio, -80 dB THD+N

This driver implements only the I2C interface to ES8311 which is used for configuration of the device.
User is responsible for initialization of I2S port and start I2S transaction to stream audio in/from the ES8311.

* See [ES8311 datasheet](http://www.everest-semi.com/pdf/ES8311%20PB.pdf)
