# Component: ES7210

ES7210 high performance four channels audio ADC features:
* High performance multi-bit delta-sigma audio ADC
* 102 dB signal to noise ratio
* -85 dB THD+N
* 24-bit, 8 to 200 kHz sampling frequency
* I2S/PCM master or slave serial data port
* Support TDM
* 256/384Fs, USB 12/24 MHz and other non standard audio system clocks
* Low power standby mode

This driver implements only the I2C interface to ES7210 which is used for configuration of the device.
User is responsible for initialization of I2S port and start I2S transaction to stream audio from the ES7210.

* See [ES7210 datasheet](http://www.everest-semi.com/pdf/ES7210%20PB.pdf)
