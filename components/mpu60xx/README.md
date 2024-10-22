# mpu60xx
I2C driver for MPU60xx family IMU. 

## Features  
  - Read Accelerometer, Gyrometer and Temperature sensor as raw or floating values.
  - Use in-buit Motion Detection feature using either interrupt or polling.

## Limitations
  - Does NOT support esp-idf Legacy I2C driver (documentation indicates it'll be removed in future releases).
  - Not tested with MPU6000. ( TRM indicates it should also work, same register set as MPU6050.)
  - Supports only I2C protocol ( Only MPU6000 has SPI capability.)

## Important Note
  - Use deferred processing to handle interrupts from MPU60xx, I2C driver require interrupt to read/write (see example).
  - INT pin of MPU60xx is required to be connected to esp32 gpio for interrupt. 

## Feature for next release (planned)
  MPU60xx has many more features which haven't been implemented in this library currently, but are planned to be added in further releases. The next intended feature is
 - Zero motion detection.
 
 
 
