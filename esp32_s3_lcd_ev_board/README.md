# BSP: ESP32-S3-LCD-EV-BOARD

* [User Guide](https://github.com/espressif/esp-dev-kits/tree/master/esp32-s3-lcd-ev-board#readme)

![](https://docs.espressif.com/projects/espressif-esp-dev-kits/zh_CN/latest/_images/board_resource.png)

ESP32-S3-LCD-EV-BOARD is a development board for evaluating and verifying ESP32-S3 screen interactive applications. It has the functions of touch screen interaction and voice interaction. The development board has the following characteristics:

* Onboard ESP32-S3-WROOM-1 module, with built-in 16 MB Flash + 8 MB PSRAM
* Onboard audio codec + audio amplifier
* Onboard dual microphone pickup
* USB type-C interface download and debugging
* It can be used with different screen sub boards, and supports `RGB`, `8080`, `SPI`, `I2C` interface screens, as below:

| Board Name                 | Screen Size (inch) | Resolution | LCD Driver IC (Interface) | Touch Driver IC |                                                                          Schematic                                                                           | Support |
| -------------------------- | ------------------ | ---------- | ------------------------- | --------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------- |
| ESP32-S3-LCD_Ev_Board_SUB1 | 0.9                | 128 x 64   | SSD1315 (I2C)             | *               | [link](https://github.com/espressif/esp-dev-kits/blob/master/docs/_static/schematics/esp32-s3-lcd-ev-board/SCH_ESP32-S3-LCD_Ev_Board_SUB1_V1.0_20220617.pdf) | Not yet |
|                            | 2.4                | 320 x 240  | ST7789V (SPI)             | XTP2046         |                                                                                                                                                              | Not yet |
| ESP32-S3-LCD_Ev_Board_SUB2 | 3.5                | 480 x 320  | ST7796S (8080)            | GT911           | [link](https://github.com/espressif/esp-dev-kits/blob/master/docs/_static/schematics/esp32-s3-lcd-ev-board/SCH_ESP32-S3-LCD_Ev_Board_SUB2_V1.0_20220615.pdf) | Not yet |
|                            | 3.95               | 480 x 480  | GC9503CV (RGB)            | FT5x06          |                                                                                                                                                              | Yes     |
| ESP32-S3-LCD_Ev_Board_SUB3 | 4.3                | 800 x 480  | Unkonw (RGB)              | GT1151          | [link](https://github.com/espressif/esp-dev-kits/blob/master/docs/_static/schematics/esp32-s3-lcd-ev-board/SCH_ESP32-S3-LCD_Ev_Board_SUB3_V1.0_20220617.pdf) | Yes     |
