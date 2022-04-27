| Supported Boards | Azure IoT kit |
| ---------------- | ------------- |

# BSP: Sensors Example

This is an example usage of Azure-IoT-Kit board.

## Sensors
All sensors are sampled and results are shown on OLED display.
User can switch between pages by pressing KEY_IO0 button.

### Magnetometer calibration
At the start of the program, magnetometer calibration is performed for 10 seconds.
Turn the board in every axis during this time to achieve best magnetometer results.

## LED and buzzer
On every press of KEY_IO0 button, the buzzer beeps and AZURE LED blinks.

## uSD card
If a uSD card is successfully mounted a hello.txt file is created and WIFI LED is turned on.
