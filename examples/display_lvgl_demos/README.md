| Supported Targets | ESP32-S3-LCD-EV-BOARD |
| ----------------- | --------------------- |

# Display LVGL Demos

This example shows LVGL internal demos with RGB LCD.

### Configurations

Here are some useful configurations in menuconfig that can be customed by user:

* `BSP_LCD_SUB_BOARD`: Choose a LCD sub-board according to hardware
* `BSP_DISPLAY_LVGL_BUF_CAPS`: Choose the memory type of LVGL buffer. Internal memory is more fast.
* `BSP_DISPLAY_LVGL_BUF_HEIGHT`: Set the height of LVGL buffer, and its width is equal to LCD's width.
* `BSP_DISPLAY_LVGL_AVOID_TEAR`: Avoid tearing effect by using double buffers. Need to enable `BSP_LCD_RGB_DOUBLE_BUFFER` and `BSP_LCD_RGB_REFRESH_TASK_ENABLE` first.

### Hardware Required

ESP32-S3-LCD-EV-BOARD with 800x480 or 480x480 LCD sub-board.
