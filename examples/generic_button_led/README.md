# BSP: Generic Button and LED Example

This is a minimalistic button and LED example with using Generig BSP.
In few function calls it sets up the use button and LED.

Available LEDs and buttons are initialized with preconfigured settings. The first LED is set to breathing and the first button can change LED blinking effect.

# Build with predefined configuration

Predefined configurations are saved in [generic_button_led](https://github.com/espressif/esp-bsp/tree/master/examples/generic_button_led) example.

```
    idf.py -p COM4 -D "SDKCONFIG_DEFAULTS=sdkconfig.esp32_s3_devkitc_1" flash monitor
```

**Note:** If you changing configuration, please remove `build` folder and `sdkconfig` file before build with new configuration.
