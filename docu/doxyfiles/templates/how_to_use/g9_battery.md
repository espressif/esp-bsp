## :battery: Battery

Some boards with battery support can measure the battery voltage using an ADC channel. BSP provides a simple API for this:

```
/* Initialize the battery voltage measurement */
bsp_voltage_init();

/* Read battery voltage in millivolts */
int voltage = bsp_voltage_battery_get();
```
