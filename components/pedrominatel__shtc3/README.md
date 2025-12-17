# I2C Driver Component for the SHTC3 Temperature and Humidity Sensor

[![Component Registry](https://components.espressif.com/components/pedrominatel/sht2x/badge.svg)](https://components.espressif.com/components/pedrominatel/shtc3)

The SHTC3 is a digital temperature and humidity sensor optimized for battery-powered, high-volume consumer electronics. It offers low power consumption and high precision, making it ideal for portable and wearable devices.

## Features

- Get temperature in Celsius
- Get the humidity in %RH
- Put the sensor to sleep
- Wake-up the sensor

## How to use

### Init the I2C bus

```c
i2c_master_bus_handle_t i2c_bus_init(uint8_t sda_io, uint8_t scl_io)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = CONFIG_SHTC3_I2C_NUM,
        .sda_io_num = sda_io,
        .scl_io_num = scl_io,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    ESP_LOGI(TAG, "I2C master bus created");

    return bus_handle;
}
```

### Init the device

```c
shtc3_handle = shtc3_device_create(bus_handle, SHTC3_I2C_ADDR, CONFIG_SHTC3_I2C_CLK_SPEED_HZ);
```

### Read the values

```c
shtc3_get_th(shtc3_handle, reg, &temperature, &humidity);
```

For more details on how to use, see the provided example.

## Resources

- [SHTC3 Product Page](https://sensirion.com/products/catalog/SHTC3)
- [SHTC3 Datasheet](https://sensirion.com/media/documents/643F9C8E/63A5A436/Datasheet_SHTC3.pdf)
