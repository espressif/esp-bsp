# DS18B20 Device Driver

[![Component Registry](https://components.espressif.com/components/espressif/ds18b20/badge.svg)](https://components.espressif.com/components/espressif/ds18b20)

DS18B20 temperature sensor only uses a single wire to write and read data, the interface is also called the `1-Wire` bus. This component only contains the sensor driver. For 1-Wire bus setup, you need to use the [onewire_bus](https://components.espressif.com/components/espressif/onewire_bus) library to initialize and enumerate the devices.

## How to enumerate DS18B20 devices on the 1-Wire bus

```c
    #define EXAMPLE_ONEWIRE_BUS_GPIO    0
    #define EXAMPLE_ONEWIRE_MAX_DS18B20 2

    // install 1-wire bus
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = EXAMPLE_ONEWIRE_BUS_GPIO,
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    int ds18b20_device_num = 0;
    ds18b20_device_handle_t ds18b20s[EXAMPLE_ONEWIRE_MAX_DS18B20];
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    // create 1-wire device iterator, which is used for device search
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI(TAG, "Device iterator created, start searching...");
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            // check if the device is a DS18B20, if so, return the ds18b20 handle
            if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[ds18b20_device_num]) == ESP_OK) {
                ESP_LOGI(TAG, "Found a DS18B20[%d], address: %016llX", ds18b20_device_num, next_onewire_device.address);
                ds18b20_device_num++;
            } else {
                ESP_LOGI(TAG, "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI(TAG, "Searching done, %d DS18B20 device(s) found", ds18b20_device_num);

    // Now you have the DS18B20 sensor handle, you can use it to read the temperature
```

## Trigger a temperature conversion and then read the data

```c
for (int i = 0; i < ds18b20_device_num; i ++) {
    ESP_ERROR_CHECK(ds18b20_trigger_temperature_conversion(ds18b20s[i]));
    ESP_ERROR_CHECK(ds18b20_get_temperature(ds18b20s[i], &temperature));
    ESP_LOGI(TAG, "temperature read from DS18B20[%d]: %.2fC", i, temperature);
}
```

## Reference

* See [DS18B20 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ds18b20.pdf)
