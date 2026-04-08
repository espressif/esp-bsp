## 0.3.1

- Bump dependency `onewire_bus` to `^1.1.0` to align with new backend capabilities (including UART backend support).
- Update `ds18b20_read` example to support backend selection in menuconfig (RMT/UART).

## 0.3.0

- Add detection for uninitialized power-on state (85.0C) and return `ESP_ERR_INVALID_STATE` when this value is read.

## 0.2.0

- Support trigger temperature conversion for all DS18B20 sensors on the same bus with a single function call (`ds18b20_trigger_temperature_conversion_for_all`).
- Renamed `ds18b20_new_device` to `ds18b20_new_device_from_enumeration`.
- Renamed `ds18b20_new_single_device` to `ds18b20_new_device_from_bus`.

## 0.1.2

- Add single device function (ds18b20_new_single_device) to create a new DS18B20 device instance without enumerating all devices on the bus.

## 0.1.1

- Fix the issue that sign-bit is not extended properly when doing temperature value conversion.

## 0.1.0

- Initial driver version, based on the [onewire_bus](https://components.espressif.com/components/espressif/onewire_bus) library.
