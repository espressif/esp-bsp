## :video_game: Sensors

Boards with integrated sensors (e.g., IMUs, environmental sensors) are abstracted using the sensor hub component.
The BSP project provides APIs for initializing each sensor and it is up to the integrating platform to provide configuration of the data acquisition and callback handlers.

For practical usage examples and supported sensors on your board, refer to the relevant examples in the BSP repository, such as:
- `sensors` (for reading sensor data)
- TODO: `display_rotation` (for IMU setup and orientation control)

### Usage

Create a handler function for sensor hub events
``` c
void sensor_event_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    ...
}
```

Set up a sensor configuration and initialize it
``` c
bsp_sensor_config_t imu_config = {
    .type = IMU_ID,
    .mode = MODE_POLLING,
    .period = 1000
};
ESP_ERROR_CHECK(bsp_sensor_init(&imu_config, &imu_sensor_handle));
```

Associate an event handler with the configured sensor
``` c
iot_sensor_handler_register(imu_sensor_handle, sensor_event_handler, NULL);
```

Start the sensor data acquisition
``` c
iot_sensor_start(imu_sensor_handle);
```

**Notes:**
- More information can be found in [sensor hub documentation page](https://docs.espressif.com/projects/esp-iot-solution/en/latest/sensors/sensor_hub.html).
