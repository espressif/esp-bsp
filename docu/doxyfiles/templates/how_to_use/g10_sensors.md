## :video_game: IMU & Sensors

Boards with integrated sensors (e.g., IMUs, environmental sensors) are not fully abstracted by the BSP. The BSP does not provide specific APIs for sensor control or data retrieval.

Instead, the BSP includes selected sensor components via the `idf_component.yml` file and assists with peripheral initialization (typically I2C bus configuration).

> [!NOTE]
> Sensor initialization and usage are left to the user. The BSP ensures the I2C bus is properly configured but does not wrap sensor APIs.

For practical usage examples and supported sensors on your board, refer to the relevant examples in the BSP repository, such as:
- `display_rotation` (for IMU setup and orientation control)
- `display_sensors` (for reading sensor data)
