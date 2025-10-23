## :camera: Camera

The BSP provides a helper function bsp_camera_start() for initializing the on-board camera module.
This function sets up the required I2C bus, video subsystem, and camera clock if necessary.

### Example Usage

Camera usage can be quite complex. For a complete example, refer to the [`display_camera_csi`](https://github.com/espressif/esp-bsp/tree/master/examples/display_camera_csi) example in the BSP repository, or to the examples provided in the [`esp_video`](https://github.com/espressif/esp-video-components/tree/master/esp_video) component.

> [!NOTE]
> Please, do not forget select right camera sensor in `menuconfig`
