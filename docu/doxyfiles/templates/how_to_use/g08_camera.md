## :camera: Camera

There is no dedicated BSP API for camera functionality. Instead, the BSP provides default configuration macros:
- `BSP_CAMERA_DEFAULT_CONFIG`
- `BSP_CAMERA_VFLIP`
- `BSP_CAMERA_HMIRROR`

These macros are designed for use with the [esp32-camera](https://components.espressif.com/components/espressif/esp32-camera) component.

> [!NOTE]
> Don't forget to initialize I2C (`bsp_i2c_init()`) before using the camera, as some camera modules require I2C for configuration.

### Example Usage

```
/* Initialize I2C bus (required by camera module) */
bsp_i2c_init();

/* Initialize the camera using BSP default config */
const camera_config_t camera_config = BSP_CAMERA_DEFAULT_CONFIG;
esp_camera_init(&camera_config);

/* Optional: Set camera orientation */
sensor_t *s = esp_camera_sensor_get();
s->set_vflip(s, BSP_CAMERA_VFLIP);     // Vertical flip
s->set_hmirror(s, BSP_CAMERA_HMIRROR); // Horizontal mirror

...

/* Capture a frame */
camera_fb_t * pic = esp_camera_fb_get();
if (pic) {
    /* Access raw image data in pic->buf with size pic->len */
    process_image(pic->buf, pic->len);  // Replace with your function
    esp_camera_fb_return(pic);
}
```
