# JPEG Decoder: TJpgDec - Tiny JPEG Decompressor

TJpgDec is a generic JPEG image decompressor module that highly optimized for small embedded systems. It works with very low memory consumption.

Some microcontrollers have tjpg decoder in ROM, it is used, if there is. [^1]

[^1]: **_NOTE:_** When there is used ROM decoder, there cannot be changed configuration. 

## Features
- Pixel Format: RGB888, RGB565
- Scaling Ratio: 1/1, 1/2, 1/4 or 1/8 Selectable on Decompression
- Allow swap the first and the last byte of the color


## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g. 
```
    idf.py add-dependency esp_jpeg==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use

Here is example of usage. This calling is **blocking**.

```

esp_jpeg_image_cfg_t jpeg_cfg = {
    .indata = (uint8_t *)jpeg_img_buf,
    .indata_size = jpeg_img_buf_size,
    .outbuf = out_img_buf,
    .outbuf_size = out_img_buf_size,
    .out_format = JPEG_IMAGE_OUT_FORMAT_RGB565,
    .out_scale = JPEG_IMAGE_SCALE_0,
    .flags = {
        .swap_color_bytes = 1,
    }
};
esp_jpeg_image_output_t outimg;

esp_jpeg_decode(&jpeg_cfg, &outimg);
```
