# ESP LCD SH1107

Implementation of the SH1107 LCD controller with esp_lcd component. 

| LCD controller | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| SH1107         | I2C                     | esp_lcd_sh1107 | [WIKI](https://www.waveshare.com/wiki/1.3inch_OLED_Module_(C)) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g. 
```
    idf.py add-dependency esp_lcd_sh1107==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Usage

For detailed usage, please go to [LCD documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd.html).

## Example initialization 

```
esp_lcd_panel_io_handle_t io_handle = NULL;
esp_lcd_panel_io_i2c_config_t io_config = ESP_LCD_IO_I2C_SH1107_CONFIG();
ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)config->i2c.port, &io_config, &io_handle));

const esp_lcd_panel_sh1107_config_t vendor_config = {
    .lcd_width = BOARD_DISP_I2C_HRES,
    .lcd_height = BOARD_DISP_I2C_VRES,
};

esp_lcd_panel_handle_t lcd_panel_handle = NULL;
esp_lcd_panel_dev_config_t panel_config = {
    .bits_per_pixel = 1,
    .reset_gpio_num = BOARD_DISP_I2C_RST,
    .vendor_config = (void*)&vendor_config,
};
ESP_ERROR_CHECK(esp_lcd_new_panel_sh1107(io_handle, &panel_config, &lcd_panel_handle));

ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel_handle));
ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel_handle));
ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel_handle, true));
```

## Rotation and LVGL usage

For using this LCD display with LVGL or when you want to use rotation (only with LVGL), please use [`esp_lvgl_port`](
https://github.com/espressif/esp-bsp/tree/master/components/esp_lvgl_port) component.
