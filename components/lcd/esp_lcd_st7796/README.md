# ESP LCD ST7796

[![Component Registry](https://components.espressif.com/components/espressif/esp_lcd_st7796/badge.svg)](https://components.espressif.com/components/espressif/esp_lcd_st7796)

Implementation of the ST7796 LCD controller with esp_lcd component.

| LCD controller | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| ST7796         | SPI/I80                 | esp_lcd_st7796 | [Specification](https://www.displayfuture.com/Display/datasheet/controller/ST7796s.pdf) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependency`, e.g.

```bash
compote manifest add-dependency espressif/esp_lcd_st7796==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Initialization Code

### I80 interface

```c
    ESP_LOGI(TAG, "Initialize Intel 8080 bus");
    esp_lcd_i80_bus_handle_t i80_bus = NULL;
    esp_lcd_i80_bus_config_t bus_config = ST7796_PANEL_BUS_I80_CONFIG(
            EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * EXAMPLE_LCD_BIT_PER_PIXEL / 8, EXAMPLE_LCD_DATA_WIDTH,
            EXAMPLE_PIN_NUM_LCD_DC, EXAMPLE_PIN_NUM_LCD_WR,
            EXAMPLE_PIN_NUM_LCD_DATA0, EXAMPLE_PIN_NUM_LCD_DATA1, EXAMPLE_PIN_NUM_LCD_DATA2, EXAMPLE_PIN_NUM_LCD_DATA3,
            EXAMPLE_PIN_NUM_LCD_DATA4, EXAMPLE_PIN_NUM_LCD_DATA5, EXAMPLE_PIN_NUM_LCD_DATA6, EXAMPLE_PIN_NUM_LCD_DATA7,
            EXAMPLE_PIN_NUM_LCD_DATA8, EXAMPLE_PIN_NUM_LCD_DATA9, EXAMPLE_PIN_NUM_LCD_DATA10, EXAMPLE_PIN_NUM_LCD_DATA11,
            EXAMPLE_PIN_NUM_LCD_DATA12, EXAMPLE_PIN_NUM_LCD_DATA13, EXAMPLE_PIN_NUM_LCD_DATA14, EXAMPLE_PIN_NUM_LCD_DATA15);
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i80_config_t io_config = ST7796_PANEL_IO_I80_CONFIG(EXAMPLE_PIN_NUM_LCD_CS, example_callback, &example_callback_ctx);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

/**
 * Uncomment these lines if use custom initialization commands.
 * The array should be declared as static const and positioned outside the function.
 */
// static const st7796_lcd_init_cmd_t lcd_init_cmds[] = {
// // {cmd, { data }, data_size, delay_ms}
//    {0xf0, (uint8_t []){0xc3}, 1, 0},
//    {0xf0, (uint8_t []){0x96}, 1, 0},
//    {0xb4, (uint8_t []){0x01}, 1, 0},
//     ...
// };

    ESP_LOGI(TAG, "Install ST7796 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    // const st7796_vendor_config_t vendor_config = {  // Uncomment these lines if use custom initialization commands
    //     .init_cmds = lcd_init_cmds,
    //     .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(st7796_lcd_init_cmd_t),
    // };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,      // Set to -1 if not use
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)      // Implemented by LCD command `36h`
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
#else
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
#endif
        .bits_per_pixel = EXAMPLE_LCD_BIT_PER_PIXEL,    // Implemented by LCD command `3Ah` (16/18/24)
        // .vendor_config = &vendor_config,            // Uncomment this line if use custom initialization commands
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7796(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    ESP_ERROR_CHECK(esp_lcd_panel_disp_off(panel_handle, false));
#else
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
#endif
```
