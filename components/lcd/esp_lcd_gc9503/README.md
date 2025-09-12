# ESP LCD GC9503

[![Component Registry](https://components.espressif.com/components/espressif/esp_lcd_gc9503/badge.svg)](https://components.espressif.com/components/espressif/esp_lcd_gc9503)

Implementation of the GC9503 LCD controller with esp_lcd component.

| LCD controller | Communication interface | Component name |                     Link to datasheet                      |
| :------------: | :---------------------: | :------------: | :--------------------------------------------------------: |
|     GC9503     |    3-wire SPI + RGB     | esp_lcd_gc9503 | [WIKI](https://github.com/espressif/esp-dev-kits/blob/master/docs/_static/esp32-s3-lcd-ev-board/datasheets/3.95_480x480_SmartDisplay/GC9503NP_DataSheet_V1.7.pdf) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g.

```
    idf.py add-dependency esp_lcd_gc9503==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Example use

For most RGB LCDs, they typically use a "[3-Wire SPI + Parallel RGB](https://focuslcds.com/3-wire-spi-parallel-rgb-interface-fan4213/)" interface. The "3-Wire SPI" interface is used for transmitting command data and the "Parallel RGB" interface is used for sending pixel data.

It's recommended to use the [esp_lcd_panel_io_additions](https://components.espressif.com/components/espressif/esp_lcd_panel_io_additions) component to bit-bang the "3-Wire SPI" interface through **GPIO** or an **IO expander** (like [TCA9554](https://components.espressif.com/components/espressif/esp_io_expander_tca9554)). To do this, please first add this component to your project manually. Then, refer to the following code to initialize the GC9503 controller.

```c
    ESP_LOGI(TAG, "Install 3-wire SPI panel IO");
    spi_line_config_t line_config = {
        .cs_io_type = IO_TYPE_GPIO,                 // Set to `IO_TYPE_EXPANDER` if using IO expander
        .cs_gpio_num = EXAMPLE_LCD_IO_SPI_CS,
        .scl_io_type = IO_TYPE_GPIO,
        .scl_gpio_num = EXAMPLE_LCD_IO_SPI_SCK,
        .sda_io_type = IO_TYPE_GPIO,
        .sda_gpio_num = EXAMPLE_LCD_IO_SPI_SDO,
        .io_expander = NULL,                        // Set to device handle if using IO expander

    };
    esp_lcd_panel_io_3wire_spi_config_t io_config = GC9503_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle));

/**
 * Uncomment these line if use custom initialization commands.
 * The array should be declared as static const and positioned outside the function.
 */
// static const gc9503_lcd_init_cmd_t lcd_init_cmds[] = {
// //  {cmd, { data }, data_size, delay_ms}
//    {0xc1, (uint8_t []){0x3f}, 1, 0},
//    {0xc2, (uint8_t []){0x0e}, 1, 0},
//    {0xc6, (uint8_t []){0xf8}, 1, 0},
//     ...
// };

    ESP_LOGI(TAG, "Install GC9503 panel driver");
    esp_lcd_rgb_panel_config_t rgb_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .dma_burst_size = 64,
        .data_width = 16,
        .bits_per_pixel = 16,
        .de_gpio_num = EXAMPLE_LCD_IO_RGB_DE,
        .pclk_gpio_num = EXAMPLE_LCD_IO_RGB_PCLK,
        .vsync_gpio_num = EXAMPLE_LCD_IO_RGB_VSYNC,
        .hsync_gpio_num = EXAMPLE_LCD_IO_RGB_HSYNC,
        .disp_gpio_num = EXAMPLE_LCD_IO_RGB_DISP,
        .data_gpio_nums = {
            EXAMPLE_LCD_IO_RGB_DATA0,
            EXAMPLE_LCD_IO_RGB_DATA1,
            EXAMPLE_LCD_IO_RGB_DATA2,
            EXAMPLE_LCD_IO_RGB_DATA3,
            EXAMPLE_LCD_IO_RGB_DATA4,
            EXAMPLE_LCD_IO_RGB_DATA5,
            EXAMPLE_LCD_IO_RGB_DATA6,
            EXAMPLE_LCD_IO_RGB_DATA7,
            EXAMPLE_LCD_IO_RGB_DATA8,
            EXAMPLE_LCD_IO_RGB_DATA9,
            EXAMPLE_LCD_IO_RGB_DATA10,
            EXAMPLE_LCD_IO_RGB_DATA11,
            EXAMPLE_LCD_IO_RGB_DATA12,
            EXAMPLE_LCD_IO_RGB_DATA13,
            EXAMPLE_LCD_IO_RGB_DATA14,
            EXAMPLE_LCD_IO_RGB_DATA15,
        },
        .timings = GC9503_480_480_PANEL_60HZ_RGB_TIMING(),
        .flags.fb_in_psram = 1,
        ...
    };
    gc9503_vendor_config_t vendor_config = {
        .rgb_config = &rgb_config,
        // .init_cmds = lcd_init_cmds,      // Uncomment these line if use custom initialization commands
        // .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(gc9503_lcd_init_cmd_t),
        .flags = {
            .mirror_by_cmd = 1,             // Only work when `auto_del_panel_io` is set to 0
            .auto_del_panel_io = 0,         /**
                                             * Set to 1 if panel IO is no longer needed after LCD initialization.
                                             * If the panel IO pins are sharing other pins of the RGB interface to save GPIOs,
                                             * Please set it to 1 to release the pins.
                                             */
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_IO_RST,           // Set to -1 if not use
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,     // Implemented by LCD command `B1h`
        .bits_per_pixel = EXAMPLE_LCD_BIT_PER_PIXEL,    // Implemented by LCD command `3Ah` (16/18/24)
        .vendor_config = &vendor_config,
    };
    esp_lcd_panel_handle_t panel_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9503(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true)); /**
                                                                     * Don't call this function if `auto_del_panel_io` is set to 0
                                                                     * and `disp_gpio_num` is set to -1
                                                                     */
```
