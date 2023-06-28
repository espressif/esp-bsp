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

Create 3-wire SPI panel IO using [esp_lcd_panel_io_additions](https://components.espressif.com/components/espressif/esp_lcd_panel_io_additions) component.

```
    spi_line_config_t line_config = {
        .cs_io_type = IO_TYPE_EXPANDER,
        .cs_expander_pin = BSP_LCD_SPI_CS,
        .scl_io_type = IO_TYPE_EXPANDER,
        .scl_expander_pin = BSP_LCD_SPI_SCK,
        .sda_io_type = IO_TYPE_EXPANDER,
        .sda_expander_pin = BSP_LCD_SPI_SDO,
        .io_expander = expander_handle,
    };
    esp_lcd_panel_io_3wire_spi_config_t io_config = GC9503_PANEL_IO_3WIRE_SPI_CONFIG(line_config);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle));
```

Initialize GC9503 and create RGB panel.

```
    esp_lcd_rgb_panel_config_t panel_conf = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .psram_trans_align = 64,
        .data_width = 16,
        .de_gpio_num = BSP_LCD_DE,
        .pclk_gpio_num = BSP_LCD_PCLK,
        .vsync_gpio_num = BSP_LCD_VSYNC,
        .hsync_gpio_num = BSP_LCD_HSYNC,
        .data_gpio_nums = {
            BSP_LCD_DATA0,
            BSP_LCD_DATA1,
            BSP_LCD_DATA2,
            BSP_LCD_DATA3,
            BSP_LCD_DATA4,
            BSP_LCD_DATA5,
            BSP_LCD_DATA6,
            BSP_LCD_DATA7,
            BSP_LCD_DATA8,
            BSP_LCD_DATA9,
            BSP_LCD_DATA10,
            BSP_LCD_DATA11,
            BSP_LCD_DATA12,
            BSP_LCD_DATA13,
            BSP_LCD_DATA14,
            BSP_LCD_DATA15,
        },
        .timings = GC9503_480_480_PANEL_60HZ_RGB_TIMING(),
        .flags.fb_in_psram = 1,
    };
    esp_lcd_panel_handle_t panel_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9503(io_handle, &panel_conf, &panel_handle));
```

Delete panel IO if it is no longer needed.

```
    ESP_ERROR_CHECK(esp_lcd_panel_io_del(io_handle));
```
