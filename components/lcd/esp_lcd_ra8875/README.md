# ESP LCD RA8875

[![Component Registry](https://components.espressif.com/components/espressif/esp_lcd_ra8875/badge.svg)](https://components.espressif.com/components/espressif/esp_lcd_ra8875)

Implementation of the RA8875 LCD controller with esp_lcd component. 

| LCD controller | Communication interface | Component name | Link to datasheet |
| :------------: | :---------------------: | :------------: | :---------------: |
| RA8875         | Parallel Intel 8080     | esp_lcd_ra8875 | [WIKI](https://www.waveshare.com/wiki/7inch_Capacitive_Touch_LCD_(C)) |

## Add to project

Packages from this repository are uploaded to [Espressif's component service](https://components.espressif.com/).
You can add them to your project via `idf.py add-dependancy`, e.g. 
```
    idf.py add-dependency esp_lcd_ra8875==1.0.0
```

Alternatively, you can create `idf_component.yml`. More is in [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html).

## Limitations

- Supported only 8-bit and 16-bit per pixel
- Supported only 8-bit and 16-bit communication interface
- Not supported color inversion

## Hardware notes

- Read is not supported on parallel communication interface. **Please don't forget put RD pin to HIGH and PS to LOW.**
- When CS pin is not used, put it to LOW.

## Usage

For detailed usage, please go to [LCD documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd.html).

## Example initialization 

```
esp_lcd_i80_bus_handle_t i80_bus = NULL;
esp_lcd_i80_bus_config_t bus_config = {
    .clk_src = LCD_CLK_SRC_PLL160M,
    .dc_gpio_num = BOARD_DISP_PARALLEL_DC,
    .wr_gpio_num = BOARD_DISP_PARALLEL_WR,
    .data_gpio_nums = {
        BOARD_DISP_PARALLEL_DB0,
        BOARD_DISP_PARALLEL_DB1,
        BOARD_DISP_PARALLEL_DB2,
        BOARD_DISP_PARALLEL_DB3,
        BOARD_DISP_PARALLEL_DB4,
        BOARD_DISP_PARALLEL_DB5,
        BOARD_DISP_PARALLEL_DB6,
        BOARD_DISP_PARALLEL_DB7,
        BOARD_DISP_PARALLEL_DB8,
        BOARD_DISP_PARALLEL_DB9,
        BOARD_DISP_PARALLEL_DB10,
        BOARD_DISP_PARALLEL_DB11,
        BOARD_DISP_PARALLEL_DB12,
        BOARD_DISP_PARALLEL_DB13,
        BOARD_DISP_PARALLEL_DB14,
        BOARD_DISP_PARALLEL_DB15,
    },
    .bus_width = BOARD_DISP_PARALLEL_WIDTH,
    .max_transfer_bytes = (BOARD_DISP_PARALLEL_HRES) * 80 * sizeof(uint16_t),
    .psram_trans_align = 64,
    .sram_trans_align = 4,
};
ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));

esp_lcd_panel_io_handle_t io_handle = NULL;
esp_lcd_panel_io_i80_config_t io_config = {
    .cs_gpio_num = BOARD_DISP_PARALLEL_CS,
    .pclk_hz = 20 * 1000 * 1000,
    .trans_queue_depth = 10,
    .dc_levels = {
        .dc_idle_level = 0,
        .dc_cmd_level = 1,
        .dc_dummy_level = 0,
        .dc_data_level = 0,
    },
    .flags = {
        .swap_color_bytes = 1,
        .pclk_idle_low = 0,
    },
    .lcd_cmd_bits = 16,
    .lcd_param_bits = 8,
};
ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &io_handle));

const esp_lcd_panel_ra8875_config_t vendor_config = {
    .wait_gpio_num = BOARD_DISP_PARALLEL_WAIT,
    .lcd_width = BOARD_DISP_PARALLEL_HRES,
    .lcd_height = BOARD_DISP_PARALLEL_VRES,
    .mcu_bit_interface = BOARD_DISP_PARALLEL_WIDTH,
};

esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = BOARD_DISP_PARALLEL_RST,
    .color_space = ESP_LCD_COLOR_SPACE_RGB,
    .bits_per_pixel = 16,
    .vendor_config = (void*)&vendor_config,
};
ESP_ERROR_CHECK(esp_lcd_new_panel_ra8875(io_handle, &panel_config, &lcd_panel_handle));

ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel_handle));
ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel_handle));
ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel_handle, true));

```
