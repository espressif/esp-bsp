idf_component_register(
    SRCS "esp-box.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
    PRIV_REQUIRES esp_timer esp_lcd esp_lcd_touch
)
