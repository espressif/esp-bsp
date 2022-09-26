idf_component_register(
    SRCS "ws_7inch.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
    PRIV_REQUIRES esp_timer esp_lcd esp_lcd_touch
)
