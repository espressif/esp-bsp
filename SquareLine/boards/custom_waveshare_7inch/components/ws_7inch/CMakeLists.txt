idf_component_register(
    SRCS "ws_7inch.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver spiffs
    PRIV_REQUIRES esp_lcd
)
