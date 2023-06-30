idf_component_register(
    SRCS "esp32_c3_lcdkit.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    PRIV_REQUIRES
        "esp_lcd"
    REQUIRES
        "driver"
        "spiffs"
)