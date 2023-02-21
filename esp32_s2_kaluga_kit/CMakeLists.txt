idf_component_register(
    SRCS "esp32_s2_kaluga_kit.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver
    PRIV_REQUIRES esp_lcd
)
