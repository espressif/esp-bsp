idf_component_register(
    SRCS "m5dial.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver spiffs
    PRIV_REQUIRES fatfs esp_lcd esp_psram
)
