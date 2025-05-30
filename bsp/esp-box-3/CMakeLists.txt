idf_component_register(
    SRCS "esp-box-3.c" "esp-box-3_idf5.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver spiffs fatfs
    PRIV_REQUIRES esp_lcd
)
