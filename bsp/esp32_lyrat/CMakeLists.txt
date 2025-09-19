idf_component_register(
    SRCS "esp32_lyrat.c" "esp32_lyrat_idf5.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    PRIV_REQUIRES esp_psram
    REQUIRES driver spiffs fatfs 
)
