idf_component_register(
    SRCS "esp32_azure_iot_kit.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver esp_lcd fatfs
    PRIV_REQUIRES spiffs esp_psram
)
