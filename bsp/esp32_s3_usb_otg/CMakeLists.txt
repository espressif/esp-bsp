idf_component_register(
    SRCS "esp32_s3_usb_otg.c" "esp32_s3_usb_otg_idf5.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver esp_lcd fatfs
    PRIV_REQUIRES usb spiffs esp_adc esp_psram
)
