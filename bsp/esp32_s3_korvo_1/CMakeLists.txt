idf_component_register(
    SRCS "esp32_s3_korvo_1.c" "esp32_s3_korvo_1_idf5.c" "led_blink_defaults.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    PRIV_REQUIRES esp_psram
    REQUIRES driver spiffs fatfs esp_adc vfs
)
