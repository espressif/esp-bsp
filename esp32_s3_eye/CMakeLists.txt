idf_component_register(
    SRCS "esp32_s3_eye.c"
    INCLUDE_DIRS "include"
    REQUIRES driver
    PRIV_REQUIRES fatfs esp_lcd esp_timer
)
