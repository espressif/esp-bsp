idf_component_register(
    SRCS "esp32_s3_korvo_2.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver
    PRIV_REQUIRES fatfs esp_timer esp_lcd esp_lcd_touch esp_adc
)
