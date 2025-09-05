idf_component_register(
    SRCS "src/esp32_p4_eye.c" "src/led_blink_defaults.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver spiffs fatfs 
    PRIV_REQUIRES 
)
