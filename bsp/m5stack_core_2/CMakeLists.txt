idf_component_register(
    SRCS "m5stack_core_2.c" "m5stack_core_2_idf5.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver spiffs
    PRIV_REQUIRES fatfs esp_lcd
)
