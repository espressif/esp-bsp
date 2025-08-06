idf_component_register(
    SRCS "m5stack_core.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver spiffs fatfs
    PRIV_REQUIRES esp_lcd esp_psram
)
