idf_component_register(
    SRCS "mag3110.c"
    INCLUDE_DIRS "include"
    REQUIRES "driver"
    PRIV_REQUIRES "esp_timer"
)
