file(GLOB_RECURSE SRCS src/*.c)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver spiffs fatfs esp_driver_ledc
    PRIV_REQUIRES esp_lcd 
)
