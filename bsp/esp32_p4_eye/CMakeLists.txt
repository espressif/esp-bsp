idf_component_register(
    SRCS "esp32_p4_eye.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver spiffs fatfs 
    PRIV_REQUIRES 
)
