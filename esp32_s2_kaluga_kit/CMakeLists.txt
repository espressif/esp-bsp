idf_component_register(
    SRCS "esp32_s2_kaluga_kit.c"
    #REQUIRES esp32-camera #@todo this swill be handled by pacman when esp-camera is published in pacman
    INCLUDE_DIRS "include"
)
