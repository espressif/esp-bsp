idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_lcd)

lvgl_port_create_c_image("images/esp_logo.png" "images/" "ARGB8888" "NONE")
lvgl_port_add_images(${COMPONENT_LIB} "images/")
