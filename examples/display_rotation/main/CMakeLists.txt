idf_component_register(SRCS "main.c"
                    REQUIRES driver
                    INCLUDE_DIRS ".")

lvgl_port_create_c_image("images/esp_logo.png" "images/gen/" "ARGB8888" "NONE")
lvgl_port_add_images(${COMPONENT_LIB} "images/gen/")
