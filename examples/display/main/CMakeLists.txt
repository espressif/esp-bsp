idf_component_register(SRCS "display_main.c" "lvgl_demo_ui.c"
                    INCLUDE_DIRS ".")

if(COMMAND lvgl_port_create_c_image)
    lvgl_port_create_c_image("images/esp_logo.png" "images/" "ARGB8888" "NONE")
    lvgl_port_create_c_image("images/esp_text.png" "images/" "ARGB8888" "NONE")
    lvgl_port_add_images(${COMPONENT_LIB} "images/")
else()
    file(GLOB_RECURSE IMAGE_SOURCES images/*.c)
    target_sources(${COMPONENT_LIB} PRIVATE ${IMAGE_SOURCES})
    idf_build_set_property(COMPILE_OPTIONS "-DLV_LVGL_H_INCLUDE_SIMPLE=1" APPEND)
endif()
