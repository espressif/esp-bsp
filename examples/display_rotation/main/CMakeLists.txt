#Get LVGL version
idf_component_get_property(lvgl_ver lvgl__lvgl COMPONENT_VERSION)

#Select image by LVGL version
if(lvgl_ver LESS "9.0.0")
    set(IMAGE_SOURCES "images/esp_logo_v8.c")
else()
    set(IMAGE_SOURCES "images/esp_logo_v9.c")
endif()

# TODO: select right image by LVGL version

idf_component_register(SRCS "main.c" ${IMAGE_SOURCES}
                    INCLUDE_DIRS ".")
