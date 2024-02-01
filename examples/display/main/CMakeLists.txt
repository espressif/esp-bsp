#Get LVGL version
idf_component_get_property(lvgl_ver lvgl__lvgl COMPONENT_VERSION)

#Select image by LVGL version
if(lvgl_ver LESS "9.0.0")
    set(IMAGE_SOURCES "images/lvgl8/esp_logo.c" "images/lvgl8/esp_text.c")
else()
    set(IMAGE_SOURCES "images/lvgl9/esp_logo.c" "images/lvgl9/esp_text.c")
endif()

idf_component_register(SRCS "display_main.c" "lvgl_demo_ui.c" ${IMAGE_SOURCES}
                    INCLUDE_DIRS ".")
