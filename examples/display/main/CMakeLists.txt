idf_component_register(SRCS "display_main.c" "lvgl_demo_ui.c"
                    INCLUDE_DIRS ".")

spiffs_create_partition_assets(
    storage
    "./images"
    FLASH_IN_PROJECT
    MMAP_FILE_SUPPORT_FORMAT ".png"
    MMAP_SUPPORT_RAW
    MMAP_RAW_COLOR_FORMAT "ARGB8888"
)

idf_component_get_property(lib espressif__esp_lv_fs COMPONENT_LIB)
target_compile_options(${lib} PRIVATE -Wno-incompatible-pointer-types -Wno-implicit-function-declaration)

