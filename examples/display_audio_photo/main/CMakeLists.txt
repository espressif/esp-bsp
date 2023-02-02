idf_component_register(SRCS "bsp_espbox_disp_example.c" "app_disp_fs.c"
                    INCLUDE_DIRS "."
                    REQUIRES "spiffs" "vfs" "es8311" "esp_jpeg")
