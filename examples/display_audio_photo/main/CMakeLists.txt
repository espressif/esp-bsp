idf_component_register(SRCS "main.c" "app_disp_fs.c"
                    INCLUDE_DIRS "."
                    REQUIRES "spiffs" "vfs")
