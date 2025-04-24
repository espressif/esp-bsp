idf_component_register(SRCS "main.c" "wifi.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi nvs_flash mqtt)
