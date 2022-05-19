set(requires esp_rainmaker nvs_flash hts221 ssd1306 json_parser app_wifi app_reset esp32_azure_iot_kit)

# esp_timer component was introduced in v4.2
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER "4.1")
    list(APPEND requires esp_timer)
endif()

idf_component_register(SRCS ./app_driver.c ./rainmaker_example_main.c
                       INCLUDE_DIRS "."
                       REQUIRES ${requires})
