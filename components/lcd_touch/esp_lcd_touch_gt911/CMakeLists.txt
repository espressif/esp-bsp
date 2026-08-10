set(REQ "esp_lcd")

# esp_lcd_touch is declared in idf_component.yml, which is only acted upon by the
# component manager. Add it explicitly when the component manager is disabled.
idf_build_get_property(idf_component_manager IDF_COMPONENT_MANAGER)
if(NOT idf_component_manager)
    list(APPEND REQ esp_lcd_touch)
endif()

idf_component_register(SRCS "esp_lcd_touch_gt911.c" INCLUDE_DIRS "include" REQUIRES ${REQ})
