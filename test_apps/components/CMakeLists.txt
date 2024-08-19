# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS "../../components" "../../components/lcd" "../../components/lcd_touch" "../../components/io_expander")
set(EXCLUDE_COMPONENTS "es8311 es7210") # Deprecated components

include($ENV{IDF_PATH}/tools/cmake/version.cmake) # $ENV{IDF_VERSION} was added after v4.3...

# Components only for IDF5.0 and greater
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
    list(APPEND EXCLUDE_COMPONENTS "esp_lcd_touch_stmpe610" "ds18b20" "esp_lcd_ssd1681")
elseif("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.3")
    list(APPEND EXCLUDE_COMPONENTS "esp_lcd_ili9881c")
endif()

# Test rgb lcd components only in IDF 5.0.5 or greater
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0.4")
    list(APPEND EXCLUDE_COMPONENTS "esp_lcd_gc9503")
endif()

# Test rgb lcd components only in esp32s3
if(NOT "${IDF_TARGET}" STREQUAL "esp32s3")
    list(APPEND EXCLUDE_COMPONENTS "esp_lcd_gc9503")
endif()

# Test mipi dsi lcd components only in esp32p4
if(NOT "${IDF_TARGET}" STREQUAL "esp32p4")
    list(APPEND EXCLUDE_COMPONENTS "esp_lcd_ili9881c")
endif()

# Test i80 lcd components only in esp32s2, esp32s3
if(NOT "${IDF_TARGET}" STREQUAL "esp32s2" AND NOT "${IDF_TARGET}" STREQUAL "esp32s3")
    list(APPEND EXCLUDE_COMPONENTS "esp_lcd_st7796")
endif()

# Set the components to include the tests for.
set(TEST_COMPONENTS bh1750 mpu6050 mag3110 hts221 fbm320 icm42670 qma6100p CACHE STRING "List of components to test")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_bsp_test_app)
