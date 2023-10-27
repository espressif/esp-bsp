# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS "../components")

include($ENV{IDF_PATH}/tools/cmake/version.cmake) # $ENV{IDF_VERSION} was added after v4.3...

# Test lcd components only in IDF 4.4 and greater versions
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "4.4")
    list(APPEND EXTRA_COMPONENT_DIRS "../components/lcd")
    list(APPEND EXTRA_COMPONENT_DIRS "../components/lcd_touch")
    list(APPEND EXTRA_COMPONENT_DIRS "../components/io_expander")
endif()
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "4.4")
    set(EXCLUDE_COMPONENTS "es8311" "es7210" "esp_lvgl_port" "ds18b20" "icm42670")
elseif("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
    set(EXCLUDE_COMPONENTS "esp_lcd_touch_stmpe610" "ds18b20" "esp_lcd_ssd1681")
endif()

# Test rgb lcd components only in esp32s3
if(NOT "${IDF_TARGET}" STREQUAL "esp32s3")
    list(APPEND EXCLUDE_COMPONENTS "esp_lcd_gc9503")
endif()

# Test i80 lcd components only in esp32s2, esp32s3
if(NOT "${IDF_TARGET}" STREQUAL "esp32s2" AND NOT "${IDF_TARGET}" STREQUAL "esp32s3")
    list(APPEND EXCLUDE_COMPONENTS "esp_lcd_st7796")
endif()

# Set the components to include the tests for.
set(TEST_COMPONENTS bh1750 ssd1306 mpu6050 mag3110 hts221 fbm320 es7210 icm42670 CACHE STRING "List of components to test")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_bsp_test_app)
