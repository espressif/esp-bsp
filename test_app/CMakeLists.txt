# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS "../components")

include($ENV{IDF_PATH}/tools/cmake/version.cmake) # $ENV{IDF_VERSION} was added after v4.3...
# Test lcd_touch components only in IDF 5.0 greater versions
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.0")
    list(APPEND EXTRA_COMPONENT_DIRS "../components/lcd_touch") #TODO: Move this to version 4.4 and greater, when it will be fixed on IDF
endif()
# Test lcd components only in IDF 4.4 and greater versions
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "4.4")
    list(APPEND EXTRA_COMPONENT_DIRS "../components/lcd")
endif()
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "4.4")
    set(EXCLUDE_COMPONENTS "es8311" "esp_jpeg")
endif()


# Set the components to include the tests for.
set(TEST_COMPONENTS bh1750 ssd1306 mpu6050 mag3110 hts221 fbm320 CACHE STRING "List of components to test")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_bsp_test_app)
