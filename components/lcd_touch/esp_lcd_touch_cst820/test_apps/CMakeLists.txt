# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for CMake to work correctly.
cmake_minimum_required(VERSION 3.16)
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test_esp_lcd_touch_cst820)
