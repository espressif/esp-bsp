# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Add RainMaker components and other common application components
set(EXTRA_COMPONENT_DIRS esp-rainmaker/components esp-rainmaker/examples/common/app_wifi esp-rainmaker/examples/common/app_reset)

set(PROJECT_VER "1.0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rainmaker-example)
