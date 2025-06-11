# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# lcd_touch components do not have their own dedicated test_app
# so we add them to build here to have at least compilation test
set(EXTRA_COMPONENT_DIRS "../../components/lcd_touch")

# Explicitly add components that do not have an test_app
# Must be kept in sync with list in .build-test-rules.yml
list(APPEND EXTRA_COMPONENT_DIRS
    "../../components/lcd/esp_lcd_ra8875"
    "../../components/bh1750"
    "../../components/fbm320"
    "../../components/hts221"
    "../../components/mag3110"
    "../../components/mpu6050"
    )


include($ENV{IDF_PATH}/tools/cmake/version.cmake) # $ENV{IDF_VERSION} was added after v4.3...

# Set the components to include the tests for.
set(TEST_COMPONENTS bh1750 mpu6050 mag3110 hts221 fbm320 CACHE STRING "List of components to test")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_bsp_test_app)
