include($ENV{IDF_PATH}/tools/cmake/version.cmake) # $ENV{IDF_VERSION} was added after v4.3...

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "4.4")
    return() 
endif()

#Get LVGL version
idf_component_get_property(lvgl_ver lvgl__lvgl COMPONENT_VERSION)
if(lvgl_ver EQUAL "")
    idf_component_get_property(lvgl_ver lvgl COMPONENT_VERSION)
endif()
message(STATUS "LVGL version: ${lvgl_ver}")

#Select folder by LVGL version
if(lvgl_ver VERSION_LESS "9.0.0")
    message(VERBOSE "Compiling esp_lvgl_port for LVGL8")
    set(PORT_FOLDER "lvgl8")
else()
    message(VERBOSE "Compiling esp_lvgl_port for LVGL9")
    set(PORT_FOLDER "lvgl9")
endif()

set(PORT_PATH "src/${PORT_FOLDER}")

# Include SIMD assembly source code for rendering
if(CONFIG_IDF_TARGET_ESP32 OR CONFIG_IDF_TARGET_ESP32S3)
    if (lvgl_ver VERSION_GREATER_EQUAL "9.0.0")
        message(VERBOSE "Compiling SIMD")
        if(CONFIG_IDF_TARGET_ESP32S3)
            file(GLOB_RECURSE ASM_SOURCES ${PORT_PATH}/simd/*_esp32s3.S)    # Select only esp32s3 related files
        else()
            file(GLOB_RECURSE ASM_SOURCES ${PORT_PATH}/simd/*_esp32.S)      # Select only esp32 related files
        endif()
    endif()
endif()

idf_component_register(
        SRCS "${PORT_PATH}/esp_lvgl_port.c" "${PORT_PATH}/esp_lvgl_port_disp.c" ${ASM_SOURCES}
        INCLUDE_DIRS "include" 
        PRIV_INCLUDE_DIRS "priv_include"
        REQUIRES "esp_lcd" 
        PRIV_REQUIRES "esp_timer")

set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY INTERFACE_LINK_LIBRARIES "-u lv_color_blend_to_argb8888_esp")
set_property(TARGET ${COMPONENT_LIB} APPEND PROPERTY INTERFACE_LINK_LIBRARIES "-u lv_color_blend_to_rgb565_esp")

#Get LVGL target 
idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
if(lvgl_lib EQUAL "")
    idf_component_get_property(lvgl_lib lvgl COMPONENT_LIB)
endif()

target_include_directories(${lvgl_lib} PRIVATE include)

set(ADD_SRCS "")
set(ADD_LIBS "")

idf_build_get_property(build_components BUILD_COMPONENTS)
if("espressif__button" IN_LIST build_components)
    list(APPEND ADD_SRCS "${PORT_PATH}/esp_lvgl_port_button.c")
    list(APPEND ADD_LIBS idf::espressif__button)
endif()
if("button" IN_LIST build_components)
    list(APPEND ADD_SRCS "${PORT_PATH}/esp_lvgl_port_button.c")
    list(APPEND ADD_LIBS idf::button)
endif()
if("espressif__esp_lcd_touch" IN_LIST build_components)
    list(APPEND ADD_SRCS "${PORT_PATH}/esp_lvgl_port_touch.c")
    list(APPEND ADD_LIBS idf::espressif__esp_lcd_touch)
endif()
if("esp_lcd_touch" IN_LIST build_components)
    list(APPEND ADD_SRCS "${PORT_PATH}/esp_lvgl_port_touch.c")
    list(APPEND ADD_LIBS idf::esp_lcd_touch)
endif()
if("espressif__knob" IN_LIST build_components)
    list(APPEND ADD_SRCS "${PORT_PATH}/esp_lvgl_port_knob.c")
    list(APPEND ADD_LIBS idf::espressif__knob)
endif()
if("knob" IN_LIST build_components)
    list(APPEND ADD_SRCS "${PORT_PATH}/esp_lvgl_port_knob.c")
    list(APPEND ADD_LIBS idf::knob)
endif()
if("espressif__usb_host_hid" IN_LIST build_components)
    list(APPEND ADD_SRCS "${PORT_PATH}/esp_lvgl_port_usbhid.c" "images/${PORT_FOLDER}/img_cursor.c")
    list(APPEND ADD_LIBS idf::espressif__usb_host_hid)
endif()
if("usb_host_hid" IN_LIST build_components)
    list(APPEND ADD_SRCS "${PORT_PATH}/esp_lvgl_port_usbhid.c" "images/${PORT_FOLDER}/img_cursor.c")
    list(APPEND ADD_LIBS idf::usb_host_hid)
endif()

if(ADD_SRCS)
    target_sources(${COMPONENT_LIB} PRIVATE ${ADD_SRCS})
endif()
if(ADD_LIBS)
    target_link_libraries(${COMPONENT_LIB} PRIVATE ${ADD_LIBS})
endif()
