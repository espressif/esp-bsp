# Out-of-tree Espressif PPA / DMA2D draw units for LVGL 9+.
# Maps CONFIG_ESP_LVGL_PORT_* -> LV_USE_* / LV_PPA_* compile definitions and
# registers sources under src/lvgl9/draw/{ppa,dma2d}.

get_filename_component(ESP_LVGL_PORT_DRAW_COMPONENT_DIR
    "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(ESP_LVGL_PORT_DRAW_ROOT
    "${ESP_LVGL_PORT_DRAW_COMPONENT_DIR}/src/lvgl9/draw")

function(esp_lvgl_port_draw_units_check_lvgl_version lvgl_ver)
    set(_draw_on OFF)
    if(CONFIG_ESP_LVGL_PORT_USE_PPA OR CONFIG_ESP_LVGL_PORT_USE_ESP_DMA2D)
        set(_draw_on ON)
    endif()
    if(_draw_on AND "${lvgl_ver}" VERSION_LESS "9.0.0")
        message(FATAL_ERROR
            "ESP LVGL port PPA/DMA2D draw units require LVGL >= 9.0.0 "
            "(got '${lvgl_ver}'). Disable ESP_LVGL_PORT_USE_PPA / "
            "ESP_LVGL_PORT_USE_ESP_DMA2D or upgrade LVGL.")
    endif()
    # Stock LVGL may still ship a basic in-tree ESP PPA unit. Enabling both
    # would duplicate lv_draw_ppa_init and other symbols at link time.
    if(CONFIG_ESP_LVGL_PORT_USE_PPA AND CONFIG_LV_USE_PPA)
        message(FATAL_ERROR
            "CONFIG_ESP_LVGL_PORT_USE_PPA and CONFIG_LV_USE_PPA cannot both be "
            "enabled. Keep CONFIG_LV_USE_PPA unset and use the esp_lvgl_port "
            "draw unit exclusively.")
    endif()
endfunction()

function(esp_lvgl_port_draw_units_apply_defs target_name)
    if(CONFIG_ESP_LVGL_PORT_USE_PPA)
        target_compile_definitions(${target_name} PUBLIC LV_USE_PPA=1)
    else()
        target_compile_definitions(${target_name} PUBLIC LV_USE_PPA=0)
    endif()

    if(CONFIG_ESP_LVGL_PORT_USE_ESP_DMA2D)
        target_compile_definitions(${target_name} PUBLIC LV_USE_ESP_DMA2D=1)
    else()
        target_compile_definitions(${target_name} PUBLIC LV_USE_ESP_DMA2D=0)
    endif()

    if(NOT CONFIG_ESP_LVGL_PORT_USE_PPA)
        return()
    endif()

    foreach(_opt
            IMG TRANSFORM LAYER BORDER MASK_RECT TILE_COMPOSER GRADIENT
            ROUND_FILL ARC LABEL LINE TRIANGLE RUNTIME_TUNING STATS ASYNC)
        if(CONFIG_ESP_LVGL_PORT_USE_PPA_${_opt})
            target_compile_definitions(${target_name} PUBLIC LV_USE_PPA_${_opt}=1)
        else()
            target_compile_definitions(${target_name} PUBLIC LV_USE_PPA_${_opt}=0)
        endif()
    endforeach()

    if(DEFINED CONFIG_ESP_LVGL_PORT_PPA_BURST_LENGTH)
        target_compile_definitions(${target_name} PUBLIC
            LV_PPA_BURST_LENGTH=${CONFIG_ESP_LVGL_PORT_PPA_BURST_LENGTH})
    endif()
    if(CONFIG_ESP_LVGL_PORT_USE_PPA_TILE_COMPOSER)
        target_compile_definitions(${target_name} PUBLIC
            LV_PPA_TILE_SIZE=${CONFIG_ESP_LVGL_PORT_PPA_TILE_SIZE}
            LV_PPA_TILE_POOL_SIZE=${CONFIG_ESP_LVGL_PORT_PPA_TILE_POOL_SIZE})
    endif()
    if(CONFIG_ESP_LVGL_PORT_USE_PPA_GRADIENT)
        target_compile_definitions(${target_name} PUBLIC
            LV_PPA_GRADIENT_STEPS=${CONFIG_ESP_LVGL_PORT_PPA_GRADIENT_STEPS})
    endif()
    if(CONFIG_ESP_LVGL_PORT_USE_PPA_ROUND_FILL)
        target_compile_definitions(${target_name} PUBLIC
            LV_PPA_ROUND_FILL_MIN_RADIUS=${CONFIG_ESP_LVGL_PORT_PPA_ROUND_FILL_MIN_RADIUS})
    endif()
    if(CONFIG_ESP_LVGL_PORT_USE_PPA_ASYNC)
        target_compile_definitions(${target_name} PUBLIC
            LV_PPA_FILL_PENDING_TRANS=${CONFIG_ESP_LVGL_PORT_PPA_FILL_PENDING_TRANS}
            LV_PPA_BLEND_PENDING_TRANS=${CONFIG_ESP_LVGL_PORT_PPA_BLEND_PENDING_TRANS}
            LV_PPA_SRM_PENDING_TRANS=${CONFIG_ESP_LVGL_PORT_PPA_SRM_PENDING_TRANS})
    endif()
endfunction()

function(esp_lvgl_port_draw_units_apply_dma2d_defs target_name)
    if(NOT CONFIG_ESP_LVGL_PORT_USE_ESP_DMA2D)
        return()
    endif()
    if(DEFINED CONFIG_ESP_LVGL_PORT_ESP_DMA2D_BURST_LENGTH)
        target_compile_definitions(${target_name} PUBLIC
            LV_ESP_DMA2D_BURST_LENGTH=${CONFIG_ESP_LVGL_PORT_ESP_DMA2D_BURST_LENGTH})
    endif()
endfunction()

function(esp_lvgl_port_draw_units_collect_srcs out_srcs_var)
    set(_srcs "")
    if(CONFIG_ESP_LVGL_PORT_USE_PPA OR CONFIG_ESP_LVGL_PORT_USE_ESP_DMA2D)
        list(APPEND _srcs "${ESP_LVGL_PORT_DRAW_ROOT}/lv_draw_esp_buf.c")
    endif()
    if(CONFIG_ESP_LVGL_PORT_USE_PPA)
        file(GLOB _ppa_srcs "${ESP_LVGL_PORT_DRAW_ROOT}/ppa/*.c")
        list(APPEND _srcs ${_ppa_srcs})
    endif()
    if(CONFIG_ESP_LVGL_PORT_USE_ESP_DMA2D)
        file(GLOB _dma_srcs "${ESP_LVGL_PORT_DRAW_ROOT}/dma2d/*.c")
        list(APPEND _srcs ${_dma_srcs})
    endif()
    set(${out_srcs_var} "${_srcs}" PARENT_SCOPE)
endfunction()

function(esp_lvgl_port_draw_units_configure target_name lvgl_ver)
    esp_lvgl_port_draw_units_check_lvgl_version("${lvgl_ver}")

    if(NOT CONFIG_ESP_LVGL_PORT_USE_PPA AND NOT CONFIG_ESP_LVGL_PORT_USE_ESP_DMA2D)
        return()
    endif()

    esp_lvgl_port_draw_units_apply_defs(${target_name})
    esp_lvgl_port_draw_units_apply_dma2d_defs(${target_name})
    # PUBLIC so apps can #include "ppa/lv_draw_ppa.h" / "dma2d/lv_draw_esp_dma2d.h"
    # and inherit LV_USE_* compile definitions.
    target_include_directories(${target_name} PUBLIC
        "${ESP_LVGL_PORT_DRAW_ROOT}")
    target_include_directories(${target_name} PRIVATE
        "${ESP_LVGL_PORT_DRAW_ROOT}/ppa"
        "${ESP_LVGL_PORT_DRAW_ROOT}/dma2d")
endfunction()
