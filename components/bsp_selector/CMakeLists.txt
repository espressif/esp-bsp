idf_component_register()

if(CONFIG_BSP_SELECT_NONE)
    message(FATAL_ERROR
        "No BSP selected. Please select a BSP in menuconfig (CONFIG_BSP_SELECT_*)"
    )
endif()
