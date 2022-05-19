set(priv_requires "")

# esp_timer component was introduced in v4.2
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER "4.1")
    list(APPEND priv_requires esp_timer)
endif()

idf_component_register(
    SRCS "mag3110.c"
    INCLUDE_DIRS "include"
    REQUIRES "driver"
    PRIV_REQUIRES ${priv_requires}
)
