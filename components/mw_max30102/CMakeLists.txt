idf_component_register(
    SRCS "mw_max30102.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer common log freertos
)
