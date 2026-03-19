idf_component_register(
    SRCS "w25q64jv.c"
    INCLUDE_DIRS "."

    REQUIRES esp_timer driver
)
