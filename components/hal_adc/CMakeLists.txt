idf_component_register(
    SRCS "hal_adc.c"
    INCLUDE_DIRS "."
    REQUIRES esp_adc common log
)
