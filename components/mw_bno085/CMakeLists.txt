idf_component_register(
    SRCS "mw_bno085.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer common log freertos
    PRIV_REQUIRES esp-dsp
)
