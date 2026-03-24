idf_component_register(
    SRCS "mw_flash.c"
    INCLUDE_DIRS "."
    REQUIRES hal_spi driver log efuse
)
