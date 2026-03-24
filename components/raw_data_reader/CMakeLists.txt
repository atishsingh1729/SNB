idf_component_register(
    SRCS "raw_data_reader.c"
    INCLUDE_DIRS "."
    REQUIRES mw_bno085 mw_max30102 mw_ntc file_reader common freertos log
)
