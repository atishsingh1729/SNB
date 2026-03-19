idf_component_register(
    SRCS
        "main.c"
        "peripherial_init.c"
        "../components/ble/ble.c"
        "Feature_processing/feature_processing.c"
        "sensor_data_processing/data_processing.c"
        "../components/imu/bno085.c"
        "../components/hr/max30102.c"
        "struct_2_json.c"
        "../components/ext_flash/w25q64jv.c"
    INCLUDE_DIRS
        "."
        "Feature_processing"

    REQUIRES bt nvs_flash json esp_adc driver
)
