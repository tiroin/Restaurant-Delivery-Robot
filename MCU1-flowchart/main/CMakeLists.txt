idf_component_register(
    SRCS
        "app_main.c"
        "ble_robot.c"
    INCLUDE_DIRS "."
    REQUIRES
        bt
        nvs_flash
        esp_driver_twai
)