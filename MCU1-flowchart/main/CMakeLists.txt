idf_component_register(
    SRCS
        "app_main.c"
        "ble_robot.c"
        "wifi_ap_ws.c"
    INCLUDE_DIRS "."
    REQUIRES
        bt
        nvs_flash
        esp_driver_twai
        esp_http_server
        esp_wifi
        esp_event
        esp_netif
    EMBED_TXTFILES
        "index.html"
        "customer.html"
)