idf_component_register(
    SRCS
        "app_main.c"
        "wifi_ap_ws.c"
    INCLUDE_DIRS "."
    EMBED_FILES  "index.html"
    REQUIRES
        esp_wifi
        esp_http_server
        esp_netif
        esp_event
        nvs_flash
        esp_driver_twai
)