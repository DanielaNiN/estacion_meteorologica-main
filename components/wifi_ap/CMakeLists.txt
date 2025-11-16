idf_component_register(
    SRCS "wifi_ap.c"
    INCLUDE_DIRS "include"
    REQUIRES nvs_flash esp_netif esp_wifi esp_event
)
