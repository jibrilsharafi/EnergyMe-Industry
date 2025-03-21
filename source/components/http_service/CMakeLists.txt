idf_component_register(
    SRCS "http_service.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server spiffs json esp_https_ota app_update led_control mbedtls esp_netif mux_control
    EMBED_TXTFILES "web/index.html" "web/main.js"
)