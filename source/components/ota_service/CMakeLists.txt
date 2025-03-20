idf_component_register(
    SRCS "ota_service.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_http_server esp_http_client esp_https_ota app_update esp_event led_control json
)