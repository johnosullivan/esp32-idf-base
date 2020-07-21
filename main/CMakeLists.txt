idf_component_register (
    SRCS  "main.c"
          "cJSON.c"
          "utils.c"
          "ws2812.c"
          "bt_manager/blufi_security.c"
          "bt_manager/bt_manager.c"
          "wifi_manager/wifi_manager.c"
          "wifi_manager/dns_server.c"
          "wifi_manager/http_server.c"
          "wifi_manager/json.c"
    REQUIRES log nvs_flash mdns wpa_supplicant lwip esp_http_server bt esp_http_client esp_websocket_client esp_https_ota
    INCLUDE_DIRS "."
    EMBED_FILES wifi_manager/style.css wifi_manager/code.js wifi_manager/index.html api_cert.pem
)
