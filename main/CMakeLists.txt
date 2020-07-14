idf_component_register(
    SRCS  "main.c"
          "utils.c"
          "wifi_manager/wifi_manager.c"
          "wifi_manager/dns_server.c"
          "wifi_manager/http_server.c"
          "wifi_manager/json.c"
    REQUIRES log nvs_flash mdns wpa_supplicant lwip esp_http_server
    INCLUDE_DIRS "."
    EMBED_FILES wifi_manager/style.css wifi_manager/code.js wifi_manager/index.html)
