#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_system.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "constants.h"

#include "utils.h"

/* WS2812_START */
#include "ws2812.h"
#include "driver/rmt.h"

#define RMT_TX_CHANNEL RMT_CHANNEL_0

led_strip_t *strip;
/* WS2812_END */

static const char *TAG_UTILS = "mihome_esp32_utils";

static void wifi_scanner_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
   if (event_id == SYSTEM_EVENT_SCAN_DONE) {
      uint16_t apCount = 0;
      esp_wifi_scan_get_ap_num(&apCount);
      if (apCount == 0) { }
      wifi_ap_record_t *list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * apCount);
      ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, list));
      int i;
      ESP_LOGI(TAG_UTILS, "======================================================================");
      ESP_LOGI(TAG_UTILS, "             SSID             |    RSSI    |           AUTH           ");
      ESP_LOGI(TAG_UTILS, "======================================================================");
      for (i=0; i<apCount; i++) {
         char *authmode;
         switch(list[i].authmode) {
            case WIFI_AUTH_OPEN:
               authmode = "WIFI_AUTH_OPEN";
               break;
            case WIFI_AUTH_WEP:
               authmode = "WIFI_AUTH_WEP";
               break;
            case WIFI_AUTH_WPA_PSK:
               authmode = "WIFI_AUTH_WPA_PSK";
               break;
            case WIFI_AUTH_WPA2_PSK:
               authmode = "WIFI_AUTH_WPA2_PSK";
               break;
            case WIFI_AUTH_WPA_WPA2_PSK:
               authmode = "WIFI_AUTH_WPA_WPA2_PSK";
               break;
            default:
               authmode = "Unknown";
               break;
         }
         ESP_LOGI(TAG_UTILS, "%26.26s    |    % 4d    |    %22.22s",list[i].ssid, list[i].rssi, authmode);
      }
      free(list);
   }
}

void set_logging_levels() {
  /* disable the default wifi logging */
  if (WIFI_CORE_LOGGING != 0) {
    esp_log_level_set("wifi", ESP_LOG_NONE);
  }
}

void wifi_scanner() {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    esp_event_handler_instance_t event_instance_wifi_scanner;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_scanner_handler, NULL, &event_instance_wifi_scanner));

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_scan_config_t scanConf = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true
    };

    ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, true));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, event_instance_wifi_scanner));
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_event_loop_delete_default());
}

esp_err_t nvs_get_str_value(char *namespace, char *key, char *ref) {
    esp_err_t err;
    nvs_handle_t nvs_handle;
    err = nvs_open(namespace, NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK) {
      return err;
    } else {
      size_t required_size = 0;
      err = nvs_get_str(nvs_handle, key, ref, &required_size);

      switch (err) {
          case ESP_OK:
              break;
          case ESP_ERR_NVS_NOT_FOUND:
              break;
          default:
              ESP_LOGI(TAG_UTILS, "nvs_get_str_value_error = (%s)", esp_err_to_name(err));
      }
    }
    nvs_close(nvs_handle);
    return err;
}

esp_err_t nvs_set_str_value(char *namespace, char *key, char *ref) {
    esp_err_t err;
    nvs_handle_t nvs_handle;
    err = nvs_open(namespace, NVS_READWRITE, &nvs_handle);

    if (err != ESP_OK) {
      return err;
    } else {
      err = nvs_set_str(nvs_handle, key, ref);
      err = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    return err;
}

void init_status_led(uint8_t pin) {
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(pin, RMT_TX_CHANNEL);
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));

    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(1, (led_strip_dev_t)config.channel);

    strip = led_strip_new_rmt_ws2812(&strip_config);
}

void update_status_led(char *color_hex) {
    int r, g, b;
    sscanf(color_hex, "%02x%02x%02x", &r, &g, &b);
    ESP_ERROR_CHECK(strip->clear(strip, 0));
    ESP_ERROR_CHECK(strip->set_pixel(strip, 0, r, g, b));
    ESP_ERROR_CHECK(strip->refresh(strip, 0));
}

void ping_ws2812_signal() {
    for (int i = 1; i < 8; i++) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        update_status_led("9400d3");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        update_status_led("4b0082");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        update_status_led("0000ff");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        update_status_led("00ff00");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        update_status_led("ffff00");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        update_status_led("ff7f00");
        vTaskDelay(100 / portTICK_PERIOD_MS);
        update_status_led("ff0000");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    update_status_led("00ff00");
}