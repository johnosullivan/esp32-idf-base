#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "freertos/event_groups.h"

#include "nvs.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_system.h"

#include "esp_http_client.h"
#include "esp_websocket_client.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "utils.h"
#include "constants.h"

// WiFI / Bluetooth Managers
#include "wifi_manager/wifi_manager.h"
#include "bt_manager/bt_manager.h"

// Over The Air (OTA) Updates
#include "esp_https_ota.h"

static const char *TAG_BASE = "mihome_esp32_base";

#define MAX_HTTP_OUTPUT_BUFFER 2048
#define NO_DATA_TIMEOUT_SEC 120

// Certs PEMS
extern const char api_cert_pem_start[] asm("_binary_api_cert_pem_start");
extern const char api_cert_pem_end[]   asm("_binary_api_cert_pem_end");

/* WS2812_START */
#include "ws2812.h"
#include "driver/rmt.h"

#define RMT_TX_CHANNEL RMT_CHANNEL_0

led_strip_t *strip;
void init_status_led();
void update_status_led(char *color_hex);
/* WS2812_END */

esp_err_t firmware_upgrade();

// Secure Websocket timer and handles
static TimerHandle_t shutdown_signal_timer;
static SemaphoreHandle_t shutdown_sema;

esp_err_t _http_event_handle(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGI(TAG_BASE, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGI(TAG_BASE, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGI(TAG_BASE, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGI(TAG_BASE, "HTTP_EVENT_ON_HEADER");
            printf("%.*s", evt->data_len, (char*)evt->data);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGI(TAG_BASE, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            if (!esp_http_client_is_chunked_response(evt->client)) {
                ESP_LOGI(TAG_BASE, "%.*s", evt->data_len, (char*)evt->data);
            }
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGI(TAG_BASE, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG_BASE, "HTTP_EVENT_DISCONNECTED");
            break;
    }
    return ESP_OK;
}

void system_monitoring_task(void *pvParameter) {
	for(;;) {
		ESP_LOGI(TAG_BASE, ": %d", esp_get_free_heap_size());
		vTaskDelay(pdMS_TO_TICKS(60000));
	}
}

static void shutdown_signaler(TimerHandle_t xTimer) {
    ESP_LOGI(TAG_BASE, "No data received for %d seconds, signaling shutdown", NO_DATA_TIMEOUT_SEC);
    xSemaphoreGive(shutdown_sema);
}

static void websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;
    switch (event_id) {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG_BASE, "WEBSOCKET_EVENT_CONNECTED");
        break;
    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_BASE, "WEBSOCKET_EVENT_DISCONNECTED");
        break;
    case WEBSOCKET_EVENT_DATA:
        if (data->payload_len != 0) {
          ESP_LOGI(TAG_BASE, "WEBSOCKET_EVENT_DATA");
          ESP_LOGI(TAG_BASE, "Received opcode=%d", data->op_code);
          ESP_LOGW(TAG_BASE, "Received=%.*s", data->data_len, (char *)data->data_ptr);
          ESP_LOGW(TAG_BASE, "Total payload length=%d, data_len=%d, current payload offset=%d", data->payload_len, data->data_len, data->payload_offset);

          char rcv_buffer[data->data_len];
          strcpy(rcv_buffer,(char*)data->data_ptr);
          //update_status_led(rcv_buffer);
          esp_err_t err = nvs_set_str_value("storage", "hardware_uuid", rcv_buffer);
          ESP_ERROR_CHECK(err);
        }

        xTimerReset(shutdown_signal_timer, portMAX_DELAY);
        break;
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGI(TAG_BASE, "WEBSOCKET_EVENT_ERROR");
        break;
    }
}

static void websocket_app_start(void) {
    esp_websocket_client_config_t websocket_cfg = {
      .uri = "ws://10.0.0.241:3000/ws",
      //.cert_pem = api_cert_pem_start
    };

    shutdown_signal_timer = xTimerCreate("ws_s_t", NO_DATA_TIMEOUT_SEC * 1000 / portTICK_PERIOD_MS, pdFALSE, NULL, shutdown_signaler);

    shutdown_sema = xSemaphoreCreateBinary();

    ESP_LOGI(TAG_BASE, "Connecting to %s...", websocket_cfg.uri);

    esp_websocket_client_handle_t client = esp_websocket_client_init(&websocket_cfg);

    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);

    esp_websocket_client_start(client);

    xTimerStart(shutdown_signal_timer, portMAX_DELAY);

    /*
    char data[32];
    int i = 0;
    while (i < 20) {
        if (esp_websocket_client_is_connected(client)) {
            int len = sprintf(data, "hello %04d", i++);
            ESP_LOGI(TAG_BASE, "Sending %s", data);
            esp_websocket_client_send_text(client, data, len, portMAX_DELAY);
        }
        vTaskDelay(5000 / portTICK_RATE_MS);
    }
    */

    xSemaphoreTake(shutdown_sema, portMAX_DELAY);
    esp_websocket_client_stop(client);
    esp_websocket_client_destroy(client);
}


void cb_connection_established(void *pvParameter) {
	ESP_LOGI(TAG_BASE, "cb_connection_established");

  wifi_ap_record_t wifidata;
  if (esp_wifi_sta_get_ap_info(&wifidata) == 0){
    ESP_LOGI(TAG_BASE, "rssi: %d", wifidata.rssi);
  }

  /*
  esp_http_client_config_t config = {
        .url = "https://9dd3b6ef0e4e.ngrok.io/ping",
        .cert_pem = api_cert_pem_start,
        .event_handler = _http_event_handle,
  };
  esp_http_client_handle_t client = esp_http_client_init(&config);
  esp_err_t err = esp_http_client_perform(client);

  if (err == ESP_OK) {
    ESP_LOGI(TAG_BASE, "Status = %d, content_length = %d",
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
  }
  esp_http_client_cleanup(client);
  */

  update_status_led("00ff00");

  //websocket_app_start();
  //ESP_ERROR_CHECK(do_firmware_upgrade());

  //esp_err_t err = nvs_set_str_value("storage", "hardware_uuid", "this is a test");
  //ESP_ERROR_CHECK(err);

  esp_err_t err;
  char uuid[30];
  err = nvs_get_str_value("storage", "hardware_uuid", uuid);
  if (err == ESP_OK) {
    ESP_LOGI(TAG_BASE, "hardware_uuid = %s", uuid);
  }

  websocket_app_start();
}

void update_status_led(char *color_hex) {
    int r, g, b;
    sscanf(color_hex, "%02x%02x%02x", &r, &g, &b);
    ESP_ERROR_CHECK(strip->clear(strip, 0));
    ESP_ERROR_CHECK(strip->set_pixel(strip, 0, r, g, b));
    ESP_ERROR_CHECK(strip->refresh(strip, 0));
}

esp_err_t firmware_upgrade()
{
    update_status_led("ffea6b");
    esp_http_client_config_t config = {
        .url = "http://10.0.0.241:8000/esp32mihome.bin",
        //.cert_pem = api_cert_pem_start,
        .timeout_ms = 60000
    };

    esp_err_t ret = esp_https_ota(&config);
    if (ret == ESP_OK) {
        esp_restart();
    } else {
        return ESP_FAIL;
    }
    return ESP_OK;
}

void init_status_led() {
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(4, RMT_TX_CHANNEL);
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));

    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(1, (led_strip_dev_t)config.channel);

    strip = led_strip_new_rmt_ws2812(&strip_config);
}

void app_main(void)
{
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG_BASE, "MiHome ESP32 Base v0.0.1 (%s chip, %d CPU cores, WiFi%s%s, revision %d, %dMB %s flash, free heap: %d)",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
            chip_info.revision,
            spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external",
            esp_get_free_heap_size());

		set_logging_levels();

    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

		/*if (CONNECT_IOT_OPTION) {
				bt_manager_start();
				bt_manager_set_callback(EVENT_STA_GOT_IP, &cb_connection_established);
		} else {
				wifi_manager_start();
				wifi_manager_set_callback(EVENT_STA_GOT_IP, &cb_connection_established);
		}*/

    /*
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
    */

    /*
    wifi_manager_start();
    wifi_manager_set_callback(EVENT_STA_GOT_IP, &cb_connection_established);

    init_status_led();
    update_status_led("0000ff");
    */

    /*
    while(true){
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    */

    xTaskCreatePinnedToCore(&system_monitoring_task, "system_monitoring_task", 2048, NULL, 1, NULL, 1);
}

/*
idf.py menuconfig
idf.py build
idf.py -p /dev/cu.usbserial-D3070SV6 flash monitor

. $HOME/esp/esp-idf/export.sh
IDF_PATH=/Users/johnosullivan/esp/esp-idf
IDF_TOOLS_EXPORT_CMD=/Users/johnosullivan/esp/esp-idf/export.sh
IDF_TOOLS_INSTALL_CMD=/Users/johnosullivan/esp/esp-idf/install.sh
IDF_PYTHON_ENV_PATH=/Users/johnosullivan/.espressif/python_env/idf4.2_py2.7_env
*/
