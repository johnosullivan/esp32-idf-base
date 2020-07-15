#include <stdio.h>
#include <stdlib.h>
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

#include "esp_http_client.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "utils.h"
#include "constants.h"

// wifi / bluetooth managers
#include "wifi_manager/wifi_manager.h"
#include "bt_manager/bt_manager.h"

static const char *TAG_BASE = "mihome_esp32_base";

/*
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
                printf("%.*s", evt->data_len, (char*)evt->data);
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
*/

void cb_connection_established(void *pvParameter){
	ESP_LOGI(TAG_BASE, "I have a connection!");
	ESP_LOGI(TAG_BASE, MIHOME_API_URI);

  /*
	esp_http_client_config_t config = {
   .url = "http://5715f3e3c017.ngrok.io/ping",
   .event_handler = _http_event_handle,
	};

	esp_http_client_handle_t client = esp_http_client_init(&config);
	esp_err_t err = esp_http_client_perform(client);

	if (err == ESP_OK) {
	   ESP_LOGI(TAG_BASE, "status = %d, content_length = %d", esp_http_client_get_status_code(client), esp_http_client_get_content_length(client));
	}

	esp_http_client_cleanup(client);
  */
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

		if (CONNECT_IOT_OPTION) {
				bt_manager_start();
				bt_manager_set_callback(EVENT_STA_GOT_IP, &cb_connection_established);
		} else {
				wifi_manager_start();
				wifi_manager_set_callback(EVENT_STA_GOT_IP, &cb_connection_established);
		}

    while(true){
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/*
idf.py menuconfig
idf.py build
idf.py -p /dev/cu.usbserial-D3070SV6 flash monitor

IDF_PATH=/Users/johnosullivan/esp/esp-idf
IDF_TOOLS_EXPORT_CMD=/Users/johnosullivan/esp/esp-idf/export.sh
IDF_TOOLS_INSTALL_CMD=/Users/johnosullivan/esp/esp-idf/install.sh
IDF_PYTHON_ENV_PATH=/Users/johnosullivan/.espressif/python_env/idf4.2_py2.7_env
*/
