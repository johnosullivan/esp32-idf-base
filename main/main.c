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

// mbed support
#include "mbedtls/rsa.h"
#include "mbedtls/gcm.h"
#include "mbedtls/platform.h"
#include "mbedtls/net.h"
#include "mbedtls/esp_debug.h"
#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"
#include "mbedtls/certs.h"
#include "mbedtls/sha256.h" /* SHA-256 only */
#include "mbedtls/md.h"     /* generic interface */

#define MAX_HTTP_OUTPUT_BUFFER 2048
#define NO_DATA_TIMEOUT_SEC 120

struct mihome_settings_t mihome_settings = {
    .cloud_url = "nrmorrdxxmlpvgcokpzifcgsgrfzxqeo",
    .cloud_uuid = "nrmorrdxxmlpvgcokpzifcgsgrfzxqeo"
};

static const char *TAG_BASE = "mihome_esp32_base";

/*
 Secure Websocket timer and handles
*/
static TimerHandle_t shutdown_signal_timer;
static SemaphoreHandle_t shutdown_sema;

void processing_ws_data(char *data);
/* WS_END */

esp_err_t firmware_upgrade();

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
                strncpy(rcv_buffer, (char*)data->data_ptr, data->data_len); // TODO: BUG CORE #0 PANIC strcpy
                ESP_LOGI(TAG_BASE, "processing_ws_data");
                processing_ws_data(rcv_buffer);
            }
            xTimerReset(shutdown_signal_timer, portMAX_DELAY);
            break;
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGI(TAG_BASE, "WEBSOCKET_EVENT_ERROR");
            break;
    }
}

void processing_ws_data(char *data) {
    cJSON *json_obj = cJSON_Parse(data);
    const cJSON *type = NULL;

    if (json_obj == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            ESP_LOGI(TAG_BASE, " error before: %s", error_ptr);
        }
        goto end;
    }

    type = cJSON_GetObjectItemCaseSensitive(json_obj, "type");

    if (cJSON_IsString(type) && (type->valuestring != NULL))
    {
        char *typeValue = type->valuestring;

        ESP_LOGI(TAG_BASE, "WSType: %s", typeValue);

        if (strcmp(typeValue, PING_COMMAND) == 0) {
            ping_ws2812_signal();
        }

        goto end;
    } else {
        goto end;
    }

    end:
      cJSON_Delete(json_obj);
}

static void websocket_app_start(void) {
    esp_websocket_client_config_t websocket_cfg = {
      .uri = "ws://mihomecloud.herokuapp.com/ws",
      //.cert_pem = api_cert_pem_start
    };

    shutdown_signal_timer = xTimerCreate("ws_s_t", NO_DATA_TIMEOUT_SEC * 1000 / portTICK_PERIOD_MS, pdFALSE, NULL, shutdown_signaler);

    shutdown_sema = xSemaphoreCreateBinary();

    ESP_LOGI(TAG_BASE, "Connecting to %s...", websocket_cfg.uri);

    esp_websocket_client_handle_t client = esp_websocket_client_init(&websocket_cfg);

    esp_websocket_register_events(client, WEBSOCKET_EVENT_ANY, websocket_event_handler, (void *)client);

    esp_websocket_client_start(client);

    xTimerStart(shutdown_signal_timer, portMAX_DELAY);

    xSemaphoreTake(shutdown_sema, portMAX_DELAY);

    esp_websocket_client_stop(client);
    esp_websocket_client_destroy(client);
}

void cb_connection_established(void *pvParameter) {
	ESP_LOGI(TAG_BASE, "connection_established");

  wifi_ap_record_t wifidata;
  if (esp_wifi_sta_get_ap_info(&wifidata) == 0){
    ESP_LOGI(TAG_BASE, "wifi_rssi: %d", wifidata.rssi);
  }


  update_status_led(HEX_COLOR_GREEN);
  websocket_app_start();
}

esp_err_t firmware_upgrade()
{
    update_status_led("ffea6b");
    esp_http_client_config_t config = {
        .url = "http://10.0.0.245:8000/esp32mihome.bin",
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

static void print_hex(const char *title, const unsigned char buf[], size_t len)
{
    mbedtls_printf("%s: ", title);

    for (size_t i = 0; i < len; i++) {
        if (buf[i] < 0xF) {
            mbedtls_printf("0%x", buf[i]);
        } else {
            mbedtls_printf("%x", buf[i]);
        }
    }

    mbedtls_printf("\n");
}

void app_main(void)
{   
    const char hello_str[] = "Hello, World!";
    const unsigned char *hello_buffer = (const unsigned char *) hello_str;
    const size_t hello_len = strlen(hello_str);

    unsigned char output1[32]; /* SHA-256 outputs 32 bytes */

    /* 0 here means use the full SHA-256, not the SHA-224 variant */
    mbedtls_sha256(hello_buffer, hello_len, output1, 0);
    print_hex("SHA256 - ", output1, sizeof output1);
        
    /*
    mbedtls_gcm_context aes;

    char *key = "nO@dz3u1ivEHUBZBir%04S*qcT@NjGei";
    char *input = "2020 does really suck!";
    char *iv = "mihomecore";

    unsigned char output[64] = {0};
    unsigned char fin[64] = {0};
    unsigned char tag[16] = {0};
  
    mbedtls_gcm_init(&aes);
    mbedtls_gcm_setkey(&aes,MBEDTLS_CIPHER_ID_AES , (const unsigned char*) key, strlen(key) * 8);
    mbedtls_gcm_starts(&aes, MBEDTLS_GCM_ENCRYPT, (const unsigned char*)iv, strlen(iv),NULL, 0);
    mbedtls_gcm_update(&aes,strlen(input),(const unsigned char*)input, output);
    mbedtls_gcm_finish(&aes, (unsigned char*)tag, 16); 
    mbedtls_gcm_free(&aes);

    print_hex("AES_GCM_T - ", tag, sizeof tag);

    printf("AES_GCM_E - : ");
  
    for (int i = 0; i < strlen(input); i++) {  
        char str[3];
        sprintf(str, "%02x", (int)output[i]);
        printf(str);
    }

    printf("\nAES_GCM_D - : ");
  
    mbedtls_gcm_init(&aes);
    mbedtls_gcm_setkey(&aes,MBEDTLS_CIPHER_ID_AES , (const unsigned char*) key, strlen(key) * 8);
    mbedtls_gcm_starts(&aes, MBEDTLS_GCM_DECRYPT, (const unsigned char*)iv, strlen(iv),NULL, 0);
    mbedtls_gcm_update(&aes,64,(const unsigned char*)output, fin);
    mbedtls_gcm_free(&aes);
  
    for (int i = 0; i < strlen(input); i++) {  
        char str[3];
        sprintf(str, "%c", (int)fin[i]);
        printf(str);
    }

    printf("\n");
    */



    /* grabs the esp chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG_BASE, "MiHome ESP32 Base %s (%s chip, %d CPU cores, WiFi%s%s, revision %d, %dMB %s flash, free heap: %d)",
            MIHOME_VERSION,
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
            chip_info.revision,
            spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external",
            esp_get_free_heap_size());

		set_logging_levels();

    // initialize the ws2812 rgb led
    init_status_led(MIHOME_LED_PIN);
    update_status_led(HEX_COLOR_BLACK);

    // initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // boot config check
    nvs_handle_t handle;
    int8_t requires_hub_restart = 0;
    err = nvs_open(MIHOME_STORAGE_NAMESPACE, NVS_READWRITE, &handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG_BASE, "Error (%s) opening NVS handler on Boot!", esp_err_to_name(err));
    } else {
        int32_t is_configured = 0; // value will default to 0, if not set yet in NVS
        err = nvs_get_i32(handle, MIHOME_STORAGE_IS_CONFIG, &is_configured);
        switch (err) {
            case ESP_OK:
                //ESP_LOGI(TAG_BASE, "configured = %d\n", is_configured);
                if (is_configured) {
                    // begin Wifi manager
                    wifi_manager_start();
                    wifi_manager_set_callback(EVENT_STA_GOT_IP, &cb_connection_established);
                } else {
                    // start BLE gatt service setup.
                    update_status_led(HEX_COLOR_BLUE);
                    bt_manager_start();
                }
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                is_configured = 0;
                err = nvs_set_i32(handle, MIHOME_STORAGE_IS_CONFIG, is_configured);
                err = nvs_commit(handle);
                requires_hub_restart = 1;
                break;
            default:
                ESP_LOGE(TAG_BASE, "Error (%s) reading", esp_err_to_name(err));
        }

        // grabs the mihome cloud size
        size_t mihome_settings_size = sizeof(mihome_settings);
        err = nvs_get_blob(handle, MIHOME_STORAGE_SETTINGS, &mihome_settings, &mihome_settings_size);
        switch (err) {
            case ESP_OK:
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                err = nvs_set_blob(handle, MIHOME_STORAGE_SETTINGS, &mihome_settings, mihome_settings_size);
                err = nvs_commit(handle);
                requires_hub_restart = 1;
                break;
            default:
                ESP_LOGE(TAG_BASE, "Error (%s) reading", esp_err_to_name(err));
        }
    }

    // close NVS handler
    nvs_close(handle);

    if (requires_hub_restart) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        esp_restart();
    }

    /*
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
    */

    /*
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
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
idf.py -p /dev/cu.usbserial-D3070SV6 erase_flash
. $HOME/esp/esp-idf/export.sh
*/
