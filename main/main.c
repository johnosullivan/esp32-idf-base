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

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

// WiFI / Bluetooth Managers
#include "wifi_manager/wifi_manager.h"
//#include "bt_manager/bt_manager.h"

// Over The Air (OTA) Updates
#include "esp_https_ota.h"

#include "cJSON.h"

#define MIHOME_VERSION  "v0.0.1"


// BlueTooth
#define BT_DEVICE_NAME   "MIHOME_SWQZ192"

char *read_bt_data;
void parse_bt_json_playload(char *data);


static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A   0x0001
#define GATTS_CHAR_UUID_TEST_A      0x1000
//#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4

//#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t char1_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t a_property = 0;

uint8_t status = 0x00;

static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)


static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};

// 0x5f _

static uint8_t raw_scan_rsp_data[] = {
        0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x4d, 0x49, 0x48, 0x4f, 0x4d, 0x45, 0x00, 0x00, 0x00, 0x00
};


static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    }
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

void write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);



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
#define PING_COMMAND "PING"

led_strip_t *strip;

void init_status_led();
void ping_ws2812_signal();
void update_status_led(char *color_hex);
/* WS2812_END */

/* WS_START */
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
  		// ESP_LOGI(TAG_BASE, ": %d", esp_get_free_heap_size());
  		vTaskDelay(pdMS_TO_TICKS(60000));
  	}
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

        if (strcmp(typeValue, PING_COMMAND) == 0) {
            ping_ws2812_signal();
        }

        ESP_LOGI(TAG_BASE, "WSType: %s", typeValue);
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
	ESP_LOGI(TAG_BASE, "cb_connection_established");

  wifi_ap_record_t wifidata;
  if (esp_wifi_sta_get_ap_info(&wifidata) == 0){
    ESP_LOGI(TAG_BASE, "wifi_rssi: %d", wifidata.rssi);
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

  //update_status_led("00ff00");

  //websocket_app_start();
  //ESP_ERROR_CHECK(do_firmware_upgrade());

  //esp_err_t err = nvs_set_str_value("storage", "hardware_uuid", "this is a test");
  //ESP_ERROR_CHECK(err);

  /*esp_err_t err;
  char uuid[30];
  err = nvs_get_str_value("storage", "hardware_uuid", uuid);
  if (err == ESP_OK) {
    ESP_LOGI(TAG_BASE, "hardware_uuid = %s", uuid);
  }*/

  //websocket_app_start();
  //firmware_upgrade();
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

void init_status_led() {
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(21, RMT_TX_CHANNEL);
    config.clk_div = 2;

    ESP_ERROR_CHECK(rmt_config(&config));

    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    led_strip_config_t strip_config = LED_STRIP_DEFAULT_CONFIG(1, (led_strip_dev_t)config.channel);

    strip = led_strip_new_rmt_ws2812(&strip_config);
}




static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~adv_config_flag);
            if (adv_config_done==0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~scan_rsp_config_flag);
            if (adv_config_done==0) {
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG_BASE, "ADST Failed");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG_BASE, "ADS Failed");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
             ESP_LOGI(TAG_BASE, "UCPS = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                      param->update_conn_params.status,
                      param->update_conn_params.min_int,
                      param->update_conn_params.max_int,
                      param->update_conn_params.conn_int,
                      param->update_conn_params.latency,
                      param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp){
        if (param->write.is_prep){
            if (prepare_write_env->prepare_buf == NULL) {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE*sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL) {
                    ESP_LOGE(TAG_BASE, "Gatt_server prep no mem\n");
                    status = ESP_GATT_NO_RESOURCES;
                }
            } else {
                if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_OFFSET;
                } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
                    status = ESP_GATT_INVALID_ATTR_LEN;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);

            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK) {
               ESP_LOGE(TAG_BASE, "SEND_RES_ERROR");
            }
            free(gatt_rsp);
            if (status != ESP_GATT_OK) {
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;
        } else {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        esp_log_buffer_hex(TAG_BASE, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(TAG_BASE,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG_BASE, "ESP_GATTS_REG_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);

            gl_profile_tab[PROFILE_APP_ID].service_id.is_primary = true;
            gl_profile_tab[PROFILE_APP_ID].service_id.id.inst_id = 0x00;
            gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(BT_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(TAG_BASE, "set device name failed, error code = %x", set_dev_name_ret);
            }

            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(TAG_BASE, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= adv_config_flag;

            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(TAG_BASE, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= scan_rsp_config_flag;

            esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
            break;
        case ESP_GATTS_READ_EVT: {
            ESP_LOGI(TAG_BASE, "ESP_GATTS_READ_EVT, conn_id %d, trans_id %d, handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);

            esp_gatt_rsp_t rsp;

            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;

            uint8_t * uhex = (uint8_t *)read_bt_data;
            int size = strlen(read_bt_data);

            ESP_LOGI(TAG_BASE, "ESP_GATTS_READ_EVT_VALUE: %s", read_bt_data);
            ESP_LOGI(TAG_BASE, "ESP_GATTS_READ_EVT_VALUE_PS: %d", size);

            rsp.attr_value.len = size;

            for(int i = 0; i < size; i += 1)
            {
              rsp.attr_value.value[i] = uhex[i];
            }

            //rsp.attr_value.value[0] = 0xde;
            //rsp.attr_value.value[1] = 0xed;
            //rsp.attr_value.value[2] = 0xbe;
            //rsp.attr_value.value[3] = 0xef;

            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            break;
        }
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(TAG_BASE, "ESP_GATTS_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
            if (!param->write.is_prep){
                ESP_LOGI(TAG_BASE, "ESP_GATTS_WRITE_EVT, value len %d", param->write.len);

                char rcv_buffer[param->write.len];
                strcpy(rcv_buffer,(char*)param->write.value);
                ESP_LOGI(TAG_BASE, "rcv_buffer: %s", rcv_buffer);

                parse_bt_json_playload(rcv_buffer);

                esp_log_buffer_hex(TAG_BASE, param->write.value, param->write.len);

                /*if (gl_profile_tab[PROFILE_APP_ID].descr_handle == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY){
                            ESP_LOGI(TAG_BASE, "notify enable");
                            uint8_t notify_data[15];
                            for (int i = 0; i < sizeof(notify_data); ++i)
                            {
                                notify_data[i] = i%0xff;
                            }
                            //the size of notify_data[] need less than MTU size
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_APP_ID].char_handle,
                                                    sizeof(notify_data), notify_data, false);
                        }
                    }else if (descr_value == 0x0002){
                        if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE){
                            ESP_LOGI(TAG_BASE, "indicate enable");
                            uint8_t indicate_data[15];
                            for (int i = 0; i < sizeof(indicate_data); ++i)
                            {
                                indicate_data[i] = i%0xff;
                            }
                            //the size of indicate_data[] need less than MTU size
                            esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_APP_ID].char_handle,
                                                    sizeof(indicate_data), indicate_data, true);
                        }
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(TAG_BASE, "notify/indicate disable ");
                    }else{
                        ESP_LOGE(TAG_BASE, "unknown descr value");
                        esp_log_buffer_hex(TAG_BASE, param->write.value, param->write.len);
                    }

                }*/
            }
            write_event_env(gatts_if, &prepare_write_env, param);
            break;
        }
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(TAG_BASE,"ESP_GATTS_EXEC_WRITE_EVT");
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(TAG_BASE, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_UNREG_EVT:
            break;
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG_BASE, "ESP_GATTS_CREATE_EVT, status %d, service_handle %d", param->create.status, param->create.service_handle);

            gl_profile_tab[PROFILE_APP_ID].service_handle = param->create.service_handle;
            gl_profile_tab[PROFILE_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

            esp_ble_gatts_start_service(gl_profile_tab[PROFILE_APP_ID].service_handle);

            a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

            esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_APP_ID].service_handle, &gl_profile_tab[PROFILE_APP_ID].char_uuid,
                                                            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                            a_property,
                                                            &gatts_demo_char1_val, NULL);
            if (add_char_ret){
                ESP_LOGE(TAG_BASE, "ACF, EC =%x",add_char_ret);
            }
            break;
        case ESP_GATTS_ADD_INCL_SRVC_EVT:
            break;
        case ESP_GATTS_ADD_CHAR_EVT: {
            /*uint16_t length = 0;
            const uint8_t *prf_char;

            ESP_LOGI(TAG_BASE, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d\n",
                    param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            gl_profile_tab[PROFILE_APP_ID].char_handle = param->add_char.attr_handle;
            gl_profile_tab[PROFILE_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
            gl_profile_tab[PROFILE_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
            esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,  &length, &prf_char);
            if (get_attr_ret == ESP_FAIL){
                ESP_LOGE(TAG_BASE, "ILLEGAL HANDLE");
            }

            ESP_LOGI(TAG_BASE, "the gatts demo char length = %x\n", length);
            for(int i = 0; i < length; i++){
                ESP_LOGI(TAG_BASE, "prf_char[%x] =%x\n",i,prf_char[i]);
            }
            esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_APP_ID].service_handle, &gl_profile_tab[PROFILE_APP_ID].descr_uuid,
                                                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
            if (add_descr_ret){
                ESP_LOGE(TAG_BASE, "add char descr failed, error code =%x", add_descr_ret);
            }*/
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            gl_profile_tab[PROFILE_APP_ID].descr_handle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG_BASE, "ESP_GATTS_ADD_CHAR_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                     param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
            break;
        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG_BASE, "ESP_GATTS_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT: {
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));

            /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;     // timeout = 400*10ms = 4000ms

            ESP_LOGI(TAG_BASE, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x",
                     param->connect.conn_id,
                     param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                     param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
            gl_profile_tab[PROFILE_APP_ID].conn_id = param->connect.conn_id;

            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG_BASE, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(TAG_BASE, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
            if (param->conf.status != ESP_GATT_OK){
                esp_log_buffer_hex(TAG_BASE, param->conf.value, param->conf.len);
            }
            break;
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        default:
            break;
    }
}














void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(TAG_BASE, "RAF APP_ID: %04x, Status: %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void parse_bt_json_playload(char *data) {
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

        if (strcmp(typeValue, "INFO") == 0) {
            cJSON *root;
            esp_chip_info_t chip_info;
            esp_chip_info(&chip_info);

            root = cJSON_CreateObject();

            cJSON_AddStringToObject(root, "mh_version", MIHOME_VERSION);
            cJSON_AddNumberToObject(root, "num_cores", chip_info.cores);
            cJSON_AddNumberToObject(root, "revision", chip_info.revision);
            cJSON_AddNumberToObject(root, "flash", spi_flash_get_chip_size() / (1024 * 1024));
            cJSON_AddStringToObject(root, "device_name", BT_DEVICE_NAME);

            read_bt_data = cJSON_PrintUnformatted(root);

            cJSON_Delete(root);
        }

        ESP_LOGI(TAG_BASE, "BT_Type: %s", typeValue);
        goto end;
    } else {
        goto end;
    }

    end:
      cJSON_Delete(json_obj);
}

void app_main(void)
{
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

    ESP_ERROR_CHECK(nvs_flash_init());

    //gpio_pad_select_gpio(BLINK_GPIO);
    //gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

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

    //init_status_led();
    //update_status_led("0000ff");

    //wifi_manager_start();
    //wifi_manager_set_callback(EVENT_STA_GOT_IP, &cb_connection_established);

    //bt_manager_start();
    //bt_manager_set_callback(EVENT_STA_GOT_IP, &cb_connection_established);

    /*vTaskDelay(5000 / portTICK_PERIOD_MS);

    ping_ws2812_signal();

    update_status_led("0000ff");*/

    /*
    cJSON *root;
    root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "hwid", "myhwid");

    char *renderedJSON = cJSON_PrintUnformatted(root);
    ESP_LOGI(TAG_BASE, "%s", renderedJSON);

    cJSON_Delete(root);
    */

    //Get MAC address for Bluetooth
    /*ESP_ERROR_CHECK(esp_read_mac(derived_mac_addr, ESP_MAC_BT));
    ESP_LOGI("BT MAC", "%x:%x:%x:%x:%x:%x",
             derived_mac_addr[0], derived_mac_addr[1], derived_mac_addr[2],
             derived_mac_addr[3], derived_mac_addr[4], derived_mac_addr[5]);*/

    /*
    while(true){
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    */


    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG_BASE, "%s ICF001: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG_BASE, "%s EBF001: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG_BASE, "%s IBF001: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG_BASE, "%s EBF002: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG_BASE, "GATTS_F001, EC = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG_BASE, "GATTS_F002, EC = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
    if (ret) {
        ESP_LOGE(TAG_BASE, "GATTS_F003, EC = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret) {
        ESP_LOGE(TAG_BASE, "MTU Failed, EC = %x", local_mtu_ret);
    }

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
