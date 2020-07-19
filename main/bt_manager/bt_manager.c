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

#include "lwip/err.h"
#include "lwip/sys.h"

#include "blufi_security.h"
#include "bt_manager.h"

static const char *TAG_BT_MG = "mihome_esp32_bt_manager";

void (**cb_ptr_arr)(void*) = NULL;

#define BLUFI_DEVICE_NAME  CONFIG_BLUFI_DEVICE_NAME
#define BT_WIFI_LIST_NUM  CONFIG_BT_WIFI_LIST_NUM

const int BT_MANAGER_WIFI_CONNECTED_BIT = BIT0;

static uint8_t service_uuid128[32] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 16,
    .p_service_uuid = service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x100,
    .adv_int_max        = 0x100,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_blufi_callbacks_t bt_callbacks = {
    .event_cb = bt_event_callback,
    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
    .encrypt_func = blufi_aes_encrypt,
    .decrypt_func = blufi_aes_decrypt,
    .checksum_func = blufi_crc_checksum,
};

static EventGroupHandle_t wifi_event_group;
static wifi_config_t sta_config;
static wifi_config_t ap_config;
static bool gl_sta_connected = false;
static bool ble_is_connected = false;
static uint8_t gl_sta_bssid[6];
static uint8_t gl_sta_ssid[32];
static int gl_sta_ssid_len;
static uint8_t server_if;
static uint16_t conn_id;

static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    wifi_mode_t mode;

    switch (event_id) {
        case IP_EVENT_STA_GOT_IP: {
            esp_blufi_extra_info_t info;

            xEventGroupSetBits(wifi_event_group, BT_MANAGER_WIFI_CONNECTED_BIT);
            esp_wifi_get_mode(&mode);

            memset(&info, 0, sizeof(esp_blufi_extra_info_t));
            memcpy(info.sta_bssid, gl_sta_bssid, 6);
            info.sta_bssid_set = true;
            info.sta_ssid = gl_sta_ssid;
            info.sta_ssid_len = gl_sta_ssid_len;
            if (ble_is_connected == true) {
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, &info);
            } else {
                ESP_LOGI(TAG_BT_MG, "BLUFI/BLE/NC");
            }

    				if(cb_ptr_arr[EVENT_STA_GOT_IP]) (*cb_ptr_arr[EVENT_STA_GOT_IP])(NULL);
            break;
        }
        default:
            break;
    }
    return;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    wifi_event_sta_connected_t *event;
    wifi_mode_t mode;

    switch (event_id) {
        case WIFI_EVENT_STA_START:
            esp_wifi_connect();
            ESP_LOGI(TAG_BT_MG, "BLUFI/STA/START");
            break;
        case WIFI_EVENT_STA_CONNECTED:
            gl_sta_connected = true;
            event = (wifi_event_sta_connected_t*) event_data;
            memcpy(gl_sta_bssid, event->bssid, 6);
            memcpy(gl_sta_ssid, event->ssid, event->ssid_len);
            gl_sta_ssid_len = event->ssid_len;
            ESP_LOGI(TAG_BT_MG, "BLUFI/STA/CONNECTED");

            esp_ble_gap_stop_advertising();
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            gl_sta_connected = false;
            memset(gl_sta_ssid, 0, 32);
            memset(gl_sta_bssid, 0, 6);
            gl_sta_ssid_len = 0;
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, BT_MANAGER_WIFI_CONNECTED_BIT);
            break;
        case WIFI_EVENT_AP_START:
            esp_wifi_get_mode(&mode);
            if (ble_is_connected == true) {
                if (gl_sta_connected) {
                    esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, NULL);
                } else {
                    esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0, NULL);
                }
            }
            break;
        case WIFI_EVENT_SCAN_DONE: {
            uint16_t apCount = 0;
            esp_wifi_scan_get_ap_num(&apCount);
            if (apCount == 0) {
                break;
            }

            wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * apCount);
            if (!ap_list) {
                ESP_LOGE(TAG_BT_MG, "malloc error, ap_list is NULL");
                break;
            }
            ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, ap_list));

            esp_blufi_ap_record_t * blufi_ap_list = (esp_blufi_ap_record_t *)malloc(apCount * sizeof(esp_blufi_ap_record_t));
            if (!blufi_ap_list) {
                if (ap_list) {
                    free(ap_list);
                }
                ESP_LOGE(TAG_BT_MG, "malloc error, blufi_ap_list is NULL");
                break;
            }
            for (int i = 0; i < apCount; ++i) {
                blufi_ap_list[i].rssi = ap_list[i].rssi;
                memcpy(blufi_ap_list[i].ssid, ap_list[i].ssid, sizeof(ap_list[i].ssid));
            }
            if (ble_is_connected == true) {
                esp_blufi_send_wifi_list(apCount, blufi_ap_list);
            } else {
                ESP_LOGI(TAG_BT_MG, "BLUFI/BLE/NC");
            }
            esp_wifi_scan_stop();
            free(ap_list);
            free(blufi_ap_list);
            break;
        }
        default:
            break;
    }
    return;
}

static void initialize_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &ip_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

void bt_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param)
{
    switch (event) {
        case ESP_BLUFI_EVENT_INIT_FINISH:
            ESP_LOGI(TAG_BT_MG, "BLUFI/Init");

            esp_ble_gap_set_device_name(BLUFI_DEVICE_NAME);
            esp_ble_gap_config_adv_data(&adv_data);
            break;
        case ESP_BLUFI_EVENT_DEINIT_FINISH:
            ESP_LOGI(TAG_BT_MG, "BLUFI/Deinit");
            break;
        case ESP_BLUFI_EVENT_BLE_CONNECT:
            ESP_LOGI(TAG_BT_MG, "BLUFI/BLE Connect");
            ble_is_connected = true;
            server_if = param->connect.server_if;
            conn_id = param->connect.conn_id;
            esp_ble_gap_stop_advertising();
            blufi_security_init();
            break;
        case ESP_BLUFI_EVENT_BLE_DISCONNECT:
            ESP_LOGI(TAG_BT_MG, "BLUFI/BLE Disconnect");
            ble_is_connected = false;
            blufi_security_deinit();
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
            ESP_LOGI(TAG_BT_MG, "BLUFI WIFI opmode = %d", param->wifi_mode.op_mode);
            ESP_ERROR_CHECK(esp_wifi_set_mode(param->wifi_mode.op_mode));
            break;
        case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
            esp_wifi_disconnect();
            esp_wifi_connect();
            break;
        case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
            ESP_LOGI(TAG_BT_MG, "BLUFI/REQ Wifi Disconnect");
            esp_wifi_disconnect();
            break;
        case ESP_BLUFI_EVENT_REPORT_ERROR:
            ESP_LOGE(TAG_BT_MG, "BLUFI/REPORT/ERR code = %d", param->report_error.state);
            esp_blufi_send_error_info(param->report_error.state);
            break;
        case ESP_BLUFI_EVENT_GET_WIFI_STATUS: {
            wifi_mode_t mode;
            esp_blufi_extra_info_t info;
            esp_wifi_get_mode(&mode);
            if (gl_sta_connected) {
                memset(&info, 0, sizeof(esp_blufi_extra_info_t));
                memcpy(info.sta_bssid, gl_sta_bssid, 6);
                info.sta_bssid_set = true;
                info.sta_ssid = gl_sta_ssid;
                info.sta_ssid_len = gl_sta_ssid_len;
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, &info);
            } else {
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0, NULL);
            }
            ESP_LOGI(TAG_BT_MG, "BLUFI/STATUS/AP");
            break;
        }
        case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
            ESP_LOGI(TAG_BT_MG, "BLUFI/CLOSE/GATT");
            esp_blufi_close(server_if, conn_id);
            break;
        case ESP_BLUFI_EVENT_DEAUTHENTICATE_STA:
            break;
    	  case ESP_BLUFI_EVENT_RECV_STA_BSSID:
            memcpy(sta_config.sta.bssid, param->sta_bssid.bssid, 6);
            sta_config.sta.bssid_set = 1;
            esp_wifi_set_config(WIFI_IF_STA, &sta_config);
            ESP_LOGD(TAG_BT_MG, "STA BSSID %s", sta_config.sta.ssid);
            break;
    	  case ESP_BLUFI_EVENT_RECV_STA_SSID:
            strncpy((char *)sta_config.sta.ssid, (char *)param->sta_ssid.ssid, param->sta_ssid.ssid_len);
            sta_config.sta.ssid[param->sta_ssid.ssid_len] = '\0';
            esp_wifi_set_config(WIFI_IF_STA, &sta_config);
            ESP_LOGD(TAG_BT_MG, "STA SSID %s", sta_config.sta.ssid);
            break;
    	  case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
            strncpy((char *)sta_config.sta.password, (char *)param->sta_passwd.passwd, param->sta_passwd.passwd_len);
            sta_config.sta.password[param->sta_passwd.passwd_len] = '\0';
            esp_wifi_set_config(WIFI_IF_STA, &sta_config);
            ESP_LOGD(TAG_BT_MG, "STA PASSWORD %s", sta_config.sta.password);
            break;
    	  case ESP_BLUFI_EVENT_RECV_SOFTAP_SSID:
            strncpy((char *)ap_config.ap.ssid, (char *)param->softap_ssid.ssid, param->softap_ssid.ssid_len);
            ap_config.ap.ssid[param->softap_ssid.ssid_len] = '\0';
            ap_config.ap.ssid_len = param->softap_ssid.ssid_len;
            esp_wifi_set_config(WIFI_IF_AP, &ap_config);
            ESP_LOGD(TAG_BT_MG, "SOFTAP SSID %s, ssid len %d", ap_config.ap.ssid, ap_config.ap.ssid_len);
            break;
    	  case ESP_BLUFI_EVENT_RECV_SOFTAP_PASSWD:
            strncpy((char *)ap_config.ap.password, (char *)param->softap_passwd.passwd, param->softap_passwd.passwd_len);
            ap_config.ap.password[param->softap_passwd.passwd_len] = '\0';
            esp_wifi_set_config(WIFI_IF_AP, &ap_config);
            ESP_LOGD(TAG_BT_MG, "SOFTAP PASSWORD %s len = %d", ap_config.ap.password, param->softap_passwd.passwd_len);
            break;
    	  case ESP_BLUFI_EVENT_RECV_SOFTAP_MAX_CONN_NUM:
            if (param->softap_max_conn_num.max_conn_num > 4) {
                return;
            }
            ap_config.ap.max_connection = param->softap_max_conn_num.max_conn_num;
            esp_wifi_set_config(WIFI_IF_AP, &ap_config);
            ESP_LOGD(TAG_BT_MG, "SOFTAP MAX CONN NUM %d", ap_config.ap.max_connection);
            break;
    	  case ESP_BLUFI_EVENT_RECV_SOFTAP_AUTH_MODE:
            if (param->softap_auth_mode.auth_mode >= WIFI_AUTH_MAX) {
                return;
            }
            ap_config.ap.authmode = param->softap_auth_mode.auth_mode;
            esp_wifi_set_config(WIFI_IF_AP, &ap_config);
            ESP_LOGD(TAG_BT_MG, "SOFTAP AUTH MODE %d", ap_config.ap.authmode);
            break;
    	  case ESP_BLUFI_EVENT_RECV_SOFTAP_CHANNEL:
            if (param->softap_channel.channel > 13) {
                return;
            }
            ap_config.ap.channel = param->softap_channel.channel;
            esp_wifi_set_config(WIFI_IF_AP, &ap_config);
            ESP_LOGD(TAG_BT_MG, "SOFTAP CHANNEL %d", ap_config.ap.channel);
            break;
        case ESP_BLUFI_EVENT_GET_WIFI_LIST:{
            wifi_scan_config_t scanConf = {
                .ssid = NULL,
                .bssid = NULL,
                .channel = 0,
                .show_hidden = false
            };
            ESP_ERROR_CHECK(esp_wifi_scan_start(&scanConf, true));
            break;
        }
        case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA:
            esp_log_buffer_hex("BLUFI/CD", param->custom_data.data, param->custom_data.data_len);
            break;
    	  case ESP_BLUFI_EVENT_RECV_USERNAME:
            /* Not handle currently */
            break;
    	  case ESP_BLUFI_EVENT_RECV_CA_CERT:
            /* Not handle currently */
            break;
    	  case ESP_BLUFI_EVENT_RECV_CLIENT_CERT:
            /* Not handle currently */
            break;
    	  case ESP_BLUFI_EVENT_RECV_SERVER_CERT:
            /* Not handle currently */
            break;
    	  case ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY:
            /* Not handle currently */
            break;;
    	  case ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY:
            /* Not handle currently */
            break;
        default:
            break;
        }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
        default:
            break;
    }
}

void bt_manager_start(){
  esp_err_t ret;

  cb_ptr_arr = malloc(sizeof(sizeof(void(*)(void*))) *MESSAGE_CODE_COUNT);
	for(int i=0; i<MESSAGE_CODE_COUNT; i++){
		cb_ptr_arr[i] = NULL;
	}

  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  initialize_wifi();

  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret) {
      ESP_LOGE(TAG_BT_MG, "%s initialize bt controller failed: %s\n", __func__, esp_err_to_name(ret));
  }

  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if (ret) {
      ESP_LOGE(TAG_BT_MG, "%s enable bt controller failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }

  ret = esp_bluedroid_init();
  if (ret) {
      ESP_LOGE(TAG_BT_MG, "%s init bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }

  ret = esp_bluedroid_enable();
  if (ret) {
      ESP_LOGE(TAG_BT_MG, "%s init bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
      return;
  }

  ESP_LOGI(TAG_BT_MG, "BLUFI/ADDR: "ESP_BD_ADDR_STR"", ESP_BD_ADDR_HEX(esp_bt_dev_get_address()));
  ESP_LOGI(TAG_BT_MG, "BLUFI/VERSION %04x", esp_blufi_get_version());

  ret = esp_ble_gap_register_callback(gap_event_handler);
  if(ret){
      ESP_LOGE(TAG_BT_MG, "%s gap register failed, error code = %x", __func__, ret);
      return;
  }

  ret = esp_blufi_register_callbacks(&bt_callbacks);
  if(ret){
      ESP_LOGE(TAG_BT_MG, "%s blufi register failed, error code = %x", __func__, ret);
      return;
  }

  esp_blufi_profile_init();
}

void bt_manager_set_callback(message_code_t message_code, void (*func_ptr)(void*) ){
	if(cb_ptr_arr && message_code < MESSAGE_CODE_COUNT){
		cb_ptr_arr[message_code] = func_ptr;
	}
}
