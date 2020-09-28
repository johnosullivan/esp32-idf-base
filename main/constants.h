#ifndef CONSTANTS_H_INCLUDED
#define CONSTANTS_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

// Lightweight cJSON parser
#include "cJSON.h"

#define BLINK_GPIO			CONFIG_BLINK_GPIO
#define MIHOME_API_URI 		CONFIG_MIHOME_API_URI
#define WIFI_CORE_LOGGING 	CONFIG_WIFI_CORE_LOGGING
#define CONNECT_IOT_OPTION 	CONFIG_CONNECT_IOT_OPTION

/**
 * @brief Defines the complete list of all messages that the wifi_manager can process.
 *
 * Some of these message are events ("EVENT"), and some of them are action ("ORDER")
 * Each of these messages can trigger a callback function and each callback function is stored
 * in a function pointer array for convenience. Because of this behavior, it is extremely important
 * to maintain a strict sequence and the top level special element 'MESSAGE_CODE_COUNT'
 *
 * @see wifi_manager_set_callback
 */
typedef enum message_code_t {
	NONE = 0,
	WM_START_HTTP_SERVER = 1,
	WM_STOP_HTTP_SERVER = 2,
	WM_START_DNS_SERVICE = 3,
	WM_STOP_DNS_SERVICE = 4,
	WM_START_WIFI_SCAN = 5,
	WM_LOAD_AND_RESTORE_STA = 6,
	WM_CONNECT_STA = 7,
	WM_DISCONNECT_STA = 8,
	WM_START_AP = 9,
	WM_START_HTTP = 10,
	WM_START_DNS_HIJACK = 11,
	EVENT_STA_DISCONNECTED = 12,
	EVENT_SCAN_DONE = 13,
	EVENT_STA_GOT_IP = 14,
	MESSAGE_CODE_COUNT = 15 /* important for the callback array */

} message_code_t;

#define MIHOME_SETTINGS_CLOUD_URL_MAX_SIZE 64
#define MIHOME_SETTINGS_CLOUD_UUID_MAX_SIZE 36

struct mihome_settings_t {
	uint8_t cloud_url[MIHOME_SETTINGS_CLOUD_URL_MAX_SIZE];
	uint8_t cloud_uuid[MIHOME_SETTINGS_CLOUD_UUID_MAX_SIZE];
};
extern struct mihome_settings_t mihome_settings;

// Certs PEMS
extern const char api_cert_pem_start[] asm("_binary_api_cert_pem_start");
extern const char api_cert_pem_end[]   asm("_binary_api_cert_pem_end");

#define BT_DEVICE_NAME   "MIHOME_SWQZ192"

#define MIHOME_VERSION  			"v0.0.1"
#define MIHOME_STORAGE_NAMESPACE 	"storage"
#define MIHOME_STORAGE_IS_CONFIG 	"is_configured"
#define MIHOME_STORAGE_SETTINGS 	"mihome_settings"

#define MIHOME_LED_PIN 		4
#define HEX_COLOR_BLUE 		"0000ff"
#define HEX_COLOR_YELLOW 	"e3ff00"
#define HEX_COLOR_GREEN 	"00ff00"
#define HEX_COLOR_BLACK 	"000000"

#define PING_COMMAND "PING"

#ifdef __cplusplus
}
#endif

#endif /* CONSTANTS_H_INCLUDED */
