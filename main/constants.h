#ifndef CONSTANTS_H_INCLUDED
#define CONSTANTS_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#define BLINK_GPIO	CONFIG_BLINK_GPIO
#define MIHOME_API_URI CONFIG_MIHOME_API_URI
#define WIFI_CORE_LOGGING CONFIG_WIFI_CORE_LOGGING
#define CONNECT_IOT_OPTION CONFIG_CONNECT_IOT_OPTION

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

#ifdef __cplusplus
}
#endif

#endif /* CONSTANTS_H_INCLUDED */
