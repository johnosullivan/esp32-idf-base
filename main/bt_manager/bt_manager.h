#ifndef BT_MANAGER_H_INCLUDED
#define BT_MANAGER_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "../constants.h"
#include "../utils.h"

// bluetooth 
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

typedef struct {
    uint8_t                 *prepare_buf;
    int                      prepare_len;
} prepare_type_env_t;

// bluetooth GATTS defines
#define GATTS_SERVICE_UUID   		0x0001
#define GATTS_CHAR_UUID      		0x1000
#define GATTS_NUM_HANDLE     		4
#define GATTS_CHAR_VAL_LEN_MAX 		0x40
#define PREPARE_BUF_MAX_SIZE 		1024
#define PROFILE_NUM 				1
#define PROFILE_APP_ID 				0

#define adv_config_flag      		(1 << 0)
#define scan_rsp_config_flag 		(1 << 1)

prepare_type_env_t prepare_write_env;
 
void bt_manager_start();

void parse_bt_json_playload(char *data);

void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

void write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

#ifdef __cplusplus
}
#endif

#endif /* BT_MANAGER_H_INCLUDED */
