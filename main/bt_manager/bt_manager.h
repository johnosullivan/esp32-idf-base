#ifndef BT_MANAGER_H_INCLUDED
#define BT_MANAGER_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "../constants.h"

// bluetooth
#include "esp_bt.h"
#include "esp_blufi_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

void bt_manager_start();

void bt_manager_set_callback(message_code_t message_code, void (*func_ptr)(void*));

void bt_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param);

#ifdef __cplusplus
}
#endif

#endif /* BT_MANAGER_H_INCLUDED */
