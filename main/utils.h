#ifndef UTILS_H_INCLUDED
#define UTILS_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "constants.h"

void wifi_scanner();

void set_logging_levels();

esp_err_t nvs_get_str_value(char *namespace, char *key, char *ref);

esp_err_t nvs_set_str_value(char *namespace, char *key, char *ref);

void init_status_led(uint8_t pin);

void update_status_led(char *color_hex);

void ping_ws2812_signal();

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H_INCLUDED */
