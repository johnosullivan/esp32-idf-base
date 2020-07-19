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

#ifdef __cplusplus
}
#endif

#endif /* UTILS_H_INCLUDED */
