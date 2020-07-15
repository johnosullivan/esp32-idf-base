#ifndef BLUFI_SECURITY_H_INCLUDED
#define BLUFI_SECURITY_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_blufi_api.h"

void blufi_dh_negotiate_data_handler(uint8_t *data, int len, uint8_t **output_data, int *output_len, bool *need_free);

int blufi_aes_encrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len);

int blufi_aes_decrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len);

uint16_t blufi_crc_checksum(uint8_t iv8, uint8_t *data, int len);

int blufi_security_init(void);

void blufi_security_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* BLUFI_SECURITY_H_INCLUDED */
