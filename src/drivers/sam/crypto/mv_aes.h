/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_AES_H__
#define __MV_AES_H__

#include <stdint.h>


void mv_aes_ecb_encrypt(uint8_t *input, const uint8_t *key, uint8_t *output, int key_size);
void mv_aes_cbc_encrypt(uint8_t *input, const uint8_t *key,
			    uint8_t *k1, uint8_t *k2, int key_size);

#endif /* __MV_AES_H__ */
