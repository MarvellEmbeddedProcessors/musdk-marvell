/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _DEV_MNG_H
#define _DEV_MNG_H

#include "mv_std.h"
#include "mng/mv_nmp.h"

#define SER_FILE_VAR_DIR	"/tmp/"
#define SER_FILE_NAME_PREFIX	"musdk-serial-cfg"
#define SER_MAX_FILE_NAME	64
#define SER_MAX_FILE_SIZE	(30 * 1024)

int dev_mng_init(struct nmp *nmp, struct nmp_params *params);
int dev_mng_terminate(struct nmp *nmp);

int dev_mng_get_free_bar(struct nmp *nmp, void **va, dma_addr_t *pa);
void dev_mng_put_bar(struct nmp *nmp, int index);

#endif /* _DEV_MNG_H */
