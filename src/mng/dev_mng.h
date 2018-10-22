/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _DEV_MNG_H
#define _DEV_MNG_H

#define SER_FILE_VAR_DIR	"/var/"
#define SER_FILE_NAME_PREFIX	"musdk-serial-cfg"
#define SER_MAX_FILE_NAME	64
#define SER_MAX_FILE_SIZE	(30 * 1024)

int dev_mng_init(struct nmp *nmp);
int dev_mng_terminate(struct nmp *nmp);

int dev_mng_get_free_bar_idx(struct nmp *nmp);
void dev_mng_put_bar_idx(struct nmp *nmp, int index);

#endif /* _DEV_MNG_H */
