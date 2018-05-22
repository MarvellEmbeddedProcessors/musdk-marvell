/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _DEV_MNG_PP2_H
#define _DEV_MNG_PP2_H

int dev_mng_pp2_init(struct nmp *nmp);
int dev_mng_pp2_terminate(struct nmp *nmp);
/**
 * Serialize the nmnicpf parameters
 *
 * @param[in]	nmnicpf		A nmnicpf handle.
 * @param[in]	buff		Pointer to a buffer where the resulting string is stored.
 *				The buffer should have a size of at least 'size' characters.
 * @param[in]	size		size of buffer.
 *
 * @retval	The number of characters that would have been written if 'size' had been sufficiently large
 * @retval	<0 on failure
 */
int dev_mng_pp2_serialize(struct nmnicpf *nmnicpf, char *buff, u32 size);

#endif /* _DEV_MNG_PP2_H */
