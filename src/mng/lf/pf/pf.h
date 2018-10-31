/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _PF_H
#define _PF_H

int nmnicpf_init(struct nmnicpf *nmnicpf);
int nmnicpf_deinit(struct nmnicpf *nmnicpf);

/**
 * Serialize the NIC-PF parameters
 *
 * @param[in]	nmnicpf		A NIC-PF handle.
 * @param[in]	buff		Pointer to a buffer where the resulting string is stored.
 *				The buffer should have a size of at least 'size' characters.
 * @param[in]	size		size of buffer.
 *
 * @retval	The number of characters that would have been written if 'size' had been sufficiently large
 * @retval	<0 on failure
 */
int nmnicpf_serialize(struct nmnicpf *nmnicpf, char *buff, u32 size);

#endif /* _PF_H */
