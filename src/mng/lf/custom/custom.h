/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _CUSTOM_H
#define _CUSTOM_H

#include "mv_std.h"
#include "drivers/mv_mqa.h"
#include "mng/mv_nmp_dispatch.h"

struct nmcstm;

struct nmcstm_params {
	u8 id;
	u8 pf_id;
	struct nmdisp *nmdisp;
	struct mqa *mqa;
};

int nmcstm_init(struct nmcstm_params *params, struct nmcstm **nmcstm);
int nmcstm_deinit(struct nmcstm *nmcstm);

/**
 * Serialize the nmcstm parameters
 *
 * @param[in]	nmcstm		A nmcstm handle.
 * @param[in]	buff		Pointer to a buffer where the resulting string is stored.
 *				The buffer should have a size of at least 'size' characters.
 * @param[in]	size		size of buffer.
 *
 * @retval	The number of characters that would have been written if 'size' had been sufficiently large
 * @retval	<0 on failure
 */
int nmcstm_serialize(struct nmcstm *nmcstm, char *buff, u32 size);

#endif /* _CUSTOM_H */

