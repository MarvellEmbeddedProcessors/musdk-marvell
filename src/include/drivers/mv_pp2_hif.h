/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_PP2_HIF_H__
#define __MV_PP2_HIF_H__

#include "mv_std.h"

/** @addtogroup grp_pp2_hif Packet Processor: Host Interface
 *
 *  Packet Processor Host Interface API documentation
 *
 *  @{
 */

struct pp2_hif;

/**
 * hif init parameters
 *
 */
struct pp2_hif_params {
	/** Used for DTS acc to find appropriate "physical" H-IF obj;
	 * E.g. "hif-0" means PPv2,HIF[0]
	 */
	const char	*match;
	u32		 out_size; /**< TX-Aggregation q_size */
	struct mv_sys_dma_mem_region *mem;
};

/**
 * Initialize a Host Interface (hif)
 *
 * @param[in]	params	A pointer to structure that contains all relevant parameters.
 * @param[out]	hif	A pointer to opaque hif handle of type 'struct pp2_hif *'.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int pp2_hif_init(struct pp2_hif_params *params, struct pp2_hif **hif);

/**
 * Destroy a Host Interface (hif)
 *
 * @param[in]	hif	A hif handle.
 *
 */
void pp2_hif_deinit(struct pp2_hif *hif);

/** @} */ /* end of grp_pp2_hif */

#endif /* __MV_PP2_HIF_H__ */
