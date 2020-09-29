/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef __MV_GIU_BPOOL_H__
#define __MV_GIU_BPOOL_H__

#include "mv_std.h"
#include "mv_mqa.h"
#include "mv_giu.h"

/** @addtogroup grp_giu_bp GIU Port I/O (GP-IO): Buffer Pool
 *
 *  GP-IO Buffer Pool API documentation
 *
 *  @{
 */

#define GIU_BPOOL_NUM_POOLS		(64)

struct giu_bpool {
	u8	 giu_id;	/**< GIU Instance */
	u8	 id;		/**< BM Pool number */

	void	*internal_param;/** parameters for internal use */
};

/* Support only a single GIU */
extern struct giu_bpool giu_bpools[GIU_BPOOL_NUM_POOLS];
extern struct giu_bpool giu_bpools_guest[GIU_BPOOL_NUM_POOLS];


struct giu_bpool_params {
	/** Used for DTS acc to find appropriate "physical" BM-Pool obj;
	 * E.g. "giu_pool-0:0" means GIU[0],pool[0]
	 */
	const char	*match;

	struct mqa	*mqa;
	struct giu	*giu;

	u16		 num_buffs; /**< number of buffers */
	u16		 buff_len; /**< buffer length */
};

/**
 * Initialize a Buffer Pool (bpool)
 *
 * @param[in]	init_params	bpool initialization parameters
 * @param[out]	bpool		A pointer to opaque bpool handle of type 'struct giu_bpool *'.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_bpool_init(struct giu_bpool_params *params, struct giu_bpool **bpool);

/**
 * De-initialize a Buffer Pool (bpool)
 *
 * @param[in]	bpool	A bpool handle.
 *
 */
void giu_bpool_deinit(struct giu_bpool *bpool);

/**
 * Serialize the bpool parameters
 *
 * The serialization API is called by the 'master' user application to serialize a buffer-pool object.
 * The output string is created in a JSON format.
 * Below is how a bpool config-string looks like:
 *	giu-pool-<giu-id>:<id> : {
 *	 "dma_dev_name": <str>,
 *	 giu_id : <int>,
 *	 id : <int>,
 *	 buff_len : <int>,
 *	 max_num_buffs : <int>,
 *	phy_base_offset: <hex>,
 *	prod_offset: <hex>,
 *	cons_offset: <hex>
 * }
 *
 * The guest application can then access the created buffer pool object, and retrieve the bpool config string
 *
 * @param[in]	bpool		bpool to serialize
 * @param[in]	buf		buffer
 * @param[in]	size		size of the buffer
 * @param[in]	depth		size of the buffer
 *
 * @retval	>0 (length of data written to buffer) on success
 * @retval	<0 on failure
 */
int giu_bpool_serialize(struct giu_bpool *bpool, char *buff, u32 size, u8 depth);

/**
 * Probe a bpool
 *
 * The probe API should be called by the user application to create the buffer-pool object for a guest application.
 *
 * @param[in]	match		The matching string to search for in the Buffer pool object.
 * @param[in]	buff		Buffer pool object.
 * @param[out]	bpool		bpool structure containing the results of the match
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int giu_bpool_probe(char *match, char *buff, struct giu_bpool **bpool);

/**
 * Remove a Buffer Pool (bpool)
 *
 * @param[in]	pool	A bpool handle.
 *
 */
void giu_bpool_remove(struct giu_bpool *pool);


/**
 * buffer parameters
 * TODO: change this to host_bpool_desc structure
 */
struct giu_buff_inf {
	dma_addr_t	addr; /**< physical address of the buffer */
	u64		cookie; /**< cookie of the buffer */
};

/**
 * Add a buffer to a giu buffer pool.
 *
 * @param[in]	pool	A bpool handle.
 * @param[out]	buff	A pointer to structure that contains the returned buffer parameters.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_bpool_put_buff(struct giu_bpool *pool, struct giu_buff_inf *buff);


/**
 * Add bulk of buffers to a giu buffer pool.
 *
 * @param[in]	buff_entry	A pointer to array of buffers to release.
 * @param[in]	num		Number of buffers to release.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_bpool_put_buffs(struct giu_bpool *pool, struct giu_buff_inf buff_entry[], u16 *num);


/**
 * Get the number of buffers in giu buffer pool.
 *
 * @param[in]	pool		A bpool handle.
 * @param[out]	num_buffs	A pointer to returned number of buffers.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_bpool_get_num_buffs(struct giu_bpool *pool, u32 *num_buffs);

/**
 * Reset the pool to empty state
 *
 * @param[in]	pool		A bpool handle.
 *
 */
void giu_bpool_reset(struct giu_bpool *bpool);

struct giu_bpool_capabilities {
	u32 buff_len; /**< buffer length */
	u32 max_num_buffs; /**< max number of buffers in the bpool */
};

/**
 * Get GIU BPool capabilities.
 *
 * @param[in]	bpool	a bpool handler.
 * @param[out]	capa	capability structure.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int giu_bpool_get_capabilities(struct giu_bpool *bpool, struct giu_bpool_capabilities *capa);

/**
 * Allocate GIU BPool.
 *
 * @retval	BPool-ID on success
 * @retval	error-code otherwise
 */
int giu_bpool_alloc(void);

/** @} */ /* end of grp_giu_bp */

#endif /* __MV_GIU_BPOOL_H__ */
