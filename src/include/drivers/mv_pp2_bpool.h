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

#ifndef __MV_PP2_BPOOL_H__
#define __MV_PP2_BPOOL_H__

#include "mv_std.h"
#include "mv_pp2_hif.h"

/** @addtogroup grp_pp2_bp Packet Processor: Buffer Pool
 *
 *  Packet Processor Buffer Pool API documentation
 *
 *  @{
 */
/* Absolute number of hardware BM Pools */
#define PP2_BPOOL_NUM_POOLS		(16)
#define PP2_MAX_NUM_PUT_BUFFS		(8192)

struct pp2_bpool {
	int	pp2_id;		/* PP2 Instance */
	int	id;		/* BM Pool number */
	void	*internal_param;/* parameters for internal use */
};

extern struct pp2_bpool pp2_bpools[][PP2_BPOOL_NUM_POOLS];


/**
 * bpool init parameters
 *
 */
struct pp2_bpool_params {
	/** Used for DTS acc to find appropriate "physical" BM-Pool obj;
	 * E.g. "pool-0:0" means PPv2[0],pool[0]
	 */
	const char			*match;
	u32				buff_len; /**< buffer length */
	int				dummy_short_pool; /**< If application only needs one pool with jumbo
							* (or larger than 4KB) buffer size, then additional
							* pool must be created with this flag set to '1';
							* in this case setting the 'buff_len' can be skipped
							*/
	struct mv_sys_dma_mem_region	*likely_buffer_mem; /**< Memory likely to be used for putting buffers */
/* TODO: will not be supported at first stage. Need to look how to handle HW IRQ
 *	int				 (*empty_cb) (void *arg, u32 status);
 *	void				 *emty_cb_arg;
 *	u16				 threashold_hi;
 *	u16				 threashold_lo;
 */
};

/**
 * Initialize a Buffer Pool (bpool)
 *
 * @param[in]	params	A pointer to structure that contains all relevant parameters.
 * @param[out]	bpool	A pointer to opaque bpool handle of type 'struct pp2_bpool *'.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int pp2_bpool_init(struct pp2_bpool_params *params, struct pp2_bpool **bpool);

/**
 * Destroy a Buffer Pool (bpool)
 *
 * @param[in]	pool	A bpool handle.
 *
 */
void pp2_bpool_deinit(struct pp2_bpool *pool);

struct pp2_buff_inf {
	dma_addr_t	addr;
	u64		cookie;
};

struct buff_release_entry {
	struct pp2_buff_inf	buff;
	struct pp2_bpool	*bpool;
};

/**
 * Get a buffer from a ppv2 buffer pool.
 *
 * @param[in]	hif	A hif handle.
 * @param[in]	pool	A bpool handle.
 * @param[out]	buff	A pointer to structure that contains the returned buffer parameters.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int pp2_bpool_get_buff(struct pp2_hif *hif, struct pp2_bpool *pool, struct pp2_buff_inf *buff);


/* TO BE DELETED in subsequent patch - Together with all dependent applications */
int pp2_bpool_put_buff(struct pp2_hif *hif, struct pp2_bpool *pool, struct pp2_buff_inf *buff);
/**
 * Add a buffer to a ppv2 buffer pool.
 *
 * @param[in]	hif		A hif handle.
 * @param[in]	buff_entry	A pointer to array of buffers to release.
 * @param[in]	num		Number of buffers to release.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int pp2_bpool_put_buffs(struct pp2_hif *hif, struct buff_release_entry buff_entry[], u16 *num);

/****************************************************************************
 *	Run-time Control API
 ****************************************************************************/

/**
 * pp2 bpool capabilities
 *
 */
struct pp2_bpool_capabilities {
	u32	max_num_buffs;
	u32	buff_len;
};

/**
 * Get bpool capabilities
 *
.* This API should be called by the user application in order to retrieve the information and capabilities needed
 * for a probed bPool object.
 *
 * @param[in]	pool		bpool probed structure
 * @param[out]	capa		bpool information and capabilities
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int pp2_bpool_get_capabilities(struct pp2_bpool *pool, struct pp2_bpool_capabilities *capa);

/**
 * Get the number of buffers in ppv2 buffer pool.
 *
 * @param[in]	pool		A bpool handle.
 * @param[out]	num_buffs	A pointer to returned number of buffers.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int pp2_bpool_get_num_buffs(struct pp2_bpool *pool, u32 *num_buffs);

/**
 * Serialize the bpool parameters
 *
 * The serialization API is called by the 'master' user application to serialize a buffer-pool object.
 * The output string is created in a JSON format.
 * Below is how a bpool config-string looks like:
 *	pool-<pp2-id>:<id> : {
 *	 iomap_filename: <str>,	(TBD)
 *	 pp2_id : <int>,
 *	 id : <int>,
 *	 buff_len : <int>,
 *	 max_num_buffs : <int>,
 *	 base_addr : <dma-addr>	(TBD)
 * }
 *
 * The guest application can then access the created buffer pool object, and retrieve the bpool config string
 *
 * @param[in]	pool		A bpool handle.
 * @param[in]	buff		Pointer to a buffer where the resulting string is stored.
 *				The buffer should have a size of at least 'size' characters.
 * @param[in]	size		size of buffer.
 *
 * @retval	The number of characters that would have been written if 'size' had been sufficiently large
 * @retval	<0 on failure
 */
int pp2_bpool_serialize(struct pp2_bpool *pool, char buff[], u32 size);

/**
 * Probe a bpool
 *
 * The probe API should be called by the user application to create the buffer-pool object for a guest application.
 *
 * @param[in]	match		The matching string to search for in the Buffer pool object.
 * @param[in]	buff		Buffer pool object.
 * @param[out]	pool		bpool structure containing the results of the match
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int pp2_bpool_probe(char *match, char *buff, struct pp2_bpool **pool);

/**
 * Remove a bpool
 *
 * @param[in]	pool		bpool structure to remove
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int pp2_bpool_remove(struct pp2_bpool *pool);

/** @} */ /* end of grp_pp2_bp */

#endif /* __MV_PP2_BPOOL_H__ */
