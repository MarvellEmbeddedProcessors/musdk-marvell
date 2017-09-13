/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef __MV_GIU_BPOOL_H__
#define __MV_GIU_BPOOL_H__

#include "mv_std.h"

/** @addtogroup grp_giu_bp Packet Processor: Buffer Pool
 *
 *  Packet Processor Buffer Pool API documentation
 *
 *  @{
 */

#define GIU_BPOOL_NUM_POOLS		(16)


struct giu_bpool {
	u32	giu_id;	/**< GIU Instance */
	int	id; /**< BM Pool number */
	u32	buff_len; /**< buffer length */

	/* Buffer Pool Q parameters */
	void	*queue;
};

extern struct giu_bpool giu_bpools[GIU_BPOOL_NUM_POOLS];

/**
 * Probe the Buffer Pool (bpool)
 *
 * @param[in]	match		Match string.
 * @param[in]	regfile_name	register file location.
 * @param[out]	bpool		A pointer to opaque bpool handle of type 'struct giu_bpool *'.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_bpool_probe(char *match, char *regfile_name, struct giu_bpool **bpool);

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
	phys_addr_t	addr; /**< physical address of the buffer */
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

/** @} */ /* end of grp_giu_bp */

#endif /* __MV_GIU_BPOOL_H__ */
