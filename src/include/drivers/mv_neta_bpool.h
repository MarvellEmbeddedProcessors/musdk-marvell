/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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

#ifndef __MV_NETA_BPOOL_H__
#define __MV_NETA_BPOOL_H__

#include "mv_std.h"

/** @addtogroup grp_neta_bp Packet Processor: Buffer Pool
 *
 *  Packet Processor Buffer Pool API documentation
 *
 *  @{
 */
/* Absolute number of hardware BM Pools */
#define NETA_BPOOL_NUM_POOLS		(4)
#define NETA_MAX_NUM_PUT_BUFFS		(4096)

struct neta_bpool {
	int	id;		/* BM Pool number */
	void	*internal_param;/* parameters for internal use */
};

extern struct neta_bpool neta_bpools[NETA_BPOOL_NUM_POOLS];


/**
 * bpool init parameters
 *
 */
struct neta_bpool_params {
	/** Used for DTS acc to find appropriate "physical" BM-Pool obj;
	 * E.g. "pool-0:0" means PPv2[0],pool[0]
	 */
	const char			*match;
	u32				 buff_len; /**< buffer length */
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
 * @param[out]	bpool	A pointer to opaque bpool handle of type 'struct neta_bpool *'.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int neta_bpool_init(struct neta_bpool_params *params, struct neta_bpool **bpool);

/**
 * Destroy a Buffer Pool (bpool)
 *
 * @param[in]	pool	A bpool handle.
 *
 */
void neta_bpool_deinit(struct neta_bpool *pool);


typedef u32	neta_dma_addr_t;

typedef u32	neta_cookie_t;

struct neta_buff_inf {
	neta_dma_addr_t addr;
	neta_cookie_t   cookie;
};

struct buff_release_entry {
	struct neta_buff_inf	buff;
	struct neta_bpool	*bpool;
};

/**
 * Get a buffer from buffers pool.
 *
 * @param[in]	pool	A bpool handle.
 * @param[out]	buff	A pointer to structure that contains the returned buffer parameters.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int neta_bpool_get_buff(struct neta_bpool *pool, struct neta_buff_inf *buff);

/**
 * Add number of buffers to buffers pool.
 *
 * @param[in]	buff_entry	A pointer to array of buffers to add.
 * @param[in]	num		Number of buffers to add.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int neta_bpool_put_buffs(struct buff_release_entry buff_entry[], int *num);

/**
 * Add one buffer to buffers pool.
 *
 * @param[in]	buff_entry	A pointer to array of buffers to add.
 * @param[in]	num		Number of buffers to add.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int neta_bpool_put_buff(struct neta_bpool *pool, struct neta_buff_inf *buff_entry);

/**
 * Get the number of buffers in buffers pool.
 *
 * @param[in]	pool		A bpool handle.
 * @param[out]	num_buffs	A pointer to returned number of buffers.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int neta_bpool_get_num_buffs(struct neta_bpool *pool, u32 *num_buffs);

/** @} */ /* end of grp_neta_bp */

#endif /* __MV_NETA_BPOOL_H__ */
