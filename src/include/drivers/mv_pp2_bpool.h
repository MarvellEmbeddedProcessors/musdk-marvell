/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
