/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
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
