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

#ifndef __GIU_QUEUE_TOPOLOGY_H__
#define __GIU_QUEUE_TOPOLOGY_H__


#define GIU_MAX_NUM_GPIO		3 /**< Maximum number of gpio instances */
#define GIU_MAX_NUM_TC			3

/** @addtogroup grp_giu_queue_toppology Queue Topology
 *
 *  GIU Queue Topology structures documentation
 *
 *  @{
 */

/* Queue handling macros. assumes q size is a power of 2 */
#define QUEUE_INDEX_INC(index_val, inc_val, q_size)	\
	(((index_val) + (inc_val)) & ((q_size) - 1))

#define QUEUE_OCCUPANCY(prod, cons, q_size)	\
	(((prod) - (cons) + (q_size)) & ((q_size) - 1))

#define QUEUE_SPACE(prod, cons, q_size)	\
	((q_size) - QUEUE_OCCUPANCY((prod), (cons), (q_size)) - 1)

#define QUEUE_FULL(prod, cons, q_size)	\
	((((prod) + 1) & ((q_size) - 1)) == (cons))

enum giu_gpio_outqs_sched_mode {
	GIU_GPIO_SCHED_M_NONE = 0,
};

/****************************************************************************
 *	gpio queue structures
 ****************************************************************************/

/**
 * gpio queue parameters
 *
 */
struct giu_gpio_queue {
	u32		desc_total; /**< number of descriptors in the ring */
	void		*desc_ring_base_phys;    /**< descriptor ring physical address */
	struct giu_gpio_desc	*desc_ring_base; /**< descriptor ring virtual address */
	union {
		u32	prod_val_shadow; /**< producer index value shadow  */
		u32	cons_val_shadow; /**< consumer index value shadow */
	};
	u32		last_cons_val; /**< last consumer index value */

	void		*prod_addr_phys; /**< producer index phys address */
	void		*cons_addr_phys; /**< consumer index phys address */
	u32		*prod_addr; /**< producer index virtual address */
	u32		*cons_addr; /**< consumer index virtual address */

	union {
		u32	buff_len; /**< Buffer length (relevant for BPool only) */
		u32	payload_offset; /**< Offset of the PL in the buffer (relevant for Data Qs only) */
	};
};

/**
 * gpio tc parameters
 */
struct giu_gpio_tc {
	/** Indicate the type of hash to use in hashing mechanism according to giu_gpio_hash_type.
	 * The hash type is common to all TC's in gpio
	 */
	enum giu_gpio_hash_type {
		GIU_GPIO_HASH_T_NONE = 0,	/* Invalid hash type (hashing mechanism is disabled) */
		GIU_GPIO_HASH_T_2_TUPLE,	/* IP-src, IP-dst */
		GIU_GPIO_HASH_T_5_TUPLE,	/* IP-src, IP-dst, IP-Prot, L4-src, L4-dst */
		GIU_GPIO_HASH_T_OUT_OF_RANGE
	} hash_type;
	u8			num_qs; /**< number of q in TC */
	struct giu_gpio_queue	*queues; /**< gpio q parameters */
	/* Remote parameters */
	u8			dest_num_qs; /**< number of q in Remote TC */
};

/**
 * gpio queues related parameters
 *
 */
struct giu_gpio_qs_params {
	u8			num_tcs; /**< number of TCs */
	struct giu_gpio_tc	*tcs; /**< TCs parameters */
};

/**
 * gpio parameters
 *
 */
struct giu_gpio_params {
	struct giu_gpio_qs_params	inqs_params; /**<  gpio inq parameters */
	struct giu_gpio_qs_params	outqs_params; /**<  gpio outq parameters */
	struct giu_gpio_queue		bpool;	/**<  gpio bpool queue parameters */
};

extern struct giu_gpio_params giu_params[GIU_MAX_NUM_GPIO];

/**
 * Init the queue topology
 *
 * @param[in]	giu_id		GIU ID.
 * @param[in]	regfile_name	Register file name.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_gpio_init_topology(int giu_id, char *regfile_name);

/**
 * Deinit queue topology
 *
 * @param[in]	giu_id		GIU ID.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_gpio_deinit_topology(int giu_id);

/**
 * Get queue topology
 *
 * @param[in]	giu_id		GIU ID.
 *
 * @retval	pointer to giu_gpio_params
 */
static inline struct giu_gpio_params *giu_gpio_get_topology(int giu_id)
{
	return &giu_params[giu_id];
}

/**
 * Set init done indication to register file
 *
 * @param[in]	giu_id		GIU ID.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_gpio_topology_set_init_done(int giu_id);

/** @} */ /* end of grp_giu_queue_toppology */

#endif /* __GIU_QUEUE_TOPOLOGY_H__ */
