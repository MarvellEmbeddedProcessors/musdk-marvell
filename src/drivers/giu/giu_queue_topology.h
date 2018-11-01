/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef __GIU_QUEUE_TOPOLOGY_H__
#define __GIU_QUEUE_TOPOLOGY_H__

#include "std_internal.h"

#include "drivers/mv_giu_gpio.h"
#include "drivers/giu_regfile_def.h"

#define GIU_MAX_NUM_GPIO		3 /**< Maximum number of gpio instances */
#define GIU_MAX_NUM_TC			8

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

struct msix_table_entry {
	u64 msg_addr;
	u32 msg_data;
	u32 vector_ctrl;
};

/**
 * gpio queue parameters
 *
 */
struct giu_gpio_queue {
	u32		desc_total; /**< number of descriptors in the ring */
	struct giu_gpio_desc	*desc_ring_base; /**< descriptor ring virtual address */
	u32		last_cons_val; /**< last consumer index value */

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
struct giu_gpio_probe_params {
	struct giu_gpio_qs_params	inqs_params; /**<  gpio inq parameters */
	struct giu_gpio_qs_params	outqs_params; /**<  gpio outq parameters */
	struct giu_gpio_queue		bpool;	/**<  gpio bpool queue parameters */
};

extern struct giu_gpio_probe_params gpio_probe_params[GIU_MAX_NUM_GPIO];

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
 * @retval	pointer to giu_gpio_probe_params
 */
static inline struct giu_gpio_probe_params *giu_gpio_get_topology(int giu_id)
{
	return &gpio_probe_params[giu_id];
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

/**
 *	giu_regfile_register_queue
 *
 *	This function register Queue params in Regfile Queue structure
 *	It gets the info from the SNIC-DB and finally update directly the regfile
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
int giu_regfile_register_queue(union giu_gpio_q_params *giu_gpio_q_p,
			       enum giu_queue_type q_type,
			       u32 q_add_data,
			       phys_addr_t qs_phys_base,
			       phys_addr_t ptrs_phys_base,
			       void **file_map);

/** @} */ /* end of grp_giu_queue_toppology */

#endif /* __GIU_QUEUE_TOPOLOGY_H__ */
