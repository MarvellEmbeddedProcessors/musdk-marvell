/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_GIU_UTILS_H__
#define __MV_GIU_UTILS_H__

#include "mv_std.h"
#include "drivers/mv_giu_gpio.h"
#include "drivers/mv_giu_bpool.h"

#include "pp2_utils.h"

/* Maximum number of ports used by application */
#define MVAPPS_GIU_MAX_NUM_PORTS		2

extern u8 mvapp_giu_max_num_qs_per_tc;

struct giu_bpools_desc {
	u8			 num_bpools;
	struct giu_bpool	*bpools[GIU_GPIO_TC_MAX_NUM_BPOOLS];
};

/*
 * Local thread port parameters
 */
struct giu_port_desc {
	int			 initialized;	/* Flag indicated is port was initialized */
	int			 id;		/* Local port ID*/
	int			 giu_id;	/* Packet Processor ID */
	int			 gpio_id;	/* GPIO port ID */
	u32			 first_inq;	/* First RXQ - relative to the Port's first RXQ */
	u16			 num_tcs;	/* Number of TCs */
	u16			 num_inqs[GIU_GPIO_MAX_NUM_TCS];	/* Number of Rx queues per TC*/
	u16			 num_outqs[GIU_GPIO_MAX_NUM_TCS];	/* Number of Tx queues */
	u32			 inq_size;	/* Rx queue size */
	u32			 outq_size;	/* Tx queue size */
	struct giu_gpio		*gpio;		/* GPIO object returned by giu_gpio_probe() */
};

struct giu_buff_ent {
	struct giu_buff_inf	 buff_inf;	/* pointer to the buffer object */
	struct giu_bpool	*bpool;		/* pointer to the bpool object */
};

/*
 * Tx shadow queue
 */
struct giu_tx_shadow_q {
	u16			read_ind;	/* read index */
	u16			write_ind;	/* write index */

	struct giu_buff_ent	*ents;
};

/*
 * Local thread port parameters
 */
struct lcl_giu_port_desc {
	int			 id;		/* Local port ID*/
	int			 lcl_id;	/* Local thread ID*/
	int			 giu_id;	/* Packet Processor ID */
	int			 gpio_id;	/* GPIO port ID */
	struct giu_gpio		*gpio;		/* GPIO object returned by giu_gpio_probe() */
	int			 num_shadow_qs;	/* Number of Tx shadow queues */
	int			 shadow_q_size;	/* Size of Tx shadow queue */
	struct giu_tx_shadow_q	*shadow_qs;	/* Tx shadow queue */
};

/*
 * Init local GIU port object per thread according to port parameters
 *
 * @param[in]	id		Port ID.
 * @param[in]	lcl_id		Thread ID.
 * @param[in]	giu_id		GIU ID.
 * @param[in]	lcl_port	A pointer to GIU port descriptor structure.
 * @param[in]	num_outqs	Number of egress queues.
 * @param[in]	outq_size	Out queue size.
 *
 */
void app_giu_port_local_init(int id,
			int lcl_id,
			int giu_id,
			struct lcl_giu_port_desc *lcl_port,
			struct giu_gpio *gpio);

/*
 * Build GIU BPool
 *
 * @param[in]	bpool_id	BPool ID.
 * @param[in]	num_of_buffs	number of buffers to allocate for this BPool.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int app_giu_build_bpool(struct giu_bpool *bpool, u32 num_of_buffs);

static inline void app_giu_set_gen_max_num_qs_per_tc(void)
{
	mvapp_giu_max_num_qs_per_tc = system_ncpus();
}

#endif /*__MV_GIU_UTILS_H__*/
