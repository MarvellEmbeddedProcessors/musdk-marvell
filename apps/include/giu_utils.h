/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef __MV_GIU_UTILS_H__
#define __MV_GIU_UTILS_H__

#include "mv_giu_gpio.h"
#include "mv_giu_bpool.h"
#include "pp2_utils.h"


/* Maximum number of ports used by application */
#define MVAPPS_GIU_MAX_NUM_PORTS		2

#define REGFILE_VAR_DIR         "/var/"
#define REGFILE_NAME_PREFIX     "nic-pf-"
#define REGFILE_MAX_FILE_NAME   64


extern u8 mvapp_giu_max_num_qs_per_tc;


/*
 * Tx shadow queue
 */
struct giu_tx_shadow_q {
	u16			read_ind;	/* read index */
	u16			write_ind;	/* write index */

	struct giu_buff_inf	*buffs_inf;	/* pointer to the buffer object */
	struct giu_bpool	*bpool;		/* pointer to the bpool object */
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
 * Init GIU port
 *
 * @param[in]	giu_id		GIU ID.
 * @param[out]	giu_gpio	GPIO handler.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int app_giu_port_init(int giu_id, struct giu_gpio **giu_gpio);

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
int app_giu_build_bpool(int bpool_id, u32 num_of_buffs);


static inline void app_giu_set_gen_max_num_qs_per_tc(void)
{
	mvapp_giu_max_num_qs_per_tc = system_ncpus();
}

#endif /*__MV_GIU_UTILS_H__*/

