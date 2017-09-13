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
void app_giu_port_local_init(int id, int lcl_id, int giu_id, struct lcl_giu_port_desc *lcl_port, struct giu_gpio *gpio,
			     u16 num_outqs, u32 outq_size);

/*
 * Build GIU BPool
 *
 * @param[in]	bpool_id	BPool ID.
 * @param[in]	bpool_inf	A pointer to a structure which contains the pool and buffer information.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int app_giu_build_bpool(int bpool_id, struct bpool_inf *infs);

#endif /*__MV_GIU_UTILS_H__*/

