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
