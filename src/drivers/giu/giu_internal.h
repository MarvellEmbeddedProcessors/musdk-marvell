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

#ifndef __GIU_INTERNAL_H__
#define __GIU_INTERNAL_H__

#include "std_internal.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_giu.h"
#include "drivers/mv_giu_gpio.h"
#include "include/gie.h"

#define GIU_LCL_Q (0)
#define GIU_REM_Q (1)

/* Queue handling macros. assumes q size is a power of 2 */
#define QUEUE_INDEX_INC(index_val, inc_val, q_size)	\
	(((index_val) + (inc_val)) & ((q_size) - 1))

#define QUEUE_OCCUPANCY(prod, cons, q_size)	\
	(((prod) - (cons) + (q_size)) & ((q_size) - 1))

#define QUEUE_SPACE(prod, cons, q_size)	\
	((q_size) - QUEUE_OCCUPANCY((prod), (cons), (q_size)) - 1)

#define QUEUE_FULL(prod, cons, q_size)	\
	((((prod) + 1) & ((q_size) - 1)) == (cons))

/****************************************************************************
 *	gpio queue structures
 ****************************************************************************/
/**
 * queue type
 *
 */
enum queue_type {
	MNG_CMD_QUEUE,
	MNG_NOTIFY_QUEUE,
	LOCAL_INGRESS_DATA_QUEUE,
	LOCAL_EGRESS_DATA_QUEUE,
	LOCAL_BM_QUEUE,
	HOST_INGRESS_DATA_QUEUE,
	HOST_EGRESS_DATA_QUEUE,
	HOST_BM_QUEUE

};

/**
 * gpio queue parameters
 *
 */
struct giu_gpio_queue {
	u32			 desc_total; /**< number of descriptors in the ring */
	struct giu_gpio_desc	*desc_ring_base; /**< descriptor ring virtual address */
	u32			 last_cons_val; /**< last consumer index value */

	u32			*prod_addr; /**< producer index virtual address */
	u32			*cons_addr; /**< consumer index virtual address */

	union {
		u32		 buff_len; /**< Buffer length (relevant for BPool only) */
		u32		 payload_offset; /**< Offset of the PL in the buffer (relevant for Data Qs only) */
	};
};

struct gie_type {
	u8 num_dma_engines;
	struct gie *gies[GIU_MAX_ENG_PER_TYPE];
	int duplicate[GIU_MAX_ENG_PER_TYPE];
};

struct giu {
	enum giu_indices_copy_mode	 indices_mode;

	struct mqa			*mqa;

	struct gie_type			 gie_types[GIU_ENG_OUT_OF_RANGE];
	struct giu_gpio			*gpio_list[GIU_MAX_NUM_GPIO];
	u64				 msi_regs_pa;	/**< MSI phys-address registers base */
	u64				 msi_regs_va;	/**< MSI virt-address registers base */
};

struct giu_mng_ch {
	struct giu	*giu;

	u32		 lcl_cmd_q_idx;
	struct mqa_q	*lcl_cmd_q;
	u32		 lcl_resp_q_idx;
	struct mqa_q	*lcl_resp_q;
	u32		 rem_cmd_q_idx;
	struct mqa_q	*rem_cmd_q;
	u32		 rem_resp_q_idx;
	struct mqa_q	*rem_resp_q;
};

struct gie *giu_get_gie_handle(struct giu *giu, enum giu_eng eng, u8 gie_index);

int giu_bpool_get_mqa_q_id(struct giu_bpool *bpool);

int giu_gpio_pre_gie(struct giu_gpio *gpio);

int giu_gpio_post_gie(struct giu_gpio *gpio);

int giu_gpio_register(struct giu *giu, struct giu_gpio *gpio, int gpio_idx);

int giu_gpio_unregister(struct giu *giu, struct giu_gpio *gpio, int gpio_idx);
int giu_gpio_dump(struct giu_gpio *gpio);

#endif /* __GIU_INTERNAL_H__ */
