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

#ifndef __MV_GIU_H__
#define __MV_GIU_H__

#include "mv_std.h"

/** @addtogroup grp_giu_init GIU: Initialization
 *
 *  Generic Interface Unit (GIU) Initialization API documentation
 *
 *  @{
 */

#define GIU_MAX_NUM_GPIO	8 /**< Maximum number of gpio instances */
#define GIU_MAX_ENG_PER_TYPE	4 /**< Maximum number of dma-engines per giu eng */

struct giu;
struct giu_mng_ch;


/**
 * GIU engine type enumaration
 */
enum giu_eng {
	GIU_ENG_MNG = 0,
	GIU_ENG_IN,
	GIU_ENG_OUT,

	GIU_ENG_OUT_OF_RANGE,
};

/**
 * GIU descriptor type enumaration
 */
enum giu_desc_type {
	GIU_DESC_IN = 0,
	GIU_DESC_OUT,
	GIU_DESC_BUFF
};

/**
 * GIU indices mode enumaration
 */
enum giu_indices_copy_mode {
	GIU_INDX_CPY_MODE_VIRT = 0,
	GIU_INDX_CPY_MODE_DMA,

	GIU_INDX_CPY_MODE_MAX
};

/**
 * GIU multi queues mode enumaration
 */
enum giu_multi_qs_mode {
	GIU_MULTI_QS_MODE_REAL = 0,
	GIU_MULTI_QS_MODE_VIRT
};

/**
 * GIU emulation parameters
 */
struct giu_emul_params {
	u8 num_dma_engines;	/**< Number of DMA engines for this type*/
	char *engine_name[GIU_MAX_ENG_PER_TYPE];
};

/**
 * GIU parameters
 */
struct giu_params {
	struct mqa		*mqa; /** a pointer to MQA handle */

	u64			 msi_regs_pa;	/**< MSI phys-address registers base */
	u64			 msi_regs_va;	/**< MSI virt-address registers base */

	u8			 num_gie_types;
	struct giu_emul_params	 gie_type_params[GIU_ENG_OUT_OF_RANGE];
};

/**
 * GIU management-channel local queue parameters
 */
struct giu_mng_ch_lcl_q_params {
	u32		len; /**< queue length */
};

/**
 * GIU management-channel remote queue parameters
 */
struct giu_mng_ch_rem_q_params {
	dma_addr_t	 pa; /**< Remote queue phys base address */
	dma_addr_t	 prod_pa; /**< queue producer physical-address */
	void		*prod_va; /**< queue producer virtual-address */
	dma_addr_t	 cons_pa; /**< queue consumer physical-address */
	void		*cons_va; /**< queue consumer virtual-address */
	u32		 len; /**< queue length */
};

/**
 * GIU management-channel parameters
 */
struct giu_mng_ch_params {
	dma_addr_t			 rem_base_pa; /**< remote queues base address phys address*/
	void				*rem_base_va; /**< remote queues base address virt address*/
	u16				 desc_size; /**< the management channel descriptors size */

	struct giu_mng_ch_lcl_q_params	 lcl_cmd_q;  /**< Local command-queue parameters */
	struct giu_mng_ch_lcl_q_params	 lcl_resp_q; /**< Local response-queue parameters */
	struct giu_mng_ch_rem_q_params	 rem_cmd_q;  /**< Remote command-queue parameters */
	struct giu_mng_ch_rem_q_params	 rem_resp_q; /**< Remote response-queue parameters */
};

/**
 * GIU management-channel MQA queues
 */
struct giu_mng_ch_qs {
	struct mqa_q			*lcl_cmd_q;  /**< Local command MQA queue handle */
	struct mqa_q			*lcl_resp_q; /**< Local response MQA queue handle */
	struct mqa_q			*rem_cmd_q;  /**< Remote command MQA queue handle */
	struct mqa_q			*rem_resp_q; /**< Remote response MQA queue handle */
};

/**
 * Initialize the global GIU
 *
 * @param[in]	params	A pointer to structure that contains all relevant parameters.
 * @param[out]	giu	A pointer to returned GIU handle.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_init(struct giu_params *params, struct giu **giu);

/**
 * Destroy the global GIU
 *
 * @param[in]	giu		A pointer to GIU handler.
 */
void giu_deinit(struct giu *giu);

/**
 * Create and initialize GIU Management channel
 *
 * @param[in]	giu		A pointer to GIU handler.
 * @param[in]	params		GIU emulator engine.
 * @param[out]	mng_ch		A pointer to the returned GIU MNG CH handle.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_mng_ch_init(struct giu *giu, struct giu_mng_ch_params *params, struct giu_mng_ch **mng_ch);

/**
 * Destroy GIU Management channel
 *
 * @param[out]	mng_ch		A pointer to GIU MNG CH handle.
 */
void giu_mng_ch_deinit(struct giu_mng_ch *mng_ch);

/**
 * get GIU Management channel MQA queues objects
 *
 * @param[out]	mng_ch		A pointer to GIU MNG CH handle.
 * @param[out]	qs		A pointer to the returned GIU MNG CH queues struct to be filled.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_mng_ch_get_qs(struct giu_mng_ch *mng_ch, struct giu_mng_ch_qs *qs);

/**
 * Start GIU emulation scheduling.
 *
 * @param[in]	giu		A pointer to GIU handler.
 * @param[in]	eng		GIU emulation engine.
 * @param[in]	time_limit	schedule time lime (0 == infinite).
 * @param[in]	qe_limit	queue elements limit for processing.
 * @param[out]	pending		pending jobs
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int giu_schedule(struct giu *giu, enum giu_eng eng, u64 time_limit, u64 qe_limit, u16 *pending);

/**
 * Return the GIU emualtion descriptor size.
 *
 * @param[in]	giu		A pointer to GIU handler.
 * @param[in]	type		type of GIU queue/descriptor.
 *
 * @retval	The size fo the descriptor
 */
int giu_get_desc_size(struct giu *giu, enum giu_desc_type type);

int giu_get_msi_regs(struct giu *giu, u64 *va, u64 *pa);

int giu_get_num_dma_engines(struct giu *giu, enum giu_eng eng, u8 *num_dma_engines);

int giu_dump(struct giu *giu);

/** @} */ /* end of grp_giu_init */

#endif /* __MV_GIU_H__ */
