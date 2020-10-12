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

#ifndef _GIE_H
#define _GIE_H

#include "mv_std.h"
#include "env/mv_sys_event.h"

struct gie;

struct gie_params {
	u64 gct_base;
	u64 gpt_base;

	/* MSI phys/virt register base */
	u64 msi_regs_phys;
	u64 msi_regs_virt;

	char *name_match;
	char *dmax_match;
};

enum gie_desc_type {
	TX_DESC = 0,
	RX_DESC,
	BUFF_DESC
};

enum gie_copy_mode_type {
	GIE_MODE_VIRT = 0,
	GIE_MODE_DMA,

	GIE_MODE_MAX
};

struct gie_event_params {
	u32 pkt_coal;
	u32 usec_coal;
	u32 tc_mask;
};


/**
 * Initialize the emulator.
 *
 * @param[in]	gie_reg		A pointer to GIE registers base.
 * @param[in]	dma_id		DMA ID.
 * @param[in]	name		A pointer to the emulator name.
 *
 * @retval	handler to the emulator.
 *
 */
int gie_init(struct gie_params *gie_pars, struct gie **gie);

/**
 * Terminate the emulator.
 *
 * @param[in]	gie		A GIE handler.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_terminate(struct gie *gie);

/**
 * Return the GIE registers.
 *
 * @param[in]	gie		A GIE handler.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
void *gie_regs(void *gie);

/**
 * Add Queue to GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	qid		queue ID.
 * @param[in]	is_remote	is this queue remote or local.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_add_queue(void *gie, u16 qid, int is_remote);

/**
 * Add Buffer Management Queue to GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	qid		queue ID.
 * @param[in]	buff_size	buffer size.
 * @param[in]	is_remote	is this queue remote or local.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_add_bm_queue(void *gie, u16 qid, int buf_size, int is_remote, void **bm_queue_ref);

/**
 * Remove Queue from GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	qid		queue ID.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_remove_queue(void *gie, u16 qid);

/**
 * Remove Buffer Management Queue from GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	qid		queue ID.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_remove_bm_queue(void *gie, u16 qid);

/**
 * Suspend Queue from GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	qid		queue ID.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_suspend_queue(void *gie, u16 qid);

/**
 * Resume Queue to GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	qid		queue ID.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_resume_queue(void *gie, u16 qid);

/**
 * Start GIE scheduling.
 *
 * @param[in]	gie		A GIE handler.
 * @param[in]	time_limit	schedule time lime (0 == infinite).
 * @param[in]	qe_limit	queue elements limit for processing.
 * @param[out]	pending		pending jobs
 *
 * @retval	0 on success
 * @retval	<0 on failure
 *
 */
int gie_schedule(void *gie, u64 time_limit, u64 qe_limit, u16 *pending);

/**
 * Return the GIE descriptor size.
 *
 * @param[in]	type		type of GIE queue/descriptor.
 *
 * @retval	The size fo the descriptor
 *
 */
int gie_get_desc_size(enum gie_desc_type type);

/**
 * Create a GIE event
 *
 * The event API is called to create a sys_event for a GIE, that
 * can later be polled through the mv_sys_event_poll() API.
 * This is only releavnt to 'NMP_SCHED_TX'
 *
 * @param[in]	gie		A pointer to a GIE object.
 * @param[in]	params		Parameters for the event.
 * @param[out]	ev		A pointer to event handle of type 'struct mv_sys_event *'.
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int gie_create_event(struct gie *gie, struct gie_event_params *params, struct mv_sys_event **ev);

/**
 * Delete a GIE event
 *
 * @param[in]	ev		A sys_event handle.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int gie_delete_event(struct mv_sys_event *ev);

/**
 * Set a GIE event
 *
 * The set_event API is called to enable the creation of events for the related GIE.
 *
 * @param[in]	ev		A sys_event handle.
 * @param[in]	en		enable/disable
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int gie_set_event(struct mv_sys_event *ev, int en);

/**
 * Get statistics
 *
 * The gie_get_stats API is called to get the statistics packets
 * count of a specific gie queue
 *
 * @param[in]	gie		A pointer to a GIE object.
 * @param[in]	qid		queue ID.
 * @param[out]	pkt_cnt	stats result.
 * @param[in]	reset	stats reset flag.
 *
 * @retval      0 on success
 * @retval      <0 on failure
 */
int gie_get_queue_stats(void *gie, u16 qid, u64 *pkt_cnt, int reset);

int gie_add_bm_queue_reference(void *giu, void *bm_queue_ref);
int gie_remove_bm_queue_refernce(void *giu, void *bm_queue_ref);

#endif /* _GIE_H */
