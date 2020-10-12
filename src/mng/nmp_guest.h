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

#ifndef _NMP_GUEST_H
#define _NMP_GUEST_H

#include "mng/mv_nmp_guest.h"
#include "mng/mv_nmp.h"
#include "include/guest_mng_cmd_desc.h"

#define NMP_MAX_BUF_STR_LEN	256

#define q_inc_idx(q, idx)	q_inc_idx_val(q, idx, 1)
#define q_inc_idx_val(q, idx, val)	((idx + val) & (q->len - 1))
#define q_rd_idx(idx)		(readl_relaxed((u32 *)idx))
#define q_wr_idx(idx, val)	(writel_relaxed(val, (u32 *)idx))
#define q_rd_cons(q)		q_rd_idx(q->cons_virt)
#define q_rd_prod(q)		q_rd_idx(q->prod_virt)
#define q_wr_cons(q, val)	q_wr_idx(q->cons_virt, val)
#define q_wr_prod(q, val)	q_wr_idx(q->prod_virt, val)

#define q_full(q, p, c)		(((p + 1) & (q->len - 1)) == c)
#define q_empty(p, c)		(p == c)
#define q_occupancy(q, p, c)	((p - c + q->len) & (q->len - 1))
#define q_space(q, p, c)	(q->len - q_occupancy(q, p, c) - 1)

struct nmp_guest_queue {
	u32		len; /**< number of descriptors in the ring */
	void		*base_addr_phys;    /**< descriptor ring physical address */
	struct cmd_desc	*base_addr_virt; /**< descriptor ring virtual address */
	void		*prod_phys; /**< producer index phys address */
	void		*cons_phys; /**< consumer index phys address */
	u32		*prod_virt; /**< producer index virtual address */
	u32		*cons_virt; /**< consumer index virtual address */

	/* Only will be used in the shadow Q which it is used internally */
	u32		prod_val; /**< producer index value */
	u32		cons_val; /**< consumer index value */
};

struct nmp_guest_giu_object {
	char	match[NMP_MAX_BUF_STR_LEN];
	u8	lf_type;
	u8	lf_id;
};

struct nmp_guest {
	u8	 id;
	u8	 lf_master_id;
	u32	 timeout;
	char	*prb_str;
	spinlock_t	sched_lock;
	spinlock_t	send_lock;
	struct nmp_guest_queue cmd_queue;
	struct nmp_guest_queue notify_queue;
	u32	total_giu_object_count;
	struct nmp_guest_giu_object giu_object[NMP_MAX_NUM_LFS * (1 + NMP_LF_MAX_NUM_LCL_BPOOLS)];
	void	*nmp;
	u32	keep_alive_thresh;
	u32	keep_alive_counter;

	/* Guest App parameters */
	u8	*msg;
	u32	max_msg_len;
	/* TODO - need to handle multiple app_cb */
	struct {
		enum nmp_guest_lf_type lf_type;
		u8 lf_id;
		u64 ev_mask;
		void *arg;
		int (*guest_ev_cb)(void *arg, enum nmp_guest_lf_type client, u8 id, u8 code,
			   u16 indx, void *msg, u16 len);
	} app_cb;

	int internal_schedule;
	struct nmp_guest_queue notify_shadow_queue;
	struct {
		void *arg;
		int (*cb)(void *arg, enum nmp_guest_lf_type client, u8 id, u8 code,
			   u16 indx, void *msg, u16 len);
	} internal_cb;

	struct {
		u8 code;
		u16 indx;
		void *resp;
		u16 resp_len;
		int got_resp; /* got resp > 0, error < 0 */
	} wait_for_resp;
};

int send_internal_msg(struct nmp_guest *guest,
		      enum cmd_dest_type client_type,
		      u8 client_id,
		      u8 code,
		      u16 indx,
		      void *msg,
		      u16 len,
		      void *resp,
		      u16 resp_len);


int guest_push_msg_to_q(struct nmp_guest_queue *q,
			enum cmd_dest_type client_type,
			u8 client_id,
			u8 code,
			u16 indx,
			void *msg,
			u16 len,
			int resp_required,
			u32 cons_idx,
			u32 *prod_idx);

struct nmp_guest *nmp_guest_get_handle(void);

#endif /* _NMP_GUEST_H */

