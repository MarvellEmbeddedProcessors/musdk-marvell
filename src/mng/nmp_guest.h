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
#ifndef _NMP_GUEST_H
#define _NMP_GUEST_H

#include "mng/mv_nmp_guest.h"
#include "lf/mng_cmd_desc.h"

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
};

struct nmp_guest {
	u8	 id;
	u8	 lf_master_id;
	u32	 timeout;
	char	*prb_str;
	struct nmp_guest_queue cmd_queue;
	struct nmp_guest_queue notify_queue;

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
};
#endif /* _NMP_GUEST_H */

