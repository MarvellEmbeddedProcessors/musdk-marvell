/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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

