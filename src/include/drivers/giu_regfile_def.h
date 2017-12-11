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

#ifndef _GIU_REGFILE_DEF_H
#define _GIU_REGFILE_DEF_H


/* Regfile flags */
#define REGFILE_NOT_READY	0x0
#define REGFILE_READY		0x1

enum giu_queue_type {
	QUEUE_EGRESS,
	QUEUE_INGRESS,
	QUEUE_BP,
	QUEUE_NOT_SET = 0xFF
};

enum giu_ingress_hash_type {
	HASH_NONE = 0,		/* Invalid hash type (hashing mechanism is disabled) */
	HASH_2_TUPLE,		/* IP-src, IP-dst */
	HASH_5_TUPLE,		/* IP-src, IP-dst, IP-Prot, L4-src, L4-dst */
};

/* Contains queue information
 *
 *  hw_id		GIU Queue ID
 *  size		Number of entries in the queue
 *  type		Egress, Ingress, BM-Pool
 *  base_phy_addr	Queue physical address
 *  prod_addr Queue	producer address physical address
 *  cons_addr Queue	consumer address physical address
 *  buff_len		Buffer len (relevant for BPools only)
 *  payload_offset	Payload offset in the buffer (Relevant for Ingress data Qs only)
 */
struct giu_queue {
	int hw_id;
	int size;
	void *phy_base_addr;
	enum giu_queue_type type;
	void *prod_addr;
	void *cons_addr;
	union {
		int buff_len;
		int payload_offset;
	};
};

/* Contains TC information
 *
 *  id			TC ID
 *  num_queues		Number queues in this TC
 *  ingress_rss_type	Ingress RSS type (for egress should be set to HASH_NONE)
 *  queues		array of queue structures
 *  dest_num_queues	number of queues in Remote TC (for egress should be set to 0)
 */
struct giu_tc {
	int id;
	int num_queues;
	enum giu_ingress_hash_type ingress_rss_type;
	struct giu_queue *queues;
	/* Remote parameters */
	int dest_num_queues;
};

/* Contains PF's queues information
 *
 *  flags			flags that indicates system state or regfile state
 *  num_ingress_tcs		Number of ingress traffic class supported
 *  num_egress_tcs		Number of egress traffic class supported
 *  ingress_tcs			array of ingress TCs
 *  egress_tcs			array of egress TCs
 *  num_bm_qs			Number of BM Qs
 *  bm_qs			array of BM Qs
 */
struct giu_regfile {
	int version;
	int flags;
	void *prod_tbl_base_phys;
	void *prod_tbl_base_virt;
	void *cons_tbl_base_phys;
	void *cons_tbl_base_virt;
	int num_ingress_tcs;
	int num_egress_tcs;
	struct giu_tc *ingress_tcs;
	struct giu_tc *egress_tcs;
	int num_bm_qs;
	struct giu_queue *bm_qs;
};

#endif /* _GIU_REGFILE_DEF_H */
