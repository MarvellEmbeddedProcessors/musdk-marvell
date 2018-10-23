/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _GIU_REGFILE_DEF_H
#define _GIU_REGFILE_DEF_H

#define UIO_MAX_NAME_STRING_SIZE 100

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
 *  base_phy_offset	Queue physical offset
 *  prod_offset Queue	producer address physical offset
 *  cons_offset Queue	consumer address physical offset
 *  buff_len		Buffer len (relevant for BPools only)
 *  payload_offset	Payload offset in the buffer (Relevant for Ingress data Qs only)
 */
struct giu_queue {
	int hw_id;
	int size;
	phys_addr_t phy_base_offset;
	enum giu_queue_type type;
	phys_addr_t prod_offset;
	phys_addr_t cons_offset;
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
	int num_ingress_tcs;
	int num_egress_tcs;
	struct giu_tc *ingress_tcs;
	struct giu_tc *egress_tcs;
	int num_bm_qs;
	struct giu_queue *bm_qs;
	char dma_uio_mem_name[UIO_MAX_NAME_STRING_SIZE];
};

#endif /* _GIU_REGFILE_DEF_H */
