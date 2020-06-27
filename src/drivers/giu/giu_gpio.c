/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#include "std_internal.h"

#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mv_giu_gpio.h"
#ifdef MVCONF_NMP_BUILT
#include "mng/mv_nmp_guest_giu.h"
#endif
#include "lib/lib_misc.h"
#include "lib/net.h"

#include "giu_internal.h"
#include "crc.h"

#define MAX_EXTRACTION_SIZE	(MV_IPV6ADDR_LEN * 2 + MV_IP_PROTO_NH_LEN + MV_L4_PORT_LEN * 2)

struct giu_gpio_lcl_q {
	u32			 q_id;
	struct mqa_q		*mqa_q;
	struct giu_gpio_queue	 queue; /* queue params for immediate use */
};

struct giu_gpio_rem_q {
	u32		 q_id;
	struct mqa_q	*mqa_q;
};


struct giu_gpio_interim_q_desc {
	u8	queue_idx;
	u8	res;
	u16	queue_desc_idx;
};

struct giu_gpio_interim_q {
	u32			 q_id;
	struct mqa_q		*mqa_q;
	struct giu_gpio_queue	 queue; /* queue params for immediate use */

	/* shadow */
	struct giu_gpio_interim_q_desc	*descs; /* save dst qid and prod_val*/
	u32	shadow_prod; /**< producer index value shadow  */
	u32	shadow_cons; /**< consumer index value shadow */
};

/** In TC - Queue topology
 */
struct giu_gpio_intc {
	u32			 pkt_offset;
	enum rss_hash_type	 rss_type;
	u32			 num_inqs;
	struct giu_gpio_lcl_q	 inqs[GIU_GPIO_TC_MAX_NUM_QS];

	u32			 num_inpools;
	struct giu_bpool	*pools[GIU_GPIO_TC_MAX_NUM_BPOOLS];

	u32			 num_rem_outqs;
	struct giu_gpio_rem_q	 rem_outqs[GIU_GPIO_TC_MAX_NUM_QS];

	u32			 num_interim_qs;
	int			 last_qid;
	struct giu_gpio_interim_q	interim_qs[GIU_GPIO_TC_MAX_NUM_QS];
};

struct giu_gpio_rem_inq {
	struct giu_gpio_rem_q	 q;
	struct giu_gpio_rem_q	 poolq;
};

/** Out TC - Queue topology
 */
struct giu_gpio_outtc {
	u32			 num_outqs;
	struct giu_gpio_lcl_q	 outqs[GIU_GPIO_TC_MAX_NUM_QS];

	u32			 rem_pkt_offset;
	enum rss_hash_type	 rem_rss_type;

	u32			 num_rem_inqs;
	struct giu_gpio_rem_inq	 rem_inqs[GIU_GPIO_TC_MAX_NUM_QS];

	u32			 num_interim_qs;
	int			 last_interim_qid;
	struct giu_gpio_interim_q	interim_qs[GIU_GPIO_TC_MAX_NUM_QS];
};

/**
 * GPIO Handler
 */
struct giu_gpio {
	u32			giu_id;	/**< GIU's Id */
	u32			id; /**< GPIO Id */
	char			match[20];
	int			is_guest;
	int			is_enable;

	struct mqa		*mqa;
	struct giu		*giu;

	int			 sg_en;
	u32			 num_intcs;
	struct giu_gpio_intc	 intcs[GIU_GPIO_MAX_NUM_TCS];
	u32			 num_outtcs;
	struct giu_gpio_outtc	 outtcs[GIU_GPIO_MAX_NUM_TCS];
};

static int destroy_q(struct giu *giu, enum giu_eng eng, struct mqa *mqa,
	struct mqa_q *q, u32 q_id, struct mqa_q *src_q, enum queue_type queue_type)
{
	struct gie *gie = NULL;
	int ret = 0;

	if (unlikely(!giu)) {
		pr_err("Invalid GIU handle!\n");
		return -EINVAL;
	}

	if (eng < GIU_ENG_OUT_OF_RANGE)
		gie = giu_get_gie_handle(giu, eng);

	if (q) {
		u32 tmp_q_id;

		mqa_queue_get_id(q, &tmp_q_id);
		if (tmp_q_id != q_id) {
			pr_err("mismatch between q object and its q-ID (%d vs %d)!\n",
				tmp_q_id, q_id);
			return -EINVAL;
		}

		pr_debug("Remove queue %d (type %d)\n", q_id, queue_type);

		if (gie) {
			/* Un-register Q from GIU */
			if (queue_type == LOCAL_BM_QUEUE ||
			    queue_type == HOST_BM_QUEUE)
				ret = gie_remove_bm_queue(gie, q_id);
			else if (queue_type == HOST_INGRESS_DATA_QUEUE) {
				u32 src_qid;

				if (!src_q) {
					pr_err("src-q for queue Idx %d is NULL\n", q_id);
					return ret;
				}
				mqa_queue_get_id(src_q, &src_qid);
				ret = gie_remove_queue(gie, src_qid);
			} else
				ret = gie_remove_queue(gie, q_id);
			if (ret) {
				pr_err("Failed to remove queue Idx %d from GIU\n", q_id);
				return ret;
			}
		}

		ret = mqa_queue_destroy(mqa, q);
		if (ret) {
			pr_err("Failed to free queue Idx %x in DB\n", q_id);
			return ret;
		}
	}

	/* Free the MQA resource */
	ret = mqa_queue_free(mqa, q_id);
	if (ret) {
		pr_err("Failed to free queue Idx %x in MQA\n", q_id);
		return ret;
	}

	return 0;
}

static inline u64 giu_gpio_outq_desc_get_phys_addr(struct giu_gpio_desc *desc)
{
	/* cmd[4] and cmd[5] holds the buffer physical address (Low and High parts) */
	return ((u64)(desc->cmds[5] & GIU_TXD_BUF_PHYS_HI_MASK) << 32) | (u64)desc->cmds[4];
}

static inline void *giu_gpio_outq_desc_get_addr(struct giu_gpio_desc *desc)
{
	/* check first if user set its own virtual-address; in that case, we should use it */
	if (desc->cmds[6])
		return (void *)(uintptr_t)(((u64)((desc->cmds[7] &
			GIU_TXD_BUF_VIRT_HI_MASK) >> 0) << 32) |
				(u64)desc->cmds[6]);
	/* if user didn't set its own VA, we assume the buffer was allocated by MUSDK dma-pool */
	return mv_sys_dma_mem_phys2virt(giu_gpio_outq_desc_get_phys_addr(desc));
}

static int giu_gpio_update_rss(struct giu_gpio *gpio, u8 tc, struct giu_gpio_desc *desc)
{
	uint8_t key_size = 0, *extracted_key;
	const uint8_t *ip_frame;
	uint64_t crc64 = CRC64_DEFAULT_INITVAL;
	enum giu_outq_l3_type l3_info = GIU_TXD_GET_L3_PRS_INFO(desc);
	enum giu_outq_l4_type l4_info = GIU_TXD_GET_L4_PRS_INFO(desc);
	int ipv4 = 0;
	u16 queue_index;

	if (GIU_TXD_GET_HK_MODE(desc)) {
		/* Hash Key exist in descriptor, lets use it */
		queue_index = GIU_TXD_GET_HASH_KEY(desc) % gpio->outtcs[tc].num_rem_inqs;
		GIU_TXD_SET_DEST_QID(desc, queue_index);
		return 0;
	}

	if (unlikely(!(l3_info >= GIU_OUTQ_L3_TYPE_IPV4_NO_OPTS && l3_info <= GIU_OUTQ_L3_TYPE_IPV6_EXT))) {
		/* Not a IP packet. No need to perform RSS. Set Dest-Qid as index '0' */
		GIU_TXD_SET_DEST_QID(desc, 0);
		return 0;
	}

	/* Need to Perform RSS and Packet is IP */
	if (l3_info >= GIU_OUTQ_L3_TYPE_IPV4_NO_OPTS && l3_info <= GIU_OUTQ_L3_TYPE_IPV4_TTL_ZERO)
		ipv4 = 1;

	ip_frame = (uint8_t *)giu_gpio_outq_desc_get_addr(desc) + GIU_TXD_GET_PKT_OFF(desc) + GIU_TXD_GET_L3_OFF(desc);
	struct mv_ipv4hdr *ipv4hdr = (struct mv_ipv4hdr *)ip_frame;
	struct mv_ipv6hdr *ipv6hdr = (struct mv_ipv6hdr *)ip_frame;

	if (ipv4) {
		extracted_key = ipv4hdr->src_addr;
		key_size = (MV_IPV4ADDR_LEN * 2);
	} else {
		extracted_key = ipv6hdr->src_addr;
		key_size = (MV_IPV6ADDR_LEN * 2);
	}
	crc64 = crc64_compute(extracted_key, key_size, crc64);

	if ((gpio->outtcs[tc].rem_rss_type == RSS_HASH_5_TUPLE) &&
		(l4_info == GIU_OUTQ_L4_TYPE_TCP || l4_info == GIU_OUTQ_L4_TYPE_UDP)) {
		struct mv_udphdr *l4hdr;

		if (ipv4)
			extracted_key = &ipv4hdr->proto;
		else
			extracted_key = &ipv6hdr->next_header;

		crc64 = crc64_compute(extracted_key, MV_IP_PROTO_NH_LEN, crc64);
		l4hdr = (struct mv_udphdr *)(ip_frame + (GIU_TXD_GET_IPHDR_LEN(desc) * 4));
		crc64 = crc64_compute((uint8_t *)&l4hdr->src_port, (MV_L4_PORT_LEN * 2), crc64);
	}

	giu_gpio_outq_desc_set_hk_mode(desc, crc64);

	queue_index = crc64 % gpio->outtcs[tc].num_rem_inqs;
	GIU_TXD_SET_DEST_QID(desc, queue_index);

	return 0;
}

/* copies from M interim to N local queues */
int giu_gpio_pre_gie(struct giu_gpio *gpio)
{
	struct giu_gpio_outtc			*outtc;
	struct giu_gpio_queue			*txq, *rxq;
	struct giu_gpio_desc			*tx_ring_base, *rx_ring_base;
	struct giu_gpio_desc			*dst_desc, *desc;
	struct giu_gpio_interim_q		*interimq;
	struct giu_gpio_interim_q_desc		*shadow_desc;
	u32 cons, prod;
	int desc_received, shadow_desc_recv;
	int dest_qid;
	int i, tc;
	int break_interimq_loop = 0;

	if (!gpio->is_enable)
		return 0;

	/* in current GPIO pass all TC */
	for (tc = 0; tc < gpio->num_outtcs; tc++) {

		outtc = &(gpio->outtcs[tc]);

		u32 prod_val[outtc->num_outqs];
		u32 cons_val[outtc->num_outqs];
		u32 free_count[outtc->num_outqs];

		/* get DST queues information */
		rmb();
		for (i = 0; i < outtc->num_outqs; i++) {
			prod_val[i] = readl_relaxed(outtc->outqs[i].queue.prod_addr);
			cons_val[i] = readl_relaxed(outtc->outqs[i].queue.cons_addr);
			free_count[i] = QUEUE_SPACE(prod_val[i], cons_val[i], outtc->outqs[i].queue.desc_total);
		}

		break_interimq_loop = 0; /* break if dest queue is full */

		for (int q = 0; q < outtc->num_interim_qs && break_interimq_loop == 0; q++) {
			/* for example: qid = 2 0 1... */
			outtc->last_interim_qid = (outtc->last_interim_qid + q) % outtc->num_interim_qs;
			interimq = &outtc->interim_qs[outtc->last_interim_qid];

			/* get src queue info */
			rxq = &outtc->interim_qs[outtc->last_interim_qid].queue;
			rx_ring_base = rxq->desc_ring_base;
			prod = readl(rxq->prod_addr);
			cons = readl_relaxed(rxq->cons_addr);

			/* num of desc from last time */
			shadow_desc_recv = QUEUE_OCCUPANCY(
					interimq->shadow_prod, interimq->shadow_cons, interimq->queue.desc_total);

			for (i = 0; i < shadow_desc_recv; i++) {
				shadow_desc = &interimq->descs[interimq->shadow_cons];
				/* if empty - GIE still didn't take previous packets to dest (local) queue */

				if (QUEUE_OCCUPANCY(cons_val[shadow_desc->queue_idx],
					shadow_desc->queue_desc_idx, interimq->queue.desc_total) == 0)
					break;

				interimq->shadow_cons =
						QUEUE_INDEX_INC(interimq->shadow_cons, 1, interimq->queue.desc_total);
			}
			writel(interimq->shadow_cons, interimq->queue.cons_addr);

			desc_received = QUEUE_OCCUPANCY(prod, cons, rxq->desc_total);
			/* num of descs this time */
			desc_received = desc_received - shadow_desc_recv;

			/* real cons (take to the account already processed packets) */
			cons = QUEUE_INDEX_INC(cons, shadow_desc_recv, interimq->queue.desc_total);

			/* get desc from SRC-Q, calculate dest_qid(RSS), copy to correspondent DST-Q */
			for (i = 0; i < desc_received; i++) {

				/* copy 1 desc from SRC-Q */
				desc = &rx_ring_base[cons];

				/* complete RSS  calculation */
				dest_qid = GIU_TXD_GET_HASH_KEY(desc) % outtc->num_rem_inqs;

				if (free_count[dest_qid] == 0) {
					/* break desc_received loop */
					break_interimq_loop = 1;
					break;
				}

				if (gpio->sg_en) {
					if (GIU_TXD_GET_FORMAT(desc) == GIU_FORMAT_DIRECT_SG) {
						u8 num_sg_ent = GIU_TXD_GET_NUM_SG_ENT(desc) + 2;

						if (free_count[dest_qid] < num_sg_ent) {
							/* break desc_received loop */
							break_interimq_loop = 1;
							break;
						}
					}
				}

				free_count[dest_qid]--;

				/* only now update cons */
				cons = QUEUE_INDEX_INC(cons, 1, rxq->desc_total);

				/* copy 1 by 1 desc to local queue */
				txq = &outtc->outqs[dest_qid].queue;
				tx_ring_base = (struct giu_gpio_desc *)txq->desc_ring_base;
				dst_desc = &tx_ring_base[prod_val[dest_qid]];

				memcpy(dst_desc, desc, sizeof(struct giu_gpio_desc));

				/* update shadow-queue with dest-qid and index */
				interimq->descs[interimq->shadow_prod].queue_idx = dest_qid;
				interimq->descs[interimq->shadow_prod].queue_desc_idx = prod_val[dest_qid];
				/* prepare for  next descr */
				interimq->shadow_prod =
						QUEUE_INDEX_INC(interimq->shadow_prod, 1, interimq->queue.desc_total);

				/* increment producer index */
				prod_val[dest_qid] = QUEUE_INDEX_INC(prod_val[dest_qid], 1, txq->desc_total);
			} /* desc_received-loop */

			/* update consumer index in GNCT - postponed */

		} /* interims-loop */

		/* update producer index in GNPT */
		wmb();
		for (i = 0; i < outtc->num_outqs; i++) {
			txq = &outtc->outqs[i].queue;
			writel_relaxed(prod_val[i], txq->prod_addr);
		}

	} /* tc-loop */

	return 0;
}

/* copies from N local to M interim queues */
int giu_gpio_post_gie(struct giu_gpio *gpio)
{
	struct giu_gpio_queue	*dst_q, *src_q;
	struct giu_gpio_desc	*dst_q_ring_base, *src_q_ring_base;
	struct giu_gpio_intc	*intc;
	struct giu_gpio_desc	*dst_desc, *src_desc;
	u32 prod_val, cons_val;
	int desc_received;
	int dst_qid;
	int break_qid_loop = 0;
	int i, tc;

	if (!gpio->is_enable)
		return 0;

	/* pass all TC */
	for (tc = 0; tc < gpio->num_intcs; tc++) {

		intc = &(gpio->intcs[tc]);

		u32 cons[gpio->num_intcs];
		u32 prod[gpio->num_intcs];
		u32 free_count[gpio->num_intcs];

		/* get dest (interim) queues info */
		rmb();
		for (dst_qid = 0; dst_qid < intc->num_interim_qs; dst_qid++) {
			dst_q = &intc->interim_qs[dst_qid].queue;
			prod[dst_qid] = readl_relaxed(dst_q->prod_addr);
			cons[dst_qid] = readl_relaxed(dst_q->cons_addr);
			free_count[dst_qid] = QUEUE_SPACE(prod[dst_qid], cons[dst_qid], dst_q->desc_total);
		}

		break_qid_loop = 0;

		for (int q = 0; q < intc->num_inqs && break_qid_loop == 0; q++) {

			/* for example: qid = 2 0 1... */
			intc->last_qid = (intc->last_qid + q) % intc->num_inqs;

			/* get SRC queues info */
			src_q = &intc->inqs[intc->last_qid].queue;
			src_q_ring_base = (struct giu_gpio_desc *)src_q->desc_ring_base;
			prod_val = readl_relaxed(src_q->prod_addr);
			cons_val = readl_relaxed(src_q->cons_addr);

			desc_received = QUEUE_OCCUPANCY(prod_val, cons_val, src_q->desc_total);

			for (i = 0; i < desc_received; i++) {
				src_desc = &src_q_ring_base[cons_val];

				/* RSS */
				/* TODO :change to RXD */
				dst_qid = GIU_TXD_GET_HASH_KEY(src_desc) % intc->num_interim_qs;

				if (free_count[dst_qid] == 0) {
					/* break if dest queue is full */
					break_qid_loop = 1;
					break;
				}

				if (gpio->sg_en) {
					if (GIU_RXD_GET_FORMAT(src_desc) == GIU_FORMAT_DIRECT_SG) {
						u8 num_sg_ent = GIU_RXD_GET_NUM_SG_ENT(src_desc) + 2;

						if (free_count[dst_qid] < num_sg_ent) {
							/* break desc_received loop */
							break_qid_loop = 1;
							break;
						}
					}
				}

				free_count[dst_qid]--;

				dst_q = &intc->interim_qs[dst_qid].queue;
				dst_q_ring_base = (struct giu_gpio_desc *)dst_q->desc_ring_base;
				dst_desc = &dst_q_ring_base[prod[dst_qid]];

				memcpy(dst_desc, src_desc, sizeof(struct giu_gpio_desc));

				/* increment consumer/producer index */
				cons_val = QUEUE_INDEX_INC(cons_val, 1, src_q->desc_total);
				prod[dst_qid] = QUEUE_INDEX_INC(prod[dst_qid], 1, dst_q->desc_total);

			} /* desc_received -loop */

			writel(cons_val, src_q->cons_addr);

		} /* qid-loop */

		/* update producer index in GNPT */
		wmb();
		for (dst_qid = 0; dst_qid < intc->num_interim_qs; dst_qid++) {
			dst_q = &intc->interim_qs[dst_qid].queue;
			writel_relaxed(prod[dst_qid], dst_q->prod_addr);
		}

	} /* tc-loop */

	return 0;
}

int giu_gpio_init(struct giu_gpio_params *params, struct giu_gpio **gpio)
{
	struct giu_gpio_intc_params	*intc_par;
	struct giu_gpio_intc		*intc = NULL;
	struct giu_gpio_outtc		*outtc = NULL;
	struct mqa_queue_params		 mqa_params;
	struct mqa_queue_info		 info;
	u8				 match_params[2];
	u32				 bm_pool_num;
	u32				 tc_idx, q_idx;
	int				 giu_id, gpio_id;
	int				 ret;

	if (!params->match) {
		pr_err("no match string found!\n");
		return -EFAULT;
	}

	if (mv_sys_match(params->match, "gpio", 2, match_params))
		return(-ENXIO);

	giu_id = match_params[0];
	gpio_id = match_params[1];

	if (gpio_id >= GIU_MAX_NUM_GPIO) {
		pr_err("giu_id (%d) exceeds max gpio number (%d)\n", giu_id, GIU_MAX_NUM_GPIO);
		return -1;
	}

	*gpio = kcalloc(1, sizeof(struct giu_gpio), GFP_KERNEL);
	if (*gpio == NULL) {
		pr_err("Failed to allocate GIU GPIO handler\n");
		goto kcalloc_error;
	}

	(*gpio)->giu_id = giu_id;
	(*gpio)->id = gpio_id;
	(*gpio)->is_guest = 0;
	(*gpio)->is_enable = 0;
	strcpy((*gpio)->match, params->match);

	(*gpio)->mqa = params->mqa;
	(*gpio)->giu = params->giu;
	(*gpio)->sg_en = params->sg_en;

	giu_gpio_register(params->giu, *gpio, gpio_id);

	pr_debug("Initializing Out-TC queues (#%d)\n", params->num_outtcs);
	(*gpio)->num_outtcs = params->num_outtcs;
	for (tc_idx = 0; tc_idx < (*gpio)->num_outtcs; tc_idx++) {

		outtc = &((*gpio)->outtcs[tc_idx]);

		/* create mqa out-interim-q */
		/* num of "guest point of view" local queues */
		outtc->num_interim_qs = params->outtcs_params[tc_idx].num_outqs;

		for (q_idx = 0; q_idx < outtc->num_interim_qs; q_idx++) {
			ret = mqa_queue_alloc((*gpio)->mqa, &outtc->interim_qs[q_idx].q_id);
			if (ret < 0) {
				pr_err("Failed to allocate queue from MQA\n");
				goto interim_ing_queue_error;
			}

			memset(&mqa_params, 0, sizeof(struct mqa_queue_params));
			mqa_params.idx  = outtc->interim_qs[q_idx].q_id;
			mqa_params.len	= params->outtcs_params[tc_idx].outqs_params[q_idx].len;
			mqa_params.size = gie_get_desc_size(RX_DESC);
			mqa_params.attr = MQA_QUEUE_LOCAL | MQA_QUEUE_INGRESS;
			mqa_params.copy_payload = 1;

			ret = mqa_queue_create((*gpio)->mqa, &mqa_params, &(outtc->interim_qs[q_idx].mqa_q));
			if (ret < 0) {
				pr_err("Failed to allocate local ingress queue %d\n", mqa_params.idx);
				goto interim_ing_queue_error;
			}

			mqa_queue_get_info(outtc->interim_qs[q_idx].mqa_q, &info);
			outtc->interim_qs[q_idx].queue.desc_total = info.len;
			outtc->interim_qs[q_idx].queue.desc_ring_base = info.virt_base_addr;
			outtc->interim_qs[q_idx].queue.last_cons_val = 0;
			outtc->interim_qs[q_idx].queue.prod_addr = info.prod_virt;
			outtc->interim_qs[q_idx].queue.cons_addr = info.cons_virt;
			outtc->interim_qs[q_idx].queue.payload_offset = 0;
			memset(info.virt_base_addr, 0, sizeof(struct giu_gpio_desc));

			/* allocate memory for interim shadow */
			outtc->interim_qs[q_idx].descs =
					kzalloc(sizeof(struct giu_gpio_interim_q_desc) * info.len, GFP_KERNEL);
			if (outtc->interim_qs[q_idx].descs == NULL) {
				pr_err("Failed to allocate GIU GPIO TC %d shadow-queue %d for RSS\n", tc_idx, q_idx);
				goto interim_ing_queue_kzalloc_error;
			}
			outtc->interim_qs[q_idx].shadow_prod = 0;
			outtc->interim_qs[q_idx].shadow_cons = 0;
		} /* for interimq q_idx  */
	} /* for tc_idx */

	pr_debug("Initializing In TC queues (#%d)\n", params->num_intcs);

	(*gpio)->num_intcs = params->num_intcs;
	for (tc_idx = 0; tc_idx < (*gpio)->num_intcs; tc_idx++) {
		intc_par = &(params->intcs_params[tc_idx]);
		intc = &((*gpio)->intcs[tc_idx]);

		/* pools */
		intc->num_inpools = intc_par->num_inpools;
		for (bm_pool_num = 0; bm_pool_num < intc_par->num_inpools; bm_pool_num++)
			intc->pools[bm_pool_num] = intc_par->pools[bm_pool_num];

		/* create mqa in interim-q */
		/* num of "guest point of view" local queues */
		intc->num_interim_qs = params->intcs_params[tc_idx].num_inqs;

		for (q_idx = 0; q_idx < intc->num_interim_qs ; q_idx++) {
			ret = mqa_queue_alloc((*gpio)->mqa, &intc->interim_qs[q_idx].q_id);
			if (ret < 0) {
				pr_err("Failed to allocate queue from MQA\n");
				goto interim_eg_queue_error;
			}

			memset(&mqa_params, 0, sizeof(struct mqa_queue_params));
			mqa_params.idx  = intc->interim_qs[q_idx].q_id;
			mqa_params.len	= params->intcs_params[tc_idx].inqs_params[q_idx].len;
			mqa_params.size = gie_get_desc_size(TX_DESC);
			mqa_params.attr = MQA_QUEUE_LOCAL | MQA_QUEUE_EGRESS;
			mqa_params.copy_payload = 1;

			ret = mqa_queue_create((*gpio)->mqa, &mqa_params, &(intc->interim_qs[q_idx].mqa_q));
			if (ret < 0) {
				pr_err("Failed to allocate local ingress queue %d\n", mqa_params.idx);
				goto interim_eg_queue_error;
			}

			mqa_queue_get_info(intc->interim_qs[q_idx].mqa_q, &info);
			intc->interim_qs[q_idx].queue.desc_total = info.len;
			intc->interim_qs[q_idx].queue.desc_ring_base = info.virt_base_addr;
			intc->interim_qs[q_idx].queue.last_cons_val = 0;
			intc->interim_qs[q_idx].queue.prod_addr = info.prod_virt;
			intc->interim_qs[q_idx].queue.cons_addr = info.cons_virt;
			intc->interim_qs[q_idx].queue.payload_offset = 0;
			memset(info.virt_base_addr, 0, sizeof(struct giu_gpio_desc));
		}
	}

	return 0;

interim_eg_queue_error:
	pr_debug("De-initializing Local Egress queues\n");
	for (tc_idx = 0; tc_idx < (*gpio)->num_intcs; tc_idx++) {
		intc = &((*gpio)->intcs[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_interim_qs; q_idx++) {
			ret = destroy_q((*gpio)->giu, GIU_ENG_OUT_OF_RANGE, (*gpio)->mqa,
					intc->interim_qs[q_idx].mqa_q,
					intc->interim_qs[q_idx].q_id,
					NULL,
					LOCAL_EGRESS_DATA_QUEUE);
			if (ret)
				pr_warn("Failed to remove queue Idx %x\n",
					intc->interim_qs[q_idx].q_id);
		}
	}

interim_ing_queue_kzalloc_error:
	if (outtc) {
		for (q_idx = 0; q_idx < outtc->num_interim_qs; q_idx++)
			kfree(outtc->interim_qs[q_idx].descs);
	}

interim_ing_queue_error:
	pr_debug("De-initializing Local Out queues\n");
	for (tc_idx = 0; tc_idx < (*gpio)->num_outtcs; tc_idx++) {
		outtc = &((*gpio)->outtcs[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_interim_qs; q_idx++) {
			ret = destroy_q((*gpio)->giu, GIU_ENG_OUT_OF_RANGE, (*gpio)->mqa,
					outtc->interim_qs[q_idx].mqa_q,
					outtc->interim_qs[q_idx].q_id,
					NULL,
					LOCAL_INGRESS_DATA_QUEUE);
			if (ret)
				pr_warn("Failed to remove queue Idx %x\n",
					outtc->interim_qs[q_idx].q_id);
		}
	}

kcalloc_error:
	return -1;
}

int giu_gpio_set_remote(struct giu_gpio *gpio, struct giu_gpio_rem_params *params)
{
	struct mqa_queue_params		 mqa_params;
	struct giu_gpio_outtc_rem_params *outtc_par;
	struct giu_gpio_intc_rem_params	*intc_par;
	struct giu_gpio_rem_q_params	*rem_q_par;
	struct giu_gpio_intc		*intc;
	struct giu_gpio_outtc		*outtc;
	struct giu_gpio_rem_q		*rem_q;
	s32				 pair_qid;
	u32				 tc_idx, bm_idx, q_idx;
	int				 ret;

	pr_debug("Initializing Out-TC queues (#%d)\n", params->num_outtcs);
	gpio->num_outtcs = params->num_outtcs;
	for (tc_idx = 0; tc_idx < gpio->num_outtcs; tc_idx++) {
		struct giu_gpio_lcl_q		*lcl_q;
		struct mqa_queue_info		 queue_info;

		outtc_par = &(params->outtcs_params[tc_idx]);
		outtc = &(gpio->outtcs[tc_idx]);

		/* Create Local Out queues */
		pr_debug("Initializing Local Out queues\n");
		outtc->num_outqs = outtc_par->num_rem_inqs;

		for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {
			lcl_q = &(outtc->outqs[q_idx]);
			rem_q_par = &(outtc_par->rem_inqs_params[q_idx].poolq_params);
			rem_q = &(outtc->rem_inqs[q_idx].poolq);

			/* Allocate queue from MQA */
			ret = mqa_queue_alloc(gpio->mqa, &lcl_q->q_id);
			if (ret < 0) {
				pr_err("Failed to allocate queue from MQA\n");
				goto lcl_ing_queue_error;
			}

			memset(&mqa_params, 0, sizeof(struct mqa_queue_params));
			mqa_params.idx  = lcl_q->q_id;
			mqa_params.len	= rem_q_par->len;
			mqa_params.size = gie_get_desc_size(RX_DESC);
			mqa_params.attr = MQA_QUEUE_LOCAL | MQA_QUEUE_INGRESS;
			mqa_params.copy_payload = 1;
			mqa_params.sg_en = gpio->sg_en;

			ret = mqa_queue_create(gpio->mqa, &mqa_params, &(lcl_q->mqa_q));
			if (ret < 0) {
				pr_err("Failed to allocate local ingress queue %d\n", mqa_params.idx);
				goto lcl_ing_queue_error;
			}

			mqa_queue_get_info(lcl_q->mqa_q, &queue_info);

			lcl_q->queue.desc_total = queue_info.len;
			lcl_q->queue.desc_ring_base = queue_info.virt_base_addr;
			lcl_q->queue.last_cons_val = 0;
			lcl_q->queue.prod_addr = queue_info.prod_virt;
			lcl_q->queue.cons_addr = queue_info.cons_virt;
			lcl_q->queue.payload_offset = 0;
			memset(queue_info.virt_base_addr, 0, sizeof(struct giu_gpio_desc));
		}

		/* Create Remote BM queues */
		pr_debug("Initializing Remote BM queues\n");
		outtc->rem_pkt_offset = outtc_par->rem_pkt_offset;
		if (outtc->rem_pkt_offset) {
			pr_err("remote side pkt-offset is not supported yet!\n");
			ret = -EINVAL;
			goto lcl_ing_queue_error;
		}
		outtc->rem_rss_type = (enum rss_hash_type)outtc_par->rem_rss_type;
		outtc->num_rem_inqs = outtc_par->num_rem_inqs;
		for (bm_idx = 0; bm_idx < outtc->num_rem_inqs; bm_idx++) {
			rem_q_par = &(outtc_par->rem_inqs_params[bm_idx].poolq_params);
			rem_q = &(outtc->rem_inqs[bm_idx].poolq);

			/* Allocate queue from MQA */
			ret = mqa_queue_alloc(gpio->mqa, &rem_q->q_id);
			if (ret < 0) {
				pr_err("Failed to allocate queue from MQA\n");
				goto host_queue_error;
			}

			memset(&mqa_params, 0, sizeof(struct mqa_queue_params));
			mqa_params.idx		   = rem_q->q_id;
			mqa_params.len		   = rem_q_par->len;
			mqa_params.size		   = rem_q_par->size;
			mqa_params.attr		   = (u32)(MQA_QUEUE_REMOTE | MQA_QUEUE_EGRESS);
			mqa_params.remote_phy_addr = (void *)(uintptr_t)rem_q_par->q_base_pa;
			mqa_params.prod_phys	   = (void *)(uintptr_t)rem_q_par->prod_base_pa;
			mqa_params.prod_virt	   = rem_q_par->prod_base_va;
			mqa_params.cons_phys	   = (void *)(uintptr_t)rem_q_par->cons_base_pa;
			mqa_params.cons_virt	   = rem_q_par->cons_base_va;
			mqa_params.host_remap	   = rem_q_par->host_remap;

			ret = mqa_queue_create(gpio->mqa, &mqa_params,  &(rem_q->mqa_q));
			if (ret < 0) {
				pr_err("Failed to allocate Host BM queue %d\n", mqa_params.idx);
				goto host_queue_error;
			}

			/* Register Host BM Queue to GIU */
			ret = gie_add_bm_queue(giu_get_gie_handle(gpio->giu, GIU_ENG_OUT),
					mqa_params.idx, rem_q_par->buff_len, GIU_REM_Q);
			if (ret) {
				pr_err("Failed to register BM Queue %d to GIU\n", mqa_params.idx);
				goto host_queue_error;
			}
			pr_debug("Host TC[%d] BM-pool[%d], queue Id %d, Registered to GIU RX\n\n",
				 tc_idx, bm_idx, mqa_params.idx);
		}

		/* Create Remote In queues */
		pr_debug("Initializing Remote In queues\n");
		for (q_idx = 0; q_idx < outtc->num_rem_inqs; q_idx++) {
			rem_q_par = &(outtc_par->rem_inqs_params[q_idx].q_params);
			rem_q = &(outtc->rem_inqs[q_idx].q);

			/* Allocate queue from MQA */
			ret = mqa_queue_alloc(gpio->mqa, &rem_q->q_id);
			if (ret < 0) {
				pr_err("Failed to allocate queue from MQA\n");
				goto host_ing_queue_error;
			}

			memset(&mqa_params, 0, sizeof(struct mqa_queue_params));
			mqa_params.idx	= rem_q->q_id;
			mqa_params.len	= rem_q_par->len;
			mqa_params.size = rem_q_par->size;
			mqa_params.attr = MQA_QUEUE_REMOTE | MQA_QUEUE_INGRESS;
			mqa_params.prio = tc_idx;
			mqa_params.remote_phy_addr =
				(void *)(uintptr_t)rem_q_par->q_base_pa;
			mqa_params.prod_phys	   =
				(void *)(uintptr_t)rem_q_par->prod_base_pa;
			mqa_params.prod_virt	   = rem_q_par->prod_base_va;
			mqa_params.cons_phys	   =
				(void *)(uintptr_t)rem_q_par->cons_base_pa;
			mqa_params.cons_virt	   = rem_q_par->cons_base_va;
			mqa_params.host_remap	   = rem_q_par->host_remap;
			mqa_params.bpool_num	   = 1;
			mqa_params.bpool_qids[0]   = outtc->rem_inqs[q_idx].poolq.q_id;

			pr_debug("Host TC[%d] RX[%d], queue Id %d, len %d, bpool ID %d trying to Registered to GIU RX\n\n",
				 tc_idx, q_idx, mqa_params.idx, mqa_params.len, mqa_params.bpool_qids[0]);

			/* Set message info */
			mqa_params.msix_inf.va = rem_q_par->msix_inf.va;
			mqa_params.msix_inf.pa = rem_q_par->msix_inf.pa;
			mqa_params.msix_inf.data = rem_q_par->msix_inf.data;
			mqa_params.msix_inf.mask_address = rem_q_par->msix_inf.mask_address;
			mqa_params.msix_inf.mask_value = rem_q_par->msix_inf.mask_value;

			ret = mqa_queue_create(gpio->mqa, &mqa_params, &(rem_q->mqa_q));
			if (ret < 0) {
				pr_err("Failed to allocate queue for Host BM\n");
				goto host_ing_queue_error;
			}

			pair_qid = outtc->outqs[q_idx].q_id;

			ret = mqa_queue_associate_pair(gpio->mqa, pair_qid, rem_q->q_id);
			if (ret) {
				pr_err("Failed to associate remote egress Queue %d\n",
					   rem_q->q_id);
				goto host_ing_queue_error;
			}

			ret = gie_add_queue(giu_get_gie_handle(gpio->giu, GIU_ENG_OUT),
					pair_qid, GIU_LCL_Q);
			if (ret) {
				pr_err("Failed to register Host Egress Queue %d to GIU\n",
					   rem_q->q_id);
				goto host_ing_queue_error;
			}
		}
	}

	pr_debug("Initializing In TC queues (#%d)\n", params->num_intcs);

	gpio->num_intcs = params->num_intcs;
	for (tc_idx = 0; tc_idx < gpio->num_intcs; tc_idx++) {
		struct giu_gpio_lcl_q		*lcl_q;
		struct mqa_queue_info		 queue_info;
		u32				 bm_pool_num;

		intc_par = &(params->intcs_params[tc_idx]);
		intc = &(gpio->intcs[tc_idx]);

		/* Create Local In queues */
		pr_debug("Initializing Local In queues\n");

		intc->num_inqs = intc_par->num_rem_outqs;

		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++) {
			lcl_q = &(intc->inqs[q_idx]);
			rem_q_par = &(intc_par->rem_outqs_params[q_idx]);
			rem_q = &(intc->rem_outqs[q_idx]);

			/* Allocate queue from MQA */
			ret = mqa_queue_alloc(gpio->mqa, &lcl_q->q_id);
			if (ret < 0) {
				pr_err("Failed to allocate queue from MQA\n");
				goto lcl_eg_queue_error;
			}

			memset(&mqa_params, 0, sizeof(struct mqa_queue_params));
			mqa_params.idx  = lcl_q->q_id;
			mqa_params.len	= rem_q_par->len;
			mqa_params.size = gie_get_desc_size(TX_DESC);
			mqa_params.attr = MQA_QUEUE_LOCAL | MQA_QUEUE_EGRESS;
			mqa_params.bpool_num = intc->num_inpools;
			for (bm_pool_num = 0; bm_pool_num < intc->num_inpools; bm_pool_num++)
				mqa_params.bpool_qids[bm_pool_num] =
						giu_bpool_get_mqa_q_id(intc->pools[bm_pool_num]);

			ret = mqa_queue_create(gpio->mqa, &mqa_params, &(lcl_q->mqa_q));
			if (ret < 0) {
				pr_err("Failed to allocate local egress queue %d\n", mqa_params.idx);
				goto lcl_eg_queue_error;
			}

			mqa_queue_get_info(lcl_q->mqa_q, &queue_info);
			lcl_q->queue.desc_total = queue_info.len;
			lcl_q->queue.desc_ring_base = queue_info.virt_base_addr;
			lcl_q->queue.last_cons_val = 0;
			lcl_q->queue.prod_addr = queue_info.prod_virt;
			lcl_q->queue.cons_addr = queue_info.cons_virt;
			lcl_q->queue.payload_offset = 0;
		}

		/* Create Remote Out queues */
		pr_debug("Initializing Remote Out queues\n");
		intc->num_rem_outqs = intc_par->num_rem_outqs;
		for (q_idx = 0; q_idx < intc->num_rem_outqs; q_idx++) {
			rem_q_par = &(intc_par->rem_outqs_params[q_idx]);
			rem_q = &(intc->rem_outqs[q_idx]);

			/* Allocate queue from MQA */
			ret = mqa_queue_alloc(gpio->mqa, &rem_q->q_id);
			if (ret < 0) {
				pr_err("Failed to allocate queue from MQA\n");
				goto host_eg_queue_error;
			}

			memset(&mqa_params, 0, sizeof(struct mqa_queue_params));
			mqa_params.idx	= rem_q->q_id;
			mqa_params.len	= rem_q_par->len;
			mqa_params.size = rem_q_par->size;
			mqa_params.attr = MQA_QUEUE_REMOTE | MQA_QUEUE_EGRESS;
			mqa_params.prio = tc_idx;
			mqa_params.remote_phy_addr =
				(void *)(uintptr_t)rem_q_par->q_base_pa;
			mqa_params.prod_phys	   =
				(void *)(uintptr_t)rem_q_par->prod_base_pa;
			mqa_params.prod_virt	   = rem_q_par->prod_base_va;
			mqa_params.cons_phys	   =
				(void *)(uintptr_t)rem_q_par->cons_base_pa;
			mqa_params.cons_virt	   = rem_q_par->cons_base_va;
			mqa_params.host_remap	   = rem_q_par->host_remap;
			mqa_params.copy_payload    = 1;
			mqa_params.sg_en	   = gpio->sg_en;

			/* Set message info */
			mqa_params.msix_inf.va = rem_q_par->msix_inf.va;
			mqa_params.msix_inf.pa = rem_q_par->msix_inf.pa;
			mqa_params.msix_inf.data = rem_q_par->msix_inf.data;
			mqa_params.msix_inf.mask_address = rem_q_par->msix_inf.mask_address;
			mqa_params.msix_inf.mask_value = rem_q_par->msix_inf.mask_value;

			ret = mqa_queue_create(gpio->mqa, &mqa_params, &(rem_q->mqa_q));
			if (ret < 0) {
				pr_err("Failed to allocate queue for Host BM\n");
				goto host_eg_queue_error;
			}

			pair_qid = intc->inqs[q_idx].q_id;

			ret = mqa_queue_associate_pair(gpio->mqa, rem_q->q_id, pair_qid);
			if (ret) {
				pr_err("Failed to associate remote egress Queue %d\n",
					   rem_q->q_id);
				goto host_eg_queue_error;
			}

			/* Register Host Egress Queue to GIU */
			ret = gie_add_queue(giu_get_gie_handle(gpio->giu, GIU_ENG_IN),
					rem_q->q_id, GIU_REM_Q);
			if (ret) {
				pr_err("Failed to register Host Egress Queue %d to GIU\n",
					   rem_q->q_id);
				goto host_eg_queue_error;
			}
		}
	}

	return 0;

host_eg_queue_error:
	pr_debug("De-initializing Remote Out queues\n");
	for (tc_idx = 0; tc_idx < gpio->num_intcs; tc_idx++) {
		intc = &(gpio->intcs[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_rem_outqs; q_idx++) {
			ret = destroy_q(gpio->giu, GIU_ENG_IN, gpio->mqa,
					intc->rem_outqs[q_idx].mqa_q,
					intc->rem_outqs[q_idx].q_id,
					intc->inqs[q_idx].mqa_q,
					HOST_EGRESS_DATA_QUEUE);
			if (ret)
				pr_warn("Failed to remove queue Idx %x\n",
					intc->rem_outqs[q_idx].q_id);
		}
	}

lcl_eg_queue_error:
	pr_debug("De-initializing Local In queues\n");
	for (tc_idx = 0; tc_idx < gpio->num_intcs; tc_idx++) {
		intc = &(gpio->intcs[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++) {
			ret = destroy_q(gpio->giu, GIU_ENG_OUT_OF_RANGE, gpio->mqa,
					intc->inqs[q_idx].mqa_q,
					intc->inqs[q_idx].q_id,
					NULL,
					LOCAL_EGRESS_DATA_QUEUE);
			if (ret)
				pr_warn("Failed to remove queue Idx %x\n",
					intc->inqs[q_idx].q_id);
		}
		intc->num_inqs = intc->num_rem_outqs = 0;
	}

host_ing_queue_error:
	pr_debug("De-initializing Remote In queues\n");
	for (tc_idx = 0; tc_idx < gpio->num_outtcs; tc_idx++) {
		outtc = &(gpio->outtcs[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_rem_inqs; q_idx++) {
			ret = destroy_q(gpio->giu, GIU_ENG_OUT, gpio->mqa,
					outtc->rem_inqs[q_idx].q.mqa_q,
					outtc->rem_inqs[q_idx].q.q_id,
					outtc->outqs[q_idx].mqa_q,
					HOST_INGRESS_DATA_QUEUE);
			if (ret)
				pr_warn("Failed to remove queue Idx %x\n",
					outtc->rem_inqs[q_idx].q.q_id);
		}
	}

host_queue_error:
	pr_debug("De-initializing Remote in BM queues\n");
	for (tc_idx = 0; tc_idx < gpio->num_outtcs; tc_idx++) {
		outtc = &(gpio->outtcs[tc_idx]);

		for (bm_idx = 0; bm_idx < outtc->num_rem_inqs; bm_idx++) {
			ret = destroy_q(gpio->giu, GIU_ENG_OUT, gpio->mqa,
					outtc->rem_inqs[bm_idx].poolq.mqa_q,
					outtc->rem_inqs[bm_idx].poolq.q_id,
					NULL,
					HOST_BM_QUEUE);
			if (ret)
				pr_warn("Failed to remove queue Idx %x\n",
					outtc->rem_inqs[bm_idx].poolq.q_id);
		}
	}

lcl_ing_queue_error:
	pr_debug("De-initializing Local Out queues\n");
	for (tc_idx = 0; tc_idx < gpio->num_outtcs; tc_idx++) {
		outtc = &(gpio->outtcs[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {
			ret = destroy_q(gpio->giu, GIU_ENG_OUT_OF_RANGE, gpio->mqa,
					outtc->outqs[q_idx].mqa_q,
					outtc->outqs[q_idx].q_id,
					NULL,
					LOCAL_INGRESS_DATA_QUEUE);
			if (ret)
				pr_warn("Failed to remove queue Idx %x\n",
					outtc->outqs[q_idx].q_id);
		}
		outtc->num_rem_inqs = outtc->num_outqs = 0;
	}

	return -1;
}

void giu_gpio_clear_remote(struct giu_gpio *gpio)
{
	struct giu_gpio_outtc	*outtc;
	struct giu_gpio_intc	*intc;
	struct giu_gpio_interim_q	*interimq;
	u32			 tc_idx, bm_idx, q_idx;
	int			 ret;

	pr_debug("De-initializing Remote Out queues\n");
	for (tc_idx = 0; tc_idx < gpio->num_intcs; tc_idx++) {
		intc = &(gpio->intcs[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_rem_outqs; q_idx++) {
			ret = destroy_q(gpio->giu, GIU_ENG_IN, gpio->mqa,
					intc->rem_outqs[q_idx].mqa_q,
					intc->rem_outqs[q_idx].q_id,
					intc->inqs[q_idx].mqa_q,
					HOST_EGRESS_DATA_QUEUE);
			if (ret)
				pr_warn("Failed to destroy queue Idx %x\n",
					intc->rem_outqs[q_idx].q_id);
		}
	}

	pr_debug("De-initializing Local Egress queues\n");
	for (tc_idx = 0; tc_idx < gpio->num_intcs; tc_idx++) {
		intc = &(gpio->intcs[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++) {
			ret = destroy_q(gpio->giu, GIU_ENG_OUT_OF_RANGE, gpio->mqa,
					intc->inqs[q_idx].mqa_q,
					intc->inqs[q_idx].q_id,
					NULL,
					LOCAL_EGRESS_DATA_QUEUE);
			if (ret)
				pr_warn("Failed to remove queue Idx %x\n",
					intc->inqs[q_idx].q_id);
		}
		intc->num_inqs = intc->num_rem_outqs = 0;
	}

	pr_debug("De-initializing Remote In queues\n");
	for (tc_idx = 0; tc_idx < gpio->num_outtcs; tc_idx++) {
		outtc = &(gpio->outtcs[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_rem_inqs; q_idx++) {
			ret = destroy_q(gpio->giu, GIU_ENG_OUT, gpio->mqa,
					outtc->rem_inqs[q_idx].q.mqa_q,
					outtc->rem_inqs[q_idx].q.q_id,
					outtc->outqs[q_idx].mqa_q,
					HOST_INGRESS_DATA_QUEUE);
			if (ret)
				pr_warn("Failed to destroy queue Idx %x\n",
					outtc->rem_inqs[q_idx].q.q_id);
		}
	}

	pr_debug("De-initializing Local Ingress queues\n");
	for (tc_idx = 0; tc_idx < gpio->num_outtcs; tc_idx++) {
		outtc = &(gpio->outtcs[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {
			ret = destroy_q(gpio->giu, GIU_ENG_OUT_OF_RANGE, gpio->mqa,
					outtc->outqs[q_idx].mqa_q,
					outtc->outqs[q_idx].q_id,
					NULL,
					LOCAL_INGRESS_DATA_QUEUE);
			if (ret)
				pr_warn("Failed to remove queue Idx %x\n",
					outtc->outqs[q_idx].q_id);
		}

		pr_debug("Clear Shadow queues\n");
		/* As some descriptors were marked as "sent" but the
		 * destination local queue was just reset, those descriptors should be marked as free.
		 */
		for (q_idx = 0; q_idx < outtc->num_interim_qs; q_idx++) {
			interimq = &outtc->interim_qs[q_idx];

			interimq->shadow_cons = interimq->shadow_prod;
			writel(interimq->shadow_cons, interimq->queue.cons_addr);
		}
	}

	pr_debug("De-initializing Remote in BM queues\n");
	for (tc_idx = 0; tc_idx < gpio->num_outtcs; tc_idx++) {
		outtc = &(gpio->outtcs[tc_idx]);

		for (bm_idx = 0; bm_idx < outtc->num_rem_inqs; bm_idx++) {
			ret = destroy_q(gpio->giu, GIU_ENG_OUT, gpio->mqa,
					outtc->rem_inqs[bm_idx].poolq.mqa_q,
					outtc->rem_inqs[bm_idx].poolq.q_id,
					NULL,
					HOST_BM_QUEUE);
			if (ret)
				pr_warn("Failed to destroy queue Idx %x\n",
					outtc->rem_inqs[bm_idx].poolq.q_id);
		}
		outtc->num_rem_inqs = outtc->num_outqs = 0;
	}
}

void giu_gpio_deinit(struct giu_gpio *gpio)
{
	struct giu_gpio_outtc	*outtc;
	struct giu_gpio_intc	*intc;
	u32			 tc_idx, q_idx;
	int			 ret;

	giu_gpio_clear_remote(gpio);

	pr_debug("De-initializing Interim In queues\n");
	for (tc_idx = 0; tc_idx < gpio->num_intcs; tc_idx++) {
		intc = &(gpio->intcs[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_interim_qs; q_idx++) {
			ret = destroy_q(gpio->giu, GIU_ENG_OUT_OF_RANGE, gpio->mqa,
					intc->interim_qs[q_idx].mqa_q,
					intc->interim_qs[q_idx].q_id,
					NULL,
					LOCAL_EGRESS_DATA_QUEUE);
			if (ret)
				pr_warn("Failed to remove queue Idx %x\n",
					intc->interim_qs[q_idx].q_id);
		}
	}

	pr_debug("De-initializing Interim Out queues\n");
	for (tc_idx = 0; tc_idx < gpio->num_outtcs; tc_idx++) {
		outtc = &(gpio->outtcs[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_interim_qs; q_idx++) {

			kfree(outtc->interim_qs[q_idx].descs);

			ret = destroy_q(gpio->giu, GIU_ENG_OUT_OF_RANGE, gpio->mqa,
					outtc->interim_qs[q_idx].mqa_q,
					outtc->interim_qs[q_idx].q_id,
					NULL,
					LOCAL_INGRESS_DATA_QUEUE);
			if (ret)
				pr_warn("Failed to remove queue Idx %x\n",
					outtc->interim_qs[q_idx].q_id);
		}
	}

	giu_gpio_unregister(gpio->giu, gpio, gpio->id);

	kfree(gpio);
}

int giu_gpio_serialize(struct giu_gpio *gpio, char *buff, u32 size, u8 depth)
{
	struct giu_gpio_outtc		*outtc;
	struct giu_gpio_intc		*intc;
	size_t				 pos = 0;
	struct mv_sys_dma_mem_info	 mem_info;
	char				 dev_name[100];
	struct mqa_queue_info		 queue_info;
	u32				 offs;
	int				 tc_idx, q_idx, bpool_id;

	if (!gpio) {
		pr_err("invalid gpio handle!\n");
		return -EINVAL;
	}

	mem_info.name = dev_name;
	mv_sys_dma_mem_get_info(&mem_info);

	json_print_to_buffer(buff, size, depth, "\"gpio-%d:%d\": {\n",
			     gpio->giu_id, gpio->id);
	json_print_to_buffer(buff, size, depth + 1, "\"giu_id\": %d,\n", gpio->giu_id);
	json_print_to_buffer(buff, size, depth + 1, "\"id\": %d,\n", gpio->id);
	json_print_to_buffer(buff, size, depth + 1, "\"sg_en\": %d,\n", gpio->sg_en);
	json_print_to_buffer(buff, size, depth + 1, "\"dma_dev_name\": \"%s\",\n", mem_info.name);

	/* Serialize IN TCs info */
	json_print_to_buffer(buff, size, depth + 1, "\"num_intcs\": %u,\n", gpio->num_intcs);
	for (tc_idx = 0; tc_idx < gpio->num_intcs; tc_idx++) {
		intc = &(gpio->intcs[tc_idx]);
		json_print_to_buffer(buff, size, depth + 1, "\"intc-%u\": {\n", tc_idx);
		json_print_to_buffer(buff, size, depth + 2, "\"pkt-offs\": %u,\n", intc->pkt_offset);
		json_print_to_buffer(buff, size, depth + 2, "\"rss-type\": %u,\n", intc->rss_type);

		/* Serialize IN Qs info */
		json_print_to_buffer(buff, size, depth + 2, "\"num_inqs\": %u,\n", intc->num_interim_qs);
		for (q_idx = 0; q_idx < intc->num_interim_qs; q_idx++) {
			mqa_queue_get_info(intc->interim_qs[q_idx].mqa_q, &queue_info);	/* interim */

			json_print_to_buffer(buff, size, depth + 3, "\"qid\": %u,\n", queue_info.q_id);
			json_print_to_buffer(buff, size, depth + 3, "\"qlen\": %u,\n", queue_info.len);
			offs = (phys_addr_t)(uintptr_t)queue_info.phy_base_addr - mem_info.paddr;
			json_print_to_buffer(buff, size, depth + 3, "\"phy_base_offset\": %#x,\n", offs);
			offs = (phys_addr_t)(uintptr_t)queue_info.prod_phys - mem_info.paddr;
			json_print_to_buffer(buff, size, depth + 3, "\"prod_offset\": %#x,\n", offs);
			offs = (phys_addr_t)(uintptr_t)queue_info.cons_phys - mem_info.paddr;
			json_print_to_buffer(buff, size, depth + 3, "\"cons_offset\": %#x,\n", offs);
			json_print_to_buffer(buff, size, depth + 2, "},\n");
		}

		/* Serialize IN BPool */
		json_print_to_buffer(buff, size, depth + 2, "\"num_inpools\": %u,\n", intc->num_inpools);
		for (bpool_id = 0; bpool_id < intc->num_inpools; bpool_id++)
			json_print_to_buffer(buff, size, depth + 2, "\"bpid\": %u,\n", intc->pools[bpool_id]->id);
		json_print_to_buffer(buff, size, depth + 1, "},\n");
	}

	/* Serialize OUT TCs info */
	json_print_to_buffer(buff, size, depth + 1, "\"num_outtcs\": %u,\n", gpio->num_outtcs);
	for (tc_idx = 0; tc_idx < gpio->num_outtcs; tc_idx++) {
		outtc = &(gpio->outtcs[tc_idx]);
		json_print_to_buffer(buff, size, depth + 1, "\"outtc-%u\": {\n", tc_idx);

		/* Serialize OUT Qs info */
		json_print_to_buffer(buff, size, depth + 2, "\"num_outqs\": %u,\n", outtc->num_interim_qs);
		for (q_idx = 0; q_idx < outtc->num_interim_qs; q_idx++) {
			json_print_to_buffer(buff, size, depth + 2, "\"outq-%u\": {\n", q_idx);
			mqa_queue_get_info(outtc->interim_qs[q_idx].mqa_q, &queue_info); /* interim */
			json_print_to_buffer(buff, size, depth + 3, "\"qid\": %u,\n", queue_info.q_id);
			json_print_to_buffer(buff, size, depth + 3, "\"qlen\": %u,\n", queue_info.len);
			offs = (phys_addr_t)(uintptr_t)queue_info.phy_base_addr - mem_info.paddr;
			json_print_to_buffer(buff, size, depth + 3, "\"phy_base_offset\": %#x,\n", offs);
			offs = (phys_addr_t)(uintptr_t)queue_info.prod_phys - mem_info.paddr;
			json_print_to_buffer(buff, size, depth + 3, "\"prod_offset\": %#x,\n", offs);
			offs = (phys_addr_t)(uintptr_t)queue_info.cons_phys - mem_info.paddr;
			json_print_to_buffer(buff, size, depth + 3, "\"cons_offset\": %#x,\n", offs);
			json_print_to_buffer(buff, size, depth + 2, "},\n");
		}
		json_print_to_buffer(buff, size, depth + 1, "},\n");
	}

	json_print_to_buffer(buff, size, depth, "},\n");

	return pos;
}

int giu_gpio_probe(char *match, char *buff, struct giu_gpio **gpio)
{
	struct giu_gpio			*_gpio;
	struct giu_gpio_outtc		*outtc;
	struct giu_gpio_intc		*intc;
	struct sys_iomem_params		 iomem_params;
	struct sys_iomem_info		 sys_iomem_info;
	char				*lbuff, *sec = NULL;
	char				 dev_name[FILE_MAX_LINE_CHARS];
	u8				 match_params[2];
	u32				 offs = 0;
	u32				 tc_idx, q_idx, bp_idx;
	u8				 giu_id = 0, gpio_id = 0;

	if (!match) {
		pr_err("no match string found!\n");
		return -EFAULT;
	}

	lbuff = kcalloc(1, strlen(buff), GFP_KERNEL);
	if (lbuff == NULL)
		return -ENOMEM;

	memcpy(lbuff, buff, strlen(buff));
	sec = lbuff;

	/* Search for match (gpio-x:x) */
	sec = strstr(sec, match);
	if (!sec) {
		pr_err("match not found %s\n", match);
		kfree(lbuff);
		return -ENXIO;
	}

	if (mv_sys_match(match, "gpio", 2, match_params))
		return(-ENXIO);

	/* Retireve giu_id and pool-id */
	json_buffer_to_input(sec, "giu_id", giu_id);
	json_buffer_to_input(sec, "id", gpio_id);

	if ((giu_id != match_params[0]) || (gpio_id != match_params[1])) {
		pr_err("IDs mismatch!\n");
		kfree(lbuff);
		return -EFAULT;
	}
	if (gpio_id >= GIU_MAX_NUM_GPIO) {
		pr_err("giu_id (%d) exceeds max gpio number (%d)\n", giu_id, GIU_MAX_NUM_GPIO);
		return -EINVAL;
	}

	pr_debug("probing: gpio %d for giu id: %d.\n", gpio_id, giu_id);

	_gpio = kcalloc(1, sizeof(struct giu_gpio), GFP_KERNEL);
	if (_gpio == NULL)
		return -ENOMEM;

	_gpio->giu_id = giu_id;
	_gpio->id = gpio_id;
	_gpio->is_guest = 1;
	_gpio->is_enable = 0;
	strcpy(_gpio->match, match);
	json_buffer_to_input(sec, "sg_en", _gpio->sg_en);

	memset(dev_name, 0, FILE_MAX_LINE_CHARS);
	json_buffer_to_input_str(sec, "dma_dev_name", dev_name);
	if (dev_name[0] == 0) {
		pr_err("'dma_dev_name' not found\n");
		kfree(_gpio);
		kfree(lbuff);
		return -EFAULT;
	}

	iomem_params.type = SYS_IOMEM_T_SHMEM;
	iomem_params.devname = dev_name;
	iomem_params.index = 1;

	if (sys_iomem_get_info(&iomem_params, &sys_iomem_info)) {
		pr_err("sys_iomem_get_info error\n");
		kfree(_gpio);
		kfree(lbuff);
		return -EFAULT;
	}

	json_buffer_to_input(sec, "num_intcs", _gpio->num_intcs);
	for (tc_idx = 0; tc_idx < _gpio->num_intcs; tc_idx++) {
		intc = &(_gpio->intcs[tc_idx]);

		json_buffer_to_input(sec, "pkt-offs", intc->pkt_offset);
		json_buffer_to_input(sec, "rss-type", intc->rss_type);
		json_buffer_to_input(sec, "num_inqs", intc->num_inqs);
		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++) {
			json_buffer_to_input(sec, "qid", intc->inqs[q_idx].q_id);
			intc->inqs[q_idx].queue.payload_offset = intc->pkt_offset;
			json_buffer_to_input(sec, "qlen", intc->inqs[q_idx].queue.desc_total);
			json_buffer_to_input(sec, "phy_base_offset", offs);
			intc->inqs[q_idx].queue.desc_ring_base =
				(struct giu_gpio_desc *)((uintptr_t)sys_iomem_info.u.shmem.va + offs);
			json_buffer_to_input(sec, "prod_offset", offs);
			intc->inqs[q_idx].queue.prod_addr =
				(u32 *)((uintptr_t)sys_iomem_info.u.shmem.va + offs);
			json_buffer_to_input(sec, "cons_offset", offs);
			intc->inqs[q_idx].queue.cons_addr =
				(u32 *)((uintptr_t)sys_iomem_info.u.shmem.va + offs);
			intc->inqs[q_idx].queue.last_cons_val = 0;
		}

		json_buffer_to_input(sec, "num_inpools", intc->num_inpools);
		for (bp_idx = 0; bp_idx < intc->num_inpools; bp_idx++) {
			u8 bp_id = 0;

			json_buffer_to_input(sec, "bpid", bp_id);
			intc->pools[bp_idx] = &giu_bpools[bp_id];
		}
	}

	json_buffer_to_input(sec, "num_outtcs", _gpio->num_outtcs);
	for (tc_idx = 0; tc_idx < _gpio->num_outtcs; tc_idx++) {
		outtc = &(_gpio->outtcs[tc_idx]);

		json_buffer_to_input(sec, "num_outqs", outtc->num_outqs);
		for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {
			json_buffer_to_input(sec, "qid", outtc->outqs[q_idx].q_id);
			json_buffer_to_input(sec, "qlen", outtc->outqs[q_idx].queue.desc_total);
			json_buffer_to_input(sec, "phy_base_offset", offs);
			outtc->outqs[q_idx].queue.desc_ring_base =
				(struct giu_gpio_desc *)((uintptr_t)sys_iomem_info.u.shmem.va + offs);
			json_buffer_to_input(sec, "prod_offset", offs);
			outtc->outqs[q_idx].queue.prod_addr =
				(u32 *)((uintptr_t)sys_iomem_info.u.shmem.va + offs);
			json_buffer_to_input(sec, "cons_offset", offs);
			outtc->outqs[q_idx].queue.cons_addr =
				(u32 *)((uintptr_t)sys_iomem_info.u.shmem.va + offs);
			outtc->outqs[q_idx].queue.last_cons_val = 0;
		}
	}

	kfree(lbuff);
	*gpio = _gpio;

	giu_gpio_disable(_gpio);
	giu_gpio_reset(_gpio);

	return 0;
}

void giu_gpio_remove(struct giu_gpio *gpio)
{
	kfree(gpio);
}

int giu_gpio_enable(struct giu_gpio *gpio)
{
	struct giu_gpio_outtc	*outtc;
	struct giu_gpio_intc	*intc;
	u32			 tc_idx, q_idx;
	int			 err;
	struct gie		*gie = NULL;

	if (gpio->is_guest) {
		err = nmp_guest_giu_gpio_enable(gpio->match);
		if (err)
			return err;
	} else {
		/* Resume all SRC-Qs to GIE scheduling */
		pr_debug("Suspend Remote Out queues\n");
		for (tc_idx = 0; tc_idx < gpio->num_intcs; tc_idx++) {
			intc = &(gpio->intcs[tc_idx]);
			outtc = &(gpio->outtcs[tc_idx]);

			gie = giu_get_gie_handle(gpio->giu, GIU_ENG_IN);
			for (q_idx = 0; q_idx < intc->num_rem_outqs; q_idx++) {
				err = gie_resume_queue(gie, intc->rem_outqs[q_idx].q_id);
				if (err) {
					pr_err("Failed to resume queue Idx %d\n",
						intc->rem_outqs[q_idx].q_id);
					return err;
				}
			}

			gie = giu_get_gie_handle(gpio->giu, GIU_ENG_OUT);
			for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {
				err = gie_resume_queue(gie, outtc->outqs[q_idx].q_id);
				if (err) {
					pr_err("Failed to resume queue Idx %d\n",
						outtc->outqs[q_idx].q_id);
					return err;
				}
			}
		}
	}

	gpio->is_enable = 1;

	return 0;
}

int giu_gpio_disable(struct giu_gpio *gpio)
{
	struct giu_gpio_outtc	*outtc;
	struct giu_gpio_intc	*intc;
	u32			 tc_idx, q_idx;
	int			 err;
	struct gie		*gie = NULL;

	if (gpio->is_guest) {
		err = nmp_guest_giu_gpio_disable(gpio->match);
		if (err)
			return err;
	} else {
		/* Suspend all SRC-Qs from GIE scheduling */
		pr_debug("Suspend Remote Out queues\n");
		for (tc_idx = 0; tc_idx < gpio->num_intcs; tc_idx++) {
			intc = &(gpio->intcs[tc_idx]);
			outtc = &(gpio->outtcs[tc_idx]);

			gie = giu_get_gie_handle(gpio->giu, GIU_ENG_IN);
			for (q_idx = 0; q_idx < intc->num_rem_outqs; q_idx++) {
				err = gie_suspend_queue(gie, intc->rem_outqs[q_idx].q_id);
				if (err) {
					pr_err("Failed to suspend queue Idx %d\n",
						intc->rem_outqs[q_idx].q_id);
					return err;
				}
			}

			gie = giu_get_gie_handle(gpio->giu, GIU_ENG_OUT);
			for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {
				err = gie_suspend_queue(gie, outtc->outqs[q_idx].q_id);
				if (err) {
					pr_err("Failed to suspend queue Idx %d\n",
						outtc->outqs[q_idx].q_id);
					return err;
				}
			}
		}
	}

	gpio->is_enable = 0;

	return 0;
}

int giu_gpio_get_link_state(struct giu_gpio *gpio, int *en)
{
#ifdef MVCONF_NMP_BUILT
	if (gpio->is_guest)
		return nmp_guest_giu_gpio_get_link_state(gpio->match, en);
	else
#endif
	pr_debug("%s is not implemented\n", __func__);

	return -ENOTSUP;
}

int giu_gpio_send(struct giu_gpio *gpio, u8 tc, u8 qid, struct giu_gpio_desc *descs, u16 *num)
{
	struct giu_gpio_outtc *outtc = &(gpio->outtcs[tc]);
	struct giu_gpio_queue *txq;
	struct giu_gpio_desc *tx_ring_base;
	u16 num_txds = *num, desc_remain;
	u16 block_size, index;
	u32 free_count, cons_val, prod_val;
	int i;

#ifdef GIU_GPIO_DEBUG
	/* Check that the requested TC is supported */
	if (tc >= gpio->num_outtcs) {
		pr_err("GIU GPIO: TC %d is not supported (Max TC is %d)\n", tc, gpio->num_outtcs);
		return -1;
	}

	/* Check that the requested Q ID exists */
	if (qid >= outtc->num_outqs) {
		pr_err("GIU GPIO: Q %d is not supported (Max Q is %d)\n", qid,
			outtc->num_outqs);
		return -1;
	}
#endif

	/* Get queue params */
	txq = &outtc->outqs[qid].queue;

	/* Get ring base */
	tx_ring_base = (struct giu_gpio_desc *)txq->desc_ring_base;

	/* Read consumer index */
	cons_val = readl(txq->cons_addr);
	/* Read producer index */
	prod_val = readl(txq->prod_addr);

#ifdef GIU_GPIO_DEBUG
	if (num_txds > txq->desc_total)
		pr_debug("More tx_descs(%u) than txq_len(%u)\n",
			num_txds, txq->desc_total);
#endif

	/* Calculate number of free descriptors */
	free_count = QUEUE_SPACE(prod_val, cons_val, txq->desc_total);
	if (unlikely(free_count < num_txds)) {
		pr_debug("num_txds(%d), free_count(%d) (GPIO %d)\n", num_txds, free_count, gpio->id);
		num_txds = free_count;
	}

	if (unlikely(!num_txds)) {
		pr_debug("GPIO full\n");
		*num = 0;
		return 0;
	}

	/* In case there is a wrap-around, handle the number of desc till the end of queue */
	block_size = min(num_txds, (u16)(txq->desc_total - prod_val));

	desc_remain = num_txds;
	index = 0; /* index in source descriptor array */

	/* In case there is a wrap-around, the first iteration will handle the
	 * descriptors till the end of queue. The rest will be handled at the
	 * following iteration.
	 * Note that there should be no more than 2 iterations.
	 **/
	do {
		if (outtc->num_rem_inqs > 1) {
			/* Calculate RSS and update descriptor */
			for (i = 0; i < block_size; i++)
				giu_gpio_update_rss(gpio, tc, &descs[i + index]);
		}

		/* Copy bulk of descriptors to descriptor ring */
		memcpy(&tx_ring_base[prod_val], &descs[index], sizeof(*tx_ring_base) * block_size);

		/* Increment producer index, update remaining descriptors count and block size */
		prod_val = QUEUE_INDEX_INC(prod_val, block_size, txq->desc_total);
		desc_remain -= block_size;
		index = block_size;	/* next desc index in source array */
		block_size = desc_remain; /* next desc index in target ring */
	} while (desc_remain > 0);

	/* Update Producer index in GNPT */
	/* make sure all writes are done (i.e. descriptor were copied)
	 * before incrementing the producer index
	 */
	writel(prod_val, txq->prod_addr);

	/* Update number of sent descriptors */
	*num = num_txds;

	return 0;
}

/* Calculate the number of transmitted packets by consumer value */
int giu_gpio_get_num_outq_done(struct giu_gpio *gpio, u8 tc, u8 qid, u16 *num)
{
	u32 tx_num = 0;
	u32 cons_val;
	struct giu_gpio_queue *txq = &gpio->outtcs[tc].outqs[qid].queue;

	/* Read consumer index */
	cons_val = readl_relaxed(txq->cons_addr);

	tx_num = QUEUE_OCCUPANCY(cons_val, txq->last_cons_val, txq->desc_total);
#ifdef GIU_GPIO_DEBUG
	if (tx_num)
		pr_debug("last %d cons %d total %d, num %d\n", txq->last_cons_val, cons_val, txq->desc_total, tx_num);
#endif
	txq->last_cons_val = cons_val;
	*num = tx_num;

	return 0;
}

int giu_gpio_recv(struct giu_gpio *gpio, u8 tc, u8 qid, struct giu_gpio_desc *descs, u16 *num)
{
	struct giu_gpio_queue *rxq;
	struct giu_gpio_desc *rx_ring_base;
	struct giu_gpio_intc *intc = &(gpio->intcs[tc]);
	u16 recv_req = *num, desc_received, desc_remain = 0;
	u16 block_size, index;
	u32 prod_val, cons_val;

	*num = 0;

#ifdef GIU_GPIO_DEBUG
	/* Check that the requested TC is supported */
	if (tc >= gpio->num_intcs) {
		pr_err("GIU GPIO: TC %d is not supported (Max TC is %d)\n", tc,
			gpio->num_intcs);
		return -1;
	}

	/* Check that the requested Q ID exists */
	if (qid >= intc->num_inqs) {
		pr_err("GIU GPIO: Q %d is not supported (Max Q is %d)\n", qid,
			intc->num_inqs);
		return -1;
	}
#endif

	rxq = &intc->inqs[qid].queue;

	/* Get ring base */
	rx_ring_base = (struct giu_gpio_desc *)rxq->desc_ring_base;

	/* Read producer index */
	prod_val = readl(rxq->prod_addr);
	/* Read consumer index */
	cons_val = readl(rxq->cons_addr);

	/* Calculate number of received descriptors in the ring.
	 * Since queue size is a power of 2, we can use below formula.
	 */
	desc_received = QUEUE_OCCUPANCY(prod_val, cons_val, rxq->desc_total);
	if (desc_received == 0) {
		pr_debug("desc_received is zero\n");
		return 0;
	}

	recv_req = min(recv_req, desc_received);

	/* In case there is a wrap around the descriptors are be stored to the
	 * end of the ring AND from the beginning of the desc ring.
	 * So the size of the first block is the number of descriptor till the
	 * end of the ring.
	 */
	if (unlikely((cons_val + recv_req) > rxq->desc_total)) {
		block_size = rxq->desc_total - cons_val;
	} else {
		/* No wrap around */
		block_size = recv_req;
	}

	desc_remain = recv_req;
	index = 0; /* index in destination descriptor array */

	/* Note: since we handle wrap-around, the should be no more than 2 iterations */
	do {
		/* Copy bulk of descriptors from the descriptor ring */
		memcpy(&descs[index], &rx_ring_base[cons_val], block_size * sizeof(*descs));

		/* Increment consumer index, update remaining descriptors count and block size */
		cons_val = QUEUE_INDEX_INC(cons_val, block_size, rxq->desc_total);
		desc_remain -= block_size;
		index = block_size; /* next desc index in destination array */
		block_size = desc_remain; /* next desc index in source ring */
	} while (desc_remain);

	/* Update Consumer index in GNCT */
	/* make sure all writes are done (i.e. descriptor were copied)
	 * before incrementing the consumer index
	 */
	writel(cons_val, rxq->cons_addr);

	/* Update number of received descriptors */
	*num = recv_req;

	return 0;
}

void giu_gpio_reset(struct giu_gpio *gpio)
{
	struct giu_gpio_outtc	*outtc;
	struct giu_gpio_intc	*intc;
	struct giu_gpio_queue	*q;
	int tc_idx, q_idx;

	if (gpio->is_guest) {
		nmp_guest_giu_gpio_reset(gpio->match);
		return;
	}

	for (tc_idx = 0; tc_idx < gpio->num_intcs; tc_idx++) {
		intc = &gpio->intcs[tc_idx];

		/* Clear intc interim qs */
		for (q_idx = 0; q_idx < intc->num_interim_qs; q_idx++) {
			q = &intc->interim_qs[q_idx].queue;
			writel(0, q->cons_addr);
			writel(0, q->prod_addr);
		}

		/* Drain intc local-q from old buffers */
		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++) {
			q = &intc->inqs[q_idx].queue;
			writel(readl_relaxed(q->prod_addr), q->cons_addr);
		}
	}

	/* Clear outtc interim qs */
	for (tc_idx = 0; tc_idx < gpio->num_outtcs; tc_idx++) {
		outtc = &gpio->outtcs[tc_idx];
		for (q_idx = 0; q_idx < outtc->num_interim_qs; q_idx++) {
			q = &outtc->interim_qs[q_idx].queue;
			writel(0, q->cons_addr);
			writel(0, q->prod_addr);
			outtc->interim_qs[q_idx].shadow_cons = 0;
			outtc->interim_qs[q_idx].shadow_prod = 0;
		}
	}
}

int giu_gpio_get_capabilities(struct giu_gpio *gpio, struct giu_gpio_capabilities *capa)
{
	int tc_idx, q_idx, bpool_id;

	/* Set ID */
	capa->id = gpio->giu_id;
	capa->sg_en = gpio->sg_en;

	/* Set number if Egress TCs */
	capa->intcs_inf.num_intcs = gpio->num_intcs;

	/* Set Egress TCs info */
	for (tc_idx = 0; tc_idx < capa->intcs_inf.num_intcs; tc_idx++) {
		struct giu_gpio_intc_info *tc_info = &capa->intcs_inf.intcs_inf[tc_idx];
		int qs_num = gpio->intcs[tc_idx].num_inqs;

		/* Set number if Egress Qs in this TC */
		tc_info->num_inqs = qs_num;

		/* Set Egress Qs info */
		for (q_idx = 0; q_idx < qs_num; q_idx++) {
			struct giu_gpio_q_info *q_info = &capa->intcs_inf.intcs_inf[tc_idx].inqs_inf[q_idx];
			struct giu_gpio_queue  *queue = &gpio->intcs[tc_idx].inqs[q_idx].queue;

			q_info->offset = queue->payload_offset;
			q_info->size = queue->desc_total - 1;
		}

		/* Set BPool handlers */
		/* TODO: for now we have a single BP per GPIO */
		tc_info->pools[0] = &giu_bpools[0];
		for (bpool_id = 1; bpool_id < qs_num; bpool_id++)
			tc_info->pools[bpool_id] = NULL;
	}

	/* Set number if Ingress TCs */
	capa->outtcs_inf.num_outtcs = gpio->num_outtcs;

	/* Set Ingress TCs info */
	for (tc_idx = 0; tc_idx < capa->outtcs_inf.num_outtcs; tc_idx++) {
		struct giu_gpio_outtc_info *tc_info = &capa->outtcs_inf.outtcs_inf[tc_idx];
		int qs_num = gpio->outtcs[tc_idx].num_outqs;

		/* Set number if Ingress Qs in this TC */
		tc_info->num_outqs = qs_num;

		/* Set Ingress Qs info */
		for (q_idx = 0; q_idx < qs_num; q_idx++) {
			struct giu_gpio_q_info *q_info = &capa->outtcs_inf.outtcs_inf[tc_idx].outqs_inf[q_idx];
			struct giu_gpio_q_info *done_q_info = &capa->outtcs_inf.outtcs_inf[tc_idx].doneqs_inf[q_idx];
			struct giu_gpio_queue  *queue = &gpio->outtcs[tc_idx].outqs[q_idx].queue;

			/* Set out Q info */
			/* TODO: done now we assume out q and done q has the same attributes */
			q_info->offset = done_q_info->offset = queue->payload_offset;
			q_info->size = done_q_info->size = queue->desc_total - 1;
		}
	}

	return 0;
}

int giu_gpio_create_event(struct giu_gpio *gpio, struct giu_gpio_event_params *params, struct mv_sys_event **ev)
{
	if (unlikely(!gpio)) {
		pr_err("Invalid GPIO handle!\n");
		return -EINVAL;
	}

	return gie_create_event(giu_get_gie_handle(gpio->giu, GIU_ENG_IN),
		(struct gie_event_params *)params, ev);
}

int giu_gpio_delete_event(struct mv_sys_event *ev)
{
	return gie_delete_event(ev);
}

int giu_gpio_set_event(struct mv_sys_event *ev, int en)
{
	return gie_set_event(ev, en);
}

int giu_gpio_get_statistics(struct giu_gpio *gpio, struct giu_gpio_statistics *stats, int reset)
{
	u32 tc_idx, q_idx;
	int rc;
	struct giu_gpio_q_statistics q_stats;

	pr_debug("get statistics\n");
	memset(stats, 0, sizeof(struct giu_gpio_statistics));

	for (tc_idx = 0; tc_idx < gpio->num_intcs; tc_idx++)
		for (q_idx = 0; q_idx < gpio->intcs[tc_idx].num_inqs; q_idx++) {
			rc = giu_gpio_get_q_statistics(gpio, 0, 0, tc_idx, q_idx, &q_stats, reset);
			if (rc) {
				pr_err("failed to get statistics on tc:%d queue:%d\n", tc_idx, q_idx);
				return rc;
			}
			stats->in_packets += q_stats.packets;
		}

	for (tc_idx = 0; tc_idx < gpio->num_outtcs; tc_idx++)
		for (q_idx = 0; q_idx < gpio->outtcs[tc_idx].num_outqs; q_idx++) {
			rc = giu_gpio_get_q_statistics(gpio, 1, 0, tc_idx, q_idx, &q_stats, reset);
			if (rc) {
				pr_err("failed to get statistics on tc:%d queue:%d\n", tc_idx, q_idx);
				return rc;
			}
			stats->out_packets += q_stats.packets;
		}

	return 0;
}

int giu_gpio_get_q_statistics(struct giu_gpio *gpio, int out, int rem, u8 tc, u8  qid,
				struct giu_gpio_q_statistics *stats, int reset)
{
	int ret;

	if (rem) {
		if (out) {
			if (tc > gpio->num_intcs) {
				pr_err("out of range intc. no such intc: %d\n", tc);
				return -1;
			}
			if (qid > gpio->intcs[tc].num_rem_outqs) {
				pr_err("out of range intc remote out queue. no such qid: %d\n", qid);
				return -1;
			}
			ret = gie_get_queue_stats(giu_get_gie_handle(gpio->giu, GIU_ENG_IN),
					gpio->intcs[tc].rem_outqs[qid].q_id, &stats->packets, reset);
			if (ret) {
				pr_err("failed to get stats: remote out tc:%d q:%d (qid:%d)\n",
					tc, qid, gpio->intcs[tc].rem_outqs[qid].q_id);
				return ret;
			}
			pr_debug("gpio queue stats: remote out tc:%d q:%d pkt_cnt:%lu\n", tc, qid, stats->packets);

		} else {
			if (tc > gpio->num_outtcs) {
				pr_err("out of range outtc. no such intc: %d\n", tc);
				return -1;
			}
			if (qid > gpio->outtcs[tc].num_rem_inqs) {
				pr_err("out of range outtc remote in queue. no such qid: %d\n", qid);
				return -1;
			}
			ret = gie_get_queue_stats(giu_get_gie_handle(gpio->giu, GIU_ENG_OUT),
					gpio->outtcs[tc].rem_inqs[qid].q.q_id, &stats->packets, reset);
			if (ret) {
				pr_err("failed to get stats: remote in tc:%d q:%d (qid:%d)\n",
					tc, qid, gpio->outtcs[tc].rem_inqs[qid].q.q_id);
				return ret;
			}
			pr_debug("gpio queue stats: remote in tc:%d q:%d pkt_cnt:%lu\n", tc, qid, stats->packets);
		}
	} else {
		if (out) {
			if (tc > gpio->num_outtcs) {
				pr_err("out of range outtc. no such outtc: %d\n", tc);
				return -1;
			}
			if (qid > gpio->outtcs[tc].num_outqs) {
				pr_err("out of range outtc local out queue. no such qid: %d\n", qid);
				return -1;
			}
			ret = gie_get_queue_stats(giu_get_gie_handle(gpio->giu, GIU_ENG_OUT),
					gpio->outtcs[tc].outqs[qid].q_id, &stats->packets, reset);
			if (ret) {
				pr_err("failed to get stats: local out tc:%d q:%d (qid:%d)\n",
					tc, qid, gpio->outtcs[tc].outqs[qid].q_id);
				return ret;
			}
			pr_debug("gpio queue stats: local out tc:%d q:%d pkt_cnt:%lu\n", tc, qid, stats->packets);

		} else {
			if (tc > gpio->num_intcs) {
				pr_err("out of range intc. no such intc: %d\n", tc);
				return -1;
			}
			if (qid > gpio->intcs[tc].num_inqs) {
				pr_err("out of range intc local in queue. no such qid: %d\n", qid);
				return -1;
			}
			ret = gie_get_queue_stats(giu_get_gie_handle(gpio->giu, GIU_ENG_IN),
					gpio->intcs[tc].inqs[qid].q_id, &stats->packets, reset);
			if (ret) {
				pr_err("failed to get stats: local in tc:%d q:%d (qid:%d)\n",
					tc, qid, gpio->intcs[tc].inqs[qid].q_id);
				return ret;
			}
			pr_debug("gpio queue stats: local in tc:%d q:%d pkt_cnt:%lu\n", tc, qid, stats->packets);
		}
	}

	return 0;
}

inline int giu_gpio_outq_desc_set_format(struct giu_gpio_desc *desc, enum giu_format format, uint8_t num_sg_ent)
{
	GIU_TXD_SET_FORMAT(desc, format);
	if (format == GIU_FORMAT_DIRECT_SG) {
		if ((num_sg_ent < 2) || (num_sg_ent > GIU_GPIO_MAX_SG_SEGMENTS)) {
			pr_err("'num_sg_ent' must be between 2-%d\n", GIU_GPIO_MAX_SG_SEGMENTS);
			return -EINVAL;
		}
		GIU_TXD_SET_NUM_SG_ENT(desc, (num_sg_ent - 2));
	}
	if (format == GIU_FORMAT_INDIRECT_SG) {
		pr_err("indirect s/g packet is not supported\n");
		return -ENOTSUP;
	}

	return 0;
}
