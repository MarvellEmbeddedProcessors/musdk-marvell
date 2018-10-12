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
#include "drivers/giu_regfile_def.h"
#include "hw_emul/gie.h"

#include "lib/lib_misc.h"
#include "lib/net.h"

#include "giu_queue_topology.h"
#include "giu_internal.h"
#include "crc.h"


#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
struct giu_gpio_shadow_desc {
	u8	queue_idx;
	u16	queue_desc_idx;
};

struct giu_gpio_shadow_queue {
	u32		desc_total; /**< number of descriptors in the ring */
	struct giu_gpio_shadow_desc	*desc_ring_base; /**< descriptor ring virtual address */
	u32	prod_val; /**< producer index value shadow  */
	u32	cons_val; /**< consumer index value shadow */
};
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS */

/**
 * GPIO Handler
 */
struct giu_gpio {
	u32			id; /**< GPIO Id */
	u32			giu_id;	/**< GIU's Id */
	struct giu_gpio_probe_params	*probe_params; /**< q topology parameters */
	struct giu_gpio_params	*params; /**< q topology initialization parameters */
#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
	struct giu_gpio_shadow_queue shadow_queue[GIU_MAX_NUM_TC];
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS */
};


static struct giu_gpio gpio_array[GIU_MAX_NUM_GPIO];


static int free_tc_queues(struct mqa *mqa, struct gie *gie,
	union giu_gpio_q_params *giu_gpio_q_p, u32 queue_num, u32 queue_type)
{
	u32 q_idx;
	u32 q_num;
	int ret;

	for (q_idx = 0; q_idx < queue_num; q_idx++) {

		(queue_type == GIU_LCL_Q_IND) ?
			(q_num = giu_gpio_q_p->lcl_q.q_id) :
			(q_num = giu_gpio_q_p->rem_q.q_id);

		mqa_queue_free(mqa, q_num);

		/* If needed, unregister the queue from GIU polling */
		if (gie) {
			ret = gie_remove_queue(gie, q_num);
			if (ret)
				pr_err("Failed to remove queue Idx %x from GIU TX\n", q_num);
		}
	}

	return 0;
}


int giu_gpio_init(struct giu_gpio_params *params, struct giu_gpio **gpio)
{
	int ret;
	u32 tc_idx;
	u32 q_idx;
	s32 pair_qid;
	u32 bm_pool_num;
	struct mqa_queue_params mqa_params;
	struct giu_gpio_outtc_params *outtc, *outtc_pair;
	struct giu_gpio_intc_params *intc, *intc_pair;
	union giu_gpio_q_params *giu_gpio_q_p;

	*gpio = kcalloc(1, sizeof(struct giu_gpio), GFP_KERNEL);
	if (*gpio == NULL) {
		pr_err("Failed to allocate GIU GPIO handler\n");
		goto error;
	}

	(*gpio)->params = params;

	/* Create Local Egress TC queues */
	pr_debug("Initializing Local Egress TC queues (TC Num %d)\n",
			params->intcs_params.num_intcs);

	for (tc_idx = 0; tc_idx < params->intcs_params.num_intcs; tc_idx++) {

		intc = &(params->intcs_params.intc_params[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++) {

			memset(&mqa_params, 0, sizeof(struct mqa_queue_params));

			mqa_params.idx  = intc->inqs_params[q_idx].lcl_q.q_id;
			mqa_params.len  = intc->inqs_params[q_idx].lcl_q.len;
			mqa_params.size = gie_get_desc_size(TX_DESC);
			mqa_params.attr = MQA_QUEUE_LOCAL | MQA_QUEUE_EGRESS;

			for (bm_pool_num = 0; bm_pool_num < intc->num_inpools; bm_pool_num++) {
				if (intc->pools[bm_pool_num].lcl_q.q_id != 0xFFFF)
					mqa_params.bpool_qids[bm_pool_num] = intc->pools[bm_pool_num].lcl_q.q_id;
				else
					break;
			}
			mqa_params.bpool_num = bm_pool_num;
			/* For Egress, the destination Q is the local Q (NIC)
			 * and the MSI-X id should be registered in MQA dest Q.
			 */
			mqa_params.msix_id = intc->rem_outqs_params[q_idx].rem_q.msix_id;

			giu_gpio_q_p = &(intc->inqs_params[q_idx]);
			if (giu_gpio_q_p) {
				ret = mqa_queue_create(params->mqa, &mqa_params, &(giu_gpio_q_p->lcl_q.q));
				if (ret < 0) {
					pr_err("Failed to allocate local egress queue %d\n", mqa_params.idx);
					goto lcl_eg_queue_error;
				}
			}
		}
	}

	/* Create Local Ingress TC queues */
	pr_debug("Initializing Local Ingress TC queues (TC Num %d)\n",
			params->outtcs_params.num_outtcs);

	for (tc_idx = 0; tc_idx < params->outtcs_params.num_outtcs; tc_idx++) {

		outtc = &(params->outtcs_params.outtc_params[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {

			memset(&mqa_params, 0, sizeof(struct mqa_queue_params));

			mqa_params.idx  = outtc->outqs_params[q_idx].lcl_q.q_id;
			mqa_params.len  = outtc->outqs_params[q_idx].lcl_q.len;
			mqa_params.size = gie_get_desc_size(RX_DESC);
			mqa_params.attr = MQA_QUEUE_LOCAL | MQA_QUEUE_INGRESS;
			mqa_params.copy_payload = 1;

			giu_gpio_q_p = &(outtc->outqs_params[q_idx]);
			if (giu_gpio_q_p) {
				ret = mqa_queue_create(params->mqa, &mqa_params, &(giu_gpio_q_p->lcl_q.q));
				if (ret < 0) {
					pr_err("Failed to allocate local ingress queue %d\n", mqa_params.idx);
					goto lcl_ing_queue_error;
				}
			}
		}
	}

	/* Create Host Ingress TC queues */
	pr_info("Initializing Host Ingress TC queues (TC Num %d)\n",
			params->outtcs_params.num_outtcs);

	for (tc_idx = 0; tc_idx < params->outtcs_params.num_outtcs; tc_idx++) {

		outtc = &(params->outtcs_params.outtc_params[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_rem_inqs; q_idx++) {

			memset(&mqa_params, 0, sizeof(struct mqa_queue_params));

			mqa_params.idx  = outtc->rem_inqs_params[q_idx].rem_q.q_id;
			mqa_params.len  = outtc->rem_inqs_params[q_idx].rem_q.len;
			mqa_params.size = outtc->rem_inqs_params[q_idx].rem_q.size;
			mqa_params.attr = MQA_QUEUE_REMOTE | MQA_QUEUE_INGRESS;
			mqa_params.prio = tc_idx;
			mqa_params.remote_phy_addr =
				(void *)(uintptr_t)outtc->rem_inqs_params[q_idx].rem_q.q_base_pa;
			mqa_params.prod_phys       =
				(void *)(uintptr_t)outtc->rem_inqs_params[q_idx].rem_q.prod_base_pa;
			mqa_params.prod_virt       = outtc->rem_inqs_params[q_idx].rem_q.prod_base_va;
			mqa_params.host_remap      = outtc->rem_inqs_params[q_idx].rem_q.host_remap;
			mqa_params.msix_id	       = outtc->rem_inqs_params[q_idx].rem_q.msix_id;
			mqa_params.bpool_num       = 1;
			mqa_params.bpool_qids[0]   = outtc->rem_poolqs_params[q_idx].rem_q.q_id;

			giu_gpio_q_p = &(outtc->rem_inqs_params[q_idx]);
			if (giu_gpio_q_p) {
				outtc_pair = &(params->outtcs_params.outtc_params[mqa_params.prio]);
				pair_qid = outtc_pair->outqs_params[0].lcl_q.q_id;
#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
				pair_qid = outtc_pair->outqs_params[q_idx].lcl_q.q_id;
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS*/

				ret = mqa_queue_create(params->mqa, &mqa_params, &(giu_gpio_q_p->rem_q.q));
				if (ret < 0) {
					pr_err("Failed to allocate queue for Host BM\n");
					goto host_ing_queue_error;
				}

				ret = mqa_queue_associate_pair(params->mqa, pair_qid, giu_gpio_q_p->rem_q.q_id);
				if (ret) {
					pr_err("Failed to associate remote egress Queue %d\n",
						   giu_gpio_q_p->rem_q.q_id);
					goto host_ing_queue_error;
				}

				ret = gie_add_queue(giu_get_gie_handle(params->giu, GIU_ENG_OUT),
						pair_qid, GIU_LCL_Q_IND);
				if (ret) {
					pr_err("Failed to register Host Egress Queue %d to GIU\n",
						   giu_gpio_q_p->rem_q.q_id);
					goto host_ing_queue_error;
				}
			}
		}
	}

	/* Create Host Egress TC queues */
	pr_info("Initializing Host Egress TC queues (TC Num %d)\n",
			params->intcs_params.num_intcs);

	for (tc_idx = 0; tc_idx < params->intcs_params.num_intcs; tc_idx++) {

		intc = &(params->intcs_params.intc_params[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_rem_outqs; q_idx++) {

			giu_gpio_q_p = &(intc->rem_outqs_params[q_idx]);
			if (giu_gpio_q_p) {
				memset(&mqa_params, 0, sizeof(struct mqa_queue_params));

				mqa_params.idx  = intc->rem_outqs_params[q_idx].rem_q.q_id;
				mqa_params.len  = intc->rem_outqs_params[q_idx].rem_q.len;
				mqa_params.size = intc->rem_outqs_params[q_idx].rem_q.size;
				mqa_params.attr = MQA_QUEUE_REMOTE | MQA_QUEUE_EGRESS;
				mqa_params.prio = tc_idx;
				mqa_params.remote_phy_addr =
					(void *)(uintptr_t)intc->rem_outqs_params[q_idx].rem_q.q_base_pa;
				mqa_params.cons_phys       =
					(void *)(uintptr_t)intc->rem_outqs_params[q_idx].rem_q.cons_base_pa;
				mqa_params.cons_virt       = intc->rem_outqs_params[q_idx].rem_q.cons_base_va;
				mqa_params.host_remap      = intc->rem_outqs_params[q_idx].rem_q.host_remap;
				mqa_params.msix_id	       = intc->rem_outqs_params[q_idx].rem_q.msix_id;
				mqa_params.copy_payload    = 1;

				intc_pair = &(params->intcs_params.intc_params[mqa_params.prio]);
				pair_qid = intc_pair->inqs_params[0].lcl_q.q_id;
#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
				pair_qid = intc_pair->inqs_params[q_idx].lcl_q.q_id;
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS*/

				ret = mqa_queue_create(params->mqa, &mqa_params, &(giu_gpio_q_p->rem_q.q));
				if (ret < 0) {
					pr_err("Failed to allocate queue for Host BM\n");
					goto host_eg_queue_error;
				}

				ret = mqa_queue_associate_pair(params->mqa, giu_gpio_q_p->rem_q.q_id, pair_qid);
				if (ret) {
					pr_err("Failed to associate remote egress Queue %d\n",
						   giu_gpio_q_p->rem_q.q_id);
					goto host_eg_queue_error;
				}

				/* Register Host Egress Queue to GIU */
				ret = gie_add_queue(giu_get_gie_handle(params->giu, GIU_ENG_IN),
						giu_gpio_q_p->rem_q.q_id, GIU_REM_Q_IND);
				if (ret) {
					pr_err("Failed to register Host Egress Queue %d to GIU\n",
						   giu_gpio_q_p->rem_q.q_id);
					goto host_eg_queue_error;
				}
			}
		}
	}

	return 0;

host_eg_queue_error:

	for (tc_idx = 0; tc_idx < params->intcs_params.num_intcs; tc_idx++) {
		intc = &(params->intcs_params.intc_params[tc_idx]);

		/* Free queue resources and registrations.
		 * for Egress, also un-register from Tx GIU.
		 */
		ret = free_tc_queues(params->mqa, giu_get_gie_handle(params->giu, GIU_ENG_IN),
				intc->rem_outqs_params, intc->num_rem_outqs, GIU_REM_Q_IND);
		if (ret)
			pr_err("Failed to free TC %d queues\n", tc_idx);
	}

host_ing_queue_error:

	for (tc_idx = 0; tc_idx < params->outtcs_params.num_outtcs; tc_idx++) {
		outtc = &(params->outtcs_params.outtc_params[tc_idx]);

		/* Free queue resources and registrations.
		 * No need to unregister the Q from GIU as it's done on local side.
		 */
		ret = free_tc_queues(params->mqa, NULL,
				outtc->rem_inqs_params, outtc->num_rem_inqs, GIU_REM_Q_IND);
		if (ret)
			pr_err("Failed to free TC %d queues\n", tc_idx);
	}

lcl_ing_queue_error:

	for (tc_idx = 0; tc_idx < params->outtcs_params.num_outtcs; tc_idx++) {
		outtc = &(params->outtcs_params.outtc_params[tc_idx]);

		/* Free queue resources and registrations.
		 * for Ingress, also un-register from Rx GIU.
		 */
		ret = free_tc_queues(params->mqa, giu_get_gie_handle(params->giu, GIU_ENG_OUT),
				outtc->outqs_params, outtc->num_outqs, GIU_LCL_Q_IND);
		if (ret)
			pr_err("Failed to free TC %d queues\n", tc_idx);
	}

lcl_eg_queue_error:

	for (tc_idx = 0; tc_idx < params->intcs_params.num_intcs; tc_idx++) {
		intc = &(params->intcs_params.intc_params[tc_idx]);

		/* Free queue resources and registrations.
		 * No need to unregister the Q from GIU as it was done on remote side.
		 */
		ret = free_tc_queues(params->mqa, NULL,
				intc->inqs_params, intc->num_inqs, GIU_LCL_Q_IND);
		if (ret)
			pr_err("Failed to free TC %d queues\n", tc_idx);
	}

error:

	return -1;
}


void giu_gpio_deinit(struct giu_gpio *gpio)
{
	int ret;
	u32 tc_idx;
	u32 q_idx;
	struct giu_gpio_outtc_params *outtc;
	struct giu_gpio_intc_params *intc;
	union giu_gpio_q_params *giu_gpio_q_p;
	struct giu_gpio_params *params = (struct giu_gpio_params *)(gpio->probe_params);

	pr_debug("De-initializing Host Egress TC queues\n");

	for (tc_idx = 0; tc_idx < params->intcs_params.num_intcs; tc_idx++) {
		intc = &(params->intcs_params.intc_params[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_rem_outqs; q_idx++) {
			giu_gpio_q_p = &(intc->rem_outqs_params[q_idx]);
			if (giu_gpio_q_p) {
				ret = giu_destroy_q(params->giu, GIU_ENG_IN, params->mqa,
						giu_gpio_q_p->rem_q.q, HOST_EGRESS_DATA_QUEUE);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", giu_gpio_q_p->rem_q.q_id);
			}
		}
	}

	pr_debug("De-initializing Host Ingress TC queues\n");

	for (tc_idx = 0; tc_idx < params->outtcs_params.num_outtcs; tc_idx++) {
		outtc = &(params->outtcs_params.outtc_params[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_rem_inqs; q_idx++) {
			giu_gpio_q_p = &(outtc->rem_inqs_params[q_idx]);
			if (giu_gpio_q_p) {
				ret = giu_destroy_q(params->giu, GIU_ENG_OUT_OF_RANGE, params->mqa,
						giu_gpio_q_p->rem_q.q, HOST_INGRESS_DATA_QUEUE);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", giu_gpio_q_p->rem_q.q_id);
			}
		}
	}

	pr_debug("De-initializing Local Ingress TC queues\n");

	for (tc_idx = 0; tc_idx < params->outtcs_params.num_outtcs; tc_idx++) {
		outtc = &(params->outtcs_params.outtc_params[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {
			giu_gpio_q_p = &(outtc->outqs_params[q_idx]);
			if (giu_gpio_q_p) {
				ret = giu_destroy_q(params->giu, GIU_ENG_OUT, params->mqa,
						giu_gpio_q_p->lcl_q.q, LOCAL_INGRESS_DATA_QUEUE);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", giu_gpio_q_p->lcl_q.q_id);
			}
		}
	}

	pr_debug("De-initializing Local Egress TC queues\n");

	for (tc_idx = 0; tc_idx < params->intcs_params.num_intcs; tc_idx++) {
		intc = &(params->intcs_params.intc_params[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++) {
			giu_gpio_q_p = &(intc->inqs_params[q_idx]);
			if (giu_gpio_q_p) {
				ret = giu_destroy_q(params->giu, GIU_ENG_OUT_OF_RANGE, params->mqa,
						giu_gpio_q_p->lcl_q.q, LOCAL_EGRESS_DATA_QUEUE);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", giu_gpio_q_p->lcl_q.q_id);
			}
		}
	}

	kfree(gpio);
}

int giu_gpio_probe(char *match, char *regfile_name, struct giu_gpio **gpio)
{
	u8 match_params[2];
	int giu_id, gpio_id;
	int tc, ret;

	if (mv_sys_match(match, "gpio", 2, match_params))
		return(-ENXIO);

	giu_id = match_params[0];
	gpio_id = match_params[1];

	pr_debug("Init gpio %d for giu id: %d.\n", gpio_id, giu_id);

	if (gpio_id >= GIU_MAX_NUM_GPIO)	{
		pr_err("giu_id (%d) exceeds mac gpio number (%d)\n", giu_id, GIU_MAX_NUM_GPIO);
		return -1;
	}

	/* Signal upper level that initialization is completed */
	ret = giu_gpio_topology_set_init_done(giu_id);
	if (ret)
		return ret;

	gpio_array[giu_id].id = gpio_id;
	gpio_array[giu_id].giu_id = giu_id;
	gpio_array[giu_id].probe_params = giu_gpio_get_topology(giu_id);
	if (gpio_array[giu_id].probe_params == NULL) {
		pr_err("queue topology was not initialized for GIU %d\n", giu_id);
		return -1;
	}

	pr_debug("giu_gpio_probe giu_id %d\n", giu_id);

	*gpio = &gpio_array[giu_id];

#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
	for (tc = 0; tc < (*gpio)->probe_params->outqs_params.num_tcs; tc++) {
		/* we assume that all Qs (in a secific TC) has the same size */
		struct giu_gpio_queue *txq = &(*gpio)->probe_params->outqs_params.tcs[tc].queues[0];

		(*gpio)->shadow_queue[tc].desc_ring_base =
			kzalloc(sizeof(struct giu_gpio_shadow_desc) * txq->desc_total, GFP_KERNEL);
		if ((*gpio)->shadow_queue[tc].desc_ring_base == NULL) {
			pr_err("Failed to allocate GIU GPIO TC %d shadow-queue for RSS\n", tc);
			goto probe_error;
		}

		(*gpio)->shadow_queue[tc].desc_total = txq->desc_total;
		(*gpio)->shadow_queue[tc].prod_val = 0;
		(*gpio)->shadow_queue[tc].cons_val = 0;
	}
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS */

	return 0;

probe_error:
	while (tc--)
		kfree((*gpio)->shadow_queue[tc].desc_ring_base);

	return -1;
}

void giu_gpio_remove(struct giu_gpio *gpio)
{
	int tc;

	pr_err("giu_gpio_remove is not implemented\n");
#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
	for (tc = 0; tc < gpio->probe_params->outqs_params.num_tcs; tc++)
		kfree(gpio->shadow_queue[tc].desc_ring_base);
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS */
}

int giu_gpio_enable(struct giu_gpio *gpio)
{
	pr_err("giu_gpio_enable is not implemented\n");
	return 0;
}

int giu_gpio_disable(struct giu_gpio *gpio)
{
	pr_err("giu_gpio_disable is not implemented\n");
	return 0;
}

static inline u64 giu_gpio_outq_desc_get_phys_addr(struct giu_gpio_desc *desc)
{
	/* cmd[4] and cmd[5] holds the buffer physical address (Low and High parts) */
	return ((u64)(desc->cmds[5] & GIU_TXD_BUF_PHYS_HI_MASK) << 32) | (u64)desc->cmds[4];
}

static inline void *giu_gpio_outq_desc_get_addr(struct giu_gpio_desc *desc)
{
	return mv_sys_dma_mem_phys2virt(giu_gpio_outq_desc_get_phys_addr(desc));
}

#define MAX_EXTRACTION_SIZE	(MV_IPV6ADDR_LEN * 2 + MV_IP_PROTO_NH_LEN + MV_L4_PORT_LEN * 2)

static int giu_gpio_update_rss(struct giu_gpio *gpio, u8 tc, struct giu_gpio_desc *desc)
{
	struct giu_gpio_probe_params *gpio_probe_params = gpio->probe_params;
	struct giu_gpio_tc *gpio_tc = &gpio_probe_params->outqs_params.tcs[tc];
	uint8_t key_size = 0, *extracted_key;
	const uint8_t *ip_frame;
	uint64_t crc64 = CRC64_DEFAULT_INITVAL;
	enum giu_outq_l3_type l3_info = GIU_TXD_GET_L3_PRS_INFO(desc);
	enum giu_outq_l4_type l4_info = GIU_TXD_GET_L4_PRS_INFO(desc);
	int ipv4 = 0;
	u16 queue_index;

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

	if ((gpio_tc->hash_type == GIU_GPIO_HASH_T_5_TUPLE) &&
		(l4_info == GIU_OUTQ_L4_TYPE_TCP || l4_info == GIU_OUTQ_L4_TYPE_UDP)) {
		if (ipv4)
			extracted_key = &ipv4hdr->proto;
		else
			extracted_key = &ipv6hdr->next_header;
		crc64 = crc64_compute(extracted_key, MV_IP_PROTO_NH_LEN, crc64);

		struct mv_udphdr *l4hdr = (struct mv_udphdr *)(ip_frame + (GIU_TXD_GET_IPHDR_LEN(desc) * 4));
		crc64 = crc64_compute((uint8_t *)&l4hdr->src_port, (MV_L4_PORT_LEN * 2), crc64);
	}

	queue_index = crc64 % gpio_tc->dest_num_qs;
	GIU_TXD_SET_DEST_QID(desc, queue_index);

	return 0;
}

#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
static int giu_gpio_send_multi_q(struct giu_gpio *gpio, u8 tc, struct giu_gpio_desc *descs, u16 *num)
{
	struct giu_gpio_shadow_queue *txq_shadow = &gpio->shadow_queue[tc];
	struct giu_gpio_tc *gpio_tc = &gpio->probe_params->outqs_params.tcs[tc];
	struct giu_gpio_queue *txq;
	struct giu_gpio_desc *tx_ring_base;
	u32 prod_val[gpio_tc->num_qs];
	u16 dest_qid, num_txds = *num;
	u32 free_count;
	int i;

	/* Calculate number of free descriptors */
	free_count = QUEUE_SPACE(txq_shadow->prod_val, txq_shadow->cons_val, txq_shadow->desc_total);

	if (unlikely(free_count < num_txds)) {
		pr_debug("num_txds(%d), free_count(%d) (GPIO %d)\n", num_txds, free_count, gpio->id);
		num_txds = free_count;
	}

	if (unlikely(!num_txds)) {
		pr_debug("GPIO full\n");
		*num = 0;
		return 0;
	}

	for (i = 0; i < gpio_tc->num_qs; i++)
		prod_val[i] = readl_relaxed(gpio_tc->queues[i].prod_addr);

	/* Calculate RSS and update descriptor */
	for (i = 0; i < num_txds; i++) {
		giu_gpio_update_rss(gpio, tc, &descs[i]);
		dest_qid = GIU_TXD_GET_DEST_QID(&descs[i]);
		/* Get queue params */
		txq = &gpio_tc->queues[dest_qid];

		/* Update shadow-queue with dest-qid and index */
		txq_shadow->desc_ring_base[txq_shadow->prod_val].queue_idx = dest_qid;
		txq_shadow->desc_ring_base[txq_shadow->prod_val].queue_desc_idx = prod_val[dest_qid];
		/* Increment producer index */
		txq_shadow->prod_val = QUEUE_INDEX_INC(txq_shadow->prod_val, 1, txq_shadow->desc_total);

		/* Get ring base */
		tx_ring_base = (struct giu_gpio_desc *)txq->desc_ring_base;

		/* Copy descriptor to descriptor ring */
		memcpy(&tx_ring_base[prod_val[dest_qid]], &descs[i], sizeof(*tx_ring_base));

		/* Increment producer index, update remaining descriptors count and block size */
		prod_val[dest_qid] = QUEUE_INDEX_INC(prod_val[dest_qid], 1, txq->desc_total);
	}

	/* make sure all writes are done (i.e. descriptor were copied)
	 * before incrementing the producer index
	 */
	wmb();

	for (i = 0; i < gpio_tc->dest_num_qs; i++) {
		txq = &gpio_tc->queues[i];
		/* Update Producer index in GNPT */
		writel_relaxed(prod_val[i], txq->prod_addr);
	}

	/* Update number of sent descriptors */
	*num = num_txds;

	return 0;
}
#endif

int giu_gpio_send(struct giu_gpio *gpio, u8 tc, u8 qid, struct giu_gpio_desc *descs, u16 *num)
{
	struct giu_gpio_queue *txq;
	struct giu_gpio_desc *tx_ring_base;
	struct giu_gpio_probe_params *gpio_probe_params = gpio->probe_params;
	u16 num_txds = *num, desc_remain;
	u16 block_size, index;
	u32 free_count, cons_val, prod_val;
	int i;

#ifdef GIU_GPIO_DEBUG
	/* Check that the requested TC is supported */
	if (tc >= gpio_probe_params.outqs_params.num_tcs) {
		pr_err("GIU GPIO: TC %d is not supported (Max TC is %d)\n", tc, gpio_probe_params.outqs_params.num_tcs);
		return -1;
	}

	/* Check that the requested Q ID exists */
	if (qid >= gpio_probe_params->outqs_params.tcs[tc].num_qs) {
		pr_err("GIU GPIO: Q %d is not supported (Max Q is %d)\n", qid,
			gpio_probe_params->outqs_params.tcs[tc].num_qs);
		return -1;
	}
#endif

#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
	if (gpio_probe_params->outqs_params.tcs[tc].dest_num_qs > 1)
		return giu_gpio_send_multi_q(gpio, tc, descs, num);
#endif
	/* Get queue params */
	txq = &gpio_probe_params->outqs_params.tcs[tc].queues[qid];

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
		if (gpio_probe_params->outqs_params.tcs[tc].dest_num_qs > 1) {
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
#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
static inline int giu_gpio_get_num_outq_done_internal(struct giu_gpio *gpio, u8 tc, u8 qid, u16 *num)
#else
int giu_gpio_get_num_outq_done(struct giu_gpio *gpio, u8 tc, u8 qid, u16 *num)
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS*/
{
	u32 tx_num = 0;
	u32 cons_val;
	struct giu_gpio_probe_params *gpio_probe_params = gpio->probe_params;
	struct giu_gpio_queue *txq = &gpio_probe_params->outqs_params.tcs[tc].queues[qid];

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

#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
/* created a shadow-q with the same size as tx-queue which will holds
 * for each sent frame its 'tx-queue-index' (as a result of RSS) and its
 * entry index in this tx-queue.
 * once application request for the number of transmitted frames, we go
 * over on all occupied entries in the shadow-q and using the saved
 * information check if it was transmitted or not. once we reach
 * an un-transmitted frame we stop and return the number of transmitted
 * frames. as the shadow-q holds the frames in the same order as the
 * application sent them, it is guarantee that only transmitted buffers
 * will be released to the pp2-pool.
 */
int giu_gpio_get_num_outq_done(struct giu_gpio *gpio, u8 tc, u8 qid, u16 *num)
{
	struct giu_gpio_shadow_queue *txq_shadow = &gpio->shadow_queue[tc];
	struct giu_gpio_shadow_desc *desc;
	struct giu_gpio_tc *gpio_tc = &gpio->probe_params->outqs_params.tcs[tc];
	u16 shadow_q_tx_num;
	u32 cons_val[gpio->probe_params->outqs_params.tcs[tc].num_qs];
	u8 i;

	if (gpio_tc->dest_num_qs == 1)
		return giu_gpio_get_num_outq_done_internal(gpio, tc, 0, num);

	*num = 0;
	shadow_q_tx_num = QUEUE_OCCUPANCY(txq_shadow->prod_val, txq_shadow->cons_val, txq_shadow->desc_total);
	if (!shadow_q_tx_num)
		return 0;

	for (i = 0; i < gpio_tc->num_qs; i++)
		cons_val[i] = readl_relaxed(gpio_tc->queues[i].cons_addr);

	for (i = 0; i < shadow_q_tx_num; i++) {
		desc = &txq_shadow->desc_ring_base[txq_shadow->cons_val];
		if (QUEUE_OCCUPANCY(cons_val[desc->queue_idx], desc->queue_desc_idx, txq_shadow->desc_total) == 0)
			break;
		txq_shadow->cons_val = QUEUE_INDEX_INC(txq_shadow->cons_val, 1, txq_shadow->desc_total);
		(*num)++;
	}

	return 0;
}
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS */

#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
static inline int giu_gpio_recv_internal(struct giu_gpio *gpio, u8 tc, u8 qid, struct giu_gpio_desc *descs, u16 *num)
#else
int giu_gpio_recv(struct giu_gpio *gpio, u8 tc, u8 qid, struct giu_gpio_desc *descs, u16 *num)
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS */
{
	struct giu_gpio_queue *rxq;
	struct giu_gpio_desc *rx_ring_base;
	struct giu_gpio_probe_params *gpio_probe_params = gpio->probe_params;
	u16 recv_req = *num, desc_received, desc_remain = 0;
	u16 block_size, index;
	u32 prod_val, cons_val;

	*num = 0;
#ifdef GIU_GPIO_DEBUG
	/* Check that the requested TC is supported */
	if (tc >= gpio_probe_params->inqs_params.num_tcs) {
		pr_err("GIU GPIO: TC %d is not supported (Max TC is %d)\n", tc,
			gpio_probe_params->outqs_params.num_tcs);
		return -1;
	}

	/* Check that the requested Q ID exists */
	if (qid >= gpio_probe_params->inqs_params.tcs[tc].num_qs) {
		pr_err("GIU GPIO: Q %d is not supported (Max Q is %d)\n", qid,
			gpio_probe_params->nqs_params.tcs[tc].num_qs);
		return -1;
	}
#endif

	rxq = &gpio_probe_params->inqs_params.tcs[tc].queues[qid];

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

#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
int giu_gpio_recv(struct giu_gpio *gpio, u8 tc, u8 qid, struct giu_gpio_desc *descs, u16 *num)
{
	static u8 curr_qid[GIU_MAX_NUM_TC];
	u8 i;
	u16 recv_req = *num, total_got = 0;
	struct giu_gpio_probe_params *gpio_probe_params = gpio->probe_params;

	for (i = 0; (i < gpio_probe_params->inqs_params.tcs[tc].num_qs) && (total_got != recv_req); i++) {
		*num = recv_req - total_got;
		giu_gpio_recv_internal(gpio, tc, curr_qid[tc], &descs[total_got], num);
		total_got += *num;
		curr_qid[tc]++;
		if (curr_qid[tc] == gpio_probe_params->inqs_params.tcs[tc].num_qs)
			curr_qid[tc] = 0;
	}

	*num = total_got;

	return 0;
}
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS */

int giu_gpio_get_capabilities(struct giu_gpio *gpio, struct giu_gpio_capabilities *capa)
{
	struct giu_gpio_probe_params *gpio_params = gpio->probe_params;
	int tc_idx, q_idx, bpool_id;

	/* Set ID */
	capa->id = gpio->giu_id;

	/* Set number if ingress TCs */
	capa->intcs_inf.num_intcs = gpio_params->inqs_params.num_tcs;

	/* Set ingress TCs info */
	for (tc_idx = 0; tc_idx < capa->intcs_inf.num_intcs; tc_idx++) {
		struct giu_gpio_intc_info *tc_info = &capa->intcs_inf.intcs_inf[tc_idx];
		int qs_num = gpio_params->inqs_params.tcs[tc_idx].num_qs;
		/* Set number if ingress Qs in this TC */
		tc_info->num_inqs = qs_num;

		/* Set ingress Qs info */
		for (q_idx = 0; q_idx < qs_num; q_idx++) {
			struct giu_gpio_q_info *q_info = &capa->intcs_inf.intcs_inf[tc_idx].inqs_inf[q_idx];
			struct giu_gpio_queue  *queue = &gpio_params->inqs_params.tcs[tc_idx].queues[q_idx];

			q_info->offset = queue->payload_offset;
			q_info->size = queue->desc_total - 1;
		}

		/* Set BPool handlers */
		/* TODO: BP ID should be taken from the regfile */
		/* TODO: for now we have a single BP per GPIO */
		tc_info->pools[0] = &giu_bpools[0];
		for (bpool_id = 1; bpool_id < qs_num; bpool_id++)
			tc_info->pools[bpool_id] = NULL;
	}

	/* Set number if egress TCs */
	capa->outtcs_inf.num_outtcs = gpio_params->outqs_params.num_tcs;

	/* Set egress TCs info */
	for (tc_idx = 0; tc_idx < capa->outtcs_inf.num_outtcs; tc_idx++) {
		struct giu_gpio_outtc_info *tc_info = &capa->outtcs_inf.outtcs_inf[tc_idx];
		int qs_num = gpio_params->outqs_params.tcs[tc_idx].num_qs;
		/* Set number if egress Qs in this TC */
		tc_info->num_outqs = qs_num;

		/* Set egress Qs info */
		for (q_idx = 0; q_idx < qs_num; q_idx++) {
			struct giu_gpio_q_info *q_info = &capa->outtcs_inf.outtcs_inf[tc_idx].outqs_inf[q_idx];
			struct giu_gpio_q_info *done_q_info = &capa->outtcs_inf.outtcs_inf[tc_idx].doneqs_inf[q_idx];
			struct giu_gpio_queue  *queue = &gpio_params->outqs_params.tcs[tc_idx].queues[q_idx];

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

	return gie_create_event(giu_get_gie_handle(gpio->params->giu, GIU_ENG_IN),
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
	struct giu_gpio_params *params;
	struct giu_gpio_intc_params *intc;
	struct giu_gpio_outtc_params *outtc;
	struct giu_gpio_q_statistics q_stats;

	params = (struct giu_gpio_params *)(gpio->params);

	pr_debug("get statistics\n");
	memset(stats, 0, sizeof(struct giu_gpio_statistics));

	for (tc_idx = 0; tc_idx < params->intcs_params.num_intcs; tc_idx++) {
		intc = &(params->intcs_params.intc_params[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++) {
			rc = giu_gpio_get_q_statistics(gpio, 0, 0, tc_idx, q_idx, &q_stats, reset);
			if (rc) {
				pr_err("failed to get statistics on tc:%d queue:%d\n", tc_idx, q_idx);
				return rc;
			}
			stats->in_packets += q_stats.packets;
		}
	}

	for (tc_idx = 0; tc_idx < params->outtcs_params.num_outtcs; tc_idx++) {
		outtc = &(params->outtcs_params.outtc_params[tc_idx]);
		for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {
			rc = giu_gpio_get_q_statistics(gpio, 1, 0, tc_idx, q_idx, &q_stats, reset);
			if (rc) {
				pr_err("failed to get statistics on tc:%d queue:%d\n", tc_idx, q_idx);
				return rc;
			}
			stats->out_packets += q_stats.packets;
		}
	}

	return 0;
}

int giu_gpio_get_q_statistics(struct giu_gpio *gpio, int out, int rem, u8 tc, u8  qid,
							  struct giu_gpio_q_statistics *stats, int reset)
{
	union giu_gpio_q_params *giu_gpio_q_p_in, *giu_gpio_q_p_out;
	struct giu_gpio_params *params = (struct giu_gpio_params *)(gpio->params);
	struct giu_gpio_intc_params *intc;
	struct giu_gpio_outtc_params *outtc;
	int ret;

	if (rem) {
		if (out) {
			if (tc > params->intcs_params.num_intcs) {
				pr_err("out of range intc. no such intc: %d\n", tc);
				return -1;
			}
			intc  = &(params->intcs_params.intc_params[tc]);
			if (qid > intc->num_rem_outqs) {
				pr_err("out of range intc remote out queue. no such qid: %d\n", qid);
				return -1;
			}
			giu_gpio_q_p_in  = &(intc->rem_outqs_params[qid]);
			ret = gie_get_queue_stats(giu_get_gie_handle(params->giu, GIU_ENG_IN),
					giu_gpio_q_p_in->rem_q.q_id, &stats->packets, reset);
			if (ret) {
				pr_err("failed to get stats: remote out tc:%d q:%d\n", tc, qid);
				return ret;
			}
			pr_debug("gpio queue stats: remote out tc:%d q:%d pkt_cnt:%lu\n", tc, qid, stats->packets);

		} else {
			if (tc > params->outtcs_params.num_outtcs) {
				pr_err("out of range outtc. no such intc: %d\n", tc);
				return -1;
			}
			outtc = &(params->outtcs_params.outtc_params[tc]);
			if (qid > outtc->num_rem_inqs) {
				pr_err("out of range outtc remote in queue. no such qid: %d\n", qid);
				return -1;
			}
			giu_gpio_q_p_out = &(outtc->outqs_params[qid]);
			ret = gie_get_queue_stats(giu_get_gie_handle(params->giu, GIU_ENG_OUT),
					giu_gpio_q_p_out->rem_q.q_id, &stats->packets, reset);
			if (ret) {
				pr_err("failed to get stats: remote in tc:%d q:%d\n", tc, qid);
				return ret;
			}
			pr_debug("gpio queue stats: remote in tc:%d q:%d pkt_cnt:%lu\n", tc, qid, stats->packets);
		}
	} else {
		if (out) {
			if (tc > params->outtcs_params.num_outtcs) {
				pr_err("out of range outtc. no such outtc: %d\n", tc);
				return -1;
			}
			outtc = &(params->outtcs_params.outtc_params[tc]);
			if (qid > outtc->num_outqs) {
				pr_err("out of range outtc local out queue. no such qid: %d\n", qid);
				return -1;
			}
			giu_gpio_q_p_out = &(outtc->outqs_params[qid]);
			ret = gie_get_queue_stats(giu_get_gie_handle(params->giu, GIU_ENG_OUT),
					giu_gpio_q_p_out->lcl_q.q_id, &stats->packets, reset);
			if (ret) {
				pr_err("failed to get stats: local out tc:%d q:%d\n", tc, qid);
				return ret;
			}
			pr_debug("gpio queue stats: local out tc:%d q:%d pkt_cnt:%lu\n", tc, qid, stats->packets);

		} else {
			if (tc > params->intcs_params.num_intcs) {
				pr_err("out of range intc. no such intc: %d\n", tc);
				return -1;
			}
			intc  = &(params->intcs_params.intc_params[tc]);
			if (qid > intc->num_inqs) {
				pr_err("out of range intc local in queue. no such qid: %d\n", qid);
				return -1;
			}
			giu_gpio_q_p_in  = &(intc->rem_outqs_params[qid]);
			ret = gie_get_queue_stats(giu_get_gie_handle(params->giu, GIU_ENG_IN),
					giu_gpio_q_p_in->lcl_q.q_id, &stats->packets, reset);
			if (ret) {
				pr_err("failed to get stats: local in tc:%d q:%d\n", tc, qid);
				return ret;
			}
			pr_debug("gpio queue stats: local in tc:%d q:%d pkt_cnt:%lu\n", tc, qid, stats->packets);
		}
	}

	return 0;
}
