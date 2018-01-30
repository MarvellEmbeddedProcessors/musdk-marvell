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

#include "std_internal.h"

#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mqa/mqa_internal.h"
#include "drivers/mv_giu_gpio_init.h"
#include "drivers/mv_giu_gpio.h"
#include "drivers/giu_regfile_def.h"
#include "hw_emul/gie.h"
#include "giu_queue_topology.h"
#include "giu_internal.h"
#include "crc.h"

#include "lib/lib_misc.h"
#include "lib/net.h"
#include "hw_emul/gie.h"



/**
 * GPIO Handler
 */
struct giu_gpio {
	u32			id; /**< GPIO Id */
	u32			giu_id;	/**< GIU's Id */
	struct giu_gpio_params	*params; /**< q topology parameters */
	struct giu_gpio_init_params	*init_params; /**< q topology initialization parameters */
};

static struct giu_gpio gpio_array[GIU_MAX_NUM_GPIO];


int giu_gpio_init(struct giu_gpio_init_params *init_params, struct giu_gpio **gpio)
{
	int ret;
	u32 tc_idx;
	u32 q_idx;
	s32 pair_qid;
	u32 bm_pool_num;
	struct mqa_queue_params params;
	struct giu_gpio_outtc_params *outtc, *outtc_pair;
	struct giu_gpio_intc_params *intc, *intc_pair;
	union giu_gpio_q_params *giu_gpio_q_p;

	*gpio = kcalloc(1, sizeof(struct giu_gpio), GFP_KERNEL);
	if (*gpio == NULL) {
		pr_err("Failed to allocate GIU GPIO handler\n");
		goto error;
	}

	(*gpio)->init_params = init_params;

	/* Create Local Egress TC queues */
	pr_debug("Initializing Local Egress TC queues (TC Num %d)\n",
			init_params->intcs_params.num_intcs);

	for (tc_idx = 0; tc_idx < init_params->intcs_params.num_intcs; tc_idx++) {

		intc = &(init_params->intcs_params.intc_params[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++) {

			memset(&params, 0, sizeof(struct mqa_queue_params));

			params.idx  = intc->inqs_params[q_idx].lcl_q.q_id;
			params.len  = intc->inqs_params[q_idx].lcl_q.len;
			params.size = gie_get_desc_size(TX_DESC);
			params.attr = LOCAL_QUEUE | EGRESS_QUEUE;

			for (bm_pool_num = 0; bm_pool_num < intc->num_inpools; bm_pool_num++) {
				if (intc->pools[bm_pool_num].lcl_q.q_id != 0xFFFF)
					params.bpool_qids[bm_pool_num] = intc->pools[bm_pool_num].lcl_q.q_id;
				else
					break;
			}
			params.bpool_num = bm_pool_num;

			giu_gpio_q_p = &(intc->inqs_params[q_idx]);
			if (giu_gpio_q_p) {
				ret = mqa_queue_create(init_params->mqa, &params, &(giu_gpio_q_p->lcl_q.q));
				if (ret < 0) {
					pr_err("Failed to allocate local egress queue %d\n", params.idx);
					goto lcl_eg_queue_error;
				}
			}
		}
	}

	/* Create Local Ingress TC queues */
	pr_debug("Initializing Local Ingress TC queues (TC Num %d)\n",
			init_params->outtcs_params.num_outtcs);

	for (tc_idx = 0; tc_idx < init_params->outtcs_params.num_outtcs; tc_idx++) {

		outtc = &(init_params->outtcs_params.outtc_params[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {

			memset(&params, 0, sizeof(struct mqa_queue_params));

			params.idx  = outtc->outqs_params[q_idx].lcl_q.q_id;
			params.len  = outtc->outqs_params[q_idx].lcl_q.len;
			params.size = gie_get_desc_size(RX_DESC);
			params.attr = LOCAL_QUEUE | INGRESS_QUEUE;
			params.copy_payload = 1;

			giu_gpio_q_p = &(outtc->outqs_params[q_idx]);
			if (giu_gpio_q_p) {
				ret = mqa_queue_create(init_params->mqa, &params, &(giu_gpio_q_p->lcl_q.q));
				if (ret < 0) {
					pr_err("Failed to allocate local ingress queue %d\n", params.idx);
					goto lcl_ing_queue_error;
				}
			}
		}
	}

	/* Create Host Ingress TC queues */
	pr_info("Initializing Host Ingress TC queues (TC Num %d)\n",
			init_params->outtcs_params.num_outtcs);

	for (tc_idx = 0; tc_idx < init_params->outtcs_params.num_outtcs; tc_idx++) {

		outtc = &(init_params->outtcs_params.outtc_params[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_rem_inqs; q_idx++) {

			memset(&params, 0, sizeof(struct mqa_queue_params));

			params.idx  = outtc->rem_inqs_params[q_idx].rem_q.q_id;
			params.len  = outtc->rem_inqs_params[q_idx].rem_q.len;
			params.size = outtc->rem_inqs_params[q_idx].rem_q.size;
			params.attr = REMOTE_QUEUE | INGRESS_QUEUE;
			params.prio = tc_idx;
			params.remote_phy_addr = (void *)outtc->rem_inqs_params[q_idx].rem_q.q_base_pa;
			params.prod_phys       = (void *)outtc->rem_inqs_params[q_idx].rem_q.prod_base_pa;
			params.prod_virt       = outtc->rem_inqs_params[q_idx].rem_q.prod_base_va;
			params.host_remap      = outtc->rem_inqs_params[q_idx].rem_q.host_remap;
			params.bpool_num       = 1;
			params.bpool_qids[0]   = outtc->rem_poolqs_params[q_idx].rem_q.q_id;

			giu_gpio_q_p = &(outtc->rem_inqs_params[q_idx]);
			if (giu_gpio_q_p) {
				outtc_pair = &(init_params->outtcs_params.outtc_params[params.prio]);
				pair_qid = outtc_pair->outqs_params[0].lcl_q.q_id;
#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
				pair_qid = outtc_pair->outqs_params[q_idx].lcl_q.q_id;
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS*/

				ret = mqa_queue_create(init_params->mqa, &params, &(giu_gpio_q_p->rem_q.q));
				if (ret < 0) {
					pr_err("Failed to allocate queue for Host BM\n");
					goto host_ing_queue_error;
				}

				ret = mqa_queue_associate_pair(init_params->mqa, pair_qid, giu_gpio_q_p->rem_q.q_id);
				if (ret) {
					pr_err("Failed to associate remote egress Queue %d\n",
						   giu_gpio_q_p->rem_q.q_id);
					goto host_ing_queue_error;
				}

				ret = gie_add_queue(init_params->gie->rx_gie, pair_qid, GIU_LCL_Q_IND);
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
			init_params->intcs_params.num_intcs);

	for (tc_idx = 0; tc_idx < init_params->intcs_params.num_intcs; tc_idx++) {

		intc = &(init_params->intcs_params.intc_params[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_rem_outqs; q_idx++) {

			giu_gpio_q_p = &(intc->rem_outqs_params[q_idx]);
			if (giu_gpio_q_p) {
				memset(&params, 0, sizeof(struct mqa_queue_params));

				params.idx  = intc->rem_outqs_params[q_idx].rem_q.q_id;
				params.len  = intc->rem_outqs_params[q_idx].rem_q.len;
				params.size = intc->rem_outqs_params[q_idx].rem_q.size;
				params.attr = REMOTE_QUEUE | EGRESS_QUEUE;
				params.prio = tc_idx;
				params.remote_phy_addr = (void *)intc->rem_outqs_params[q_idx].rem_q.q_base_pa;
				params.cons_phys       = (void *)intc->rem_outqs_params[q_idx].rem_q.cons_base_pa;
				params.cons_virt       = intc->rem_outqs_params[q_idx].rem_q.cons_base_va;
				params.host_remap      = intc->rem_outqs_params[q_idx].rem_q.host_remap;
				params.copy_payload    = 1;

				intc_pair = &(init_params->intcs_params.intc_params[params.prio]);
				pair_qid = intc_pair->inqs_params[0].lcl_q.q_id;
#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
				pair_qid = intc_pair->inqs_params[q_idx].lcl_q.q_id;
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS*/

				ret = mqa_queue_create(init_params->mqa, &params, &(giu_gpio_q_p->rem_q.q));
				if (ret < 0) {
					pr_err("Failed to allocate queue for Host BM\n");
					goto host_eg_queue_error;
				}

				ret = mqa_queue_associate_pair(init_params->mqa, giu_gpio_q_p->rem_q.q_id, pair_qid);
				if (ret) {
					pr_err("Failed to associate remote egress Queue %d\n",
						   giu_gpio_q_p->rem_q.q_id);
					goto host_eg_queue_error;
				}

				/* Register Host Egress Queue to GIU */
				ret = gie_add_queue(init_params->gie->tx_gie, giu_gpio_q_p->rem_q.q_id, GIU_REM_Q_IND);
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

	for (tc_idx = 0; tc_idx < init_params->intcs_params.num_intcs; tc_idx++) {
		intc = &(init_params->intcs_params.intc_params[tc_idx]);

		/* Free queue resources and registrations.
		 * for Egress, also un-register from Tx GIU.
		 */
		ret = giu_free_tc_queues(init_params->mqa, intc->rem_outqs_params,
							intc->num_rem_outqs, GIU_REM_Q_IND, init_params->gie->tx_gie);
		if (ret)
			pr_err("Failed to free TC %d queues\n", tc_idx);
	}

host_ing_queue_error:

	for (tc_idx = 0; tc_idx < init_params->outtcs_params.num_outtcs; tc_idx++) {
		outtc = &(init_params->outtcs_params.outtc_params[tc_idx]);

		/* Free queue resources and registrations.
		 * No need to unregister the Q from GIU as it's done on local side.
		 */
		ret = giu_free_tc_queues(init_params->mqa, outtc->rem_inqs_params,
							outtc->num_rem_inqs, GIU_REM_Q_IND, 0);
		if (ret)
			pr_err("Failed to free TC %d queues\n", tc_idx);
	}

lcl_ing_queue_error:

	for (tc_idx = 0; tc_idx < init_params->outtcs_params.num_outtcs; tc_idx++) {
		outtc = &(init_params->outtcs_params.outtc_params[tc_idx]);

		/* Free queue resources and registrations.
		 * for Ingress, also un-register from Rx GIU.
		 */
		ret = giu_free_tc_queues(init_params->mqa, outtc->outqs_params,
							outtc->num_outqs, GIU_LCL_Q_IND, init_params->gie->rx_gie);
		if (ret)
			pr_err("Failed to free TC %d queues\n", tc_idx);
	}

lcl_eg_queue_error:

	for (tc_idx = 0; tc_idx < init_params->intcs_params.num_intcs; tc_idx++) {
		intc = &(init_params->intcs_params.intc_params[tc_idx]);

		/* Free queue resources and registrations.
		 * No need to unregister the Q from GIU as it was done on remote side.
		 */
		ret = giu_free_tc_queues(init_params->mqa, intc->inqs_params,
							intc->num_inqs, GIU_LCL_Q_IND, 0);
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
	struct giu_gpio_init_params *init_params = (struct giu_gpio_init_params *)(gpio->params);

	pr_debug("De-initializing Host Egress TC queues\n");

	for (tc_idx = 0; tc_idx < init_params->intcs_params.num_intcs; tc_idx++) {
		intc = &(init_params->intcs_params.intc_params[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_rem_outqs; q_idx++) {
			giu_gpio_q_p = &(intc->rem_outqs_params[q_idx]);
			if (giu_gpio_q_p) {
				ret = giu_queue_remove(init_params->mqa, giu_gpio_q_p->rem_q.q,
							HOST_EGRESS_DATA_QUEUE, init_params->gie->tx_gie);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", giu_gpio_q_p->rem_q.q_id);
			}
		}
	}

	pr_debug("De-initializing Host Ingress TC queues\n");

	for (tc_idx = 0; tc_idx < init_params->outtcs_params.num_outtcs; tc_idx++) {
		outtc = &(init_params->outtcs_params.outtc_params[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_rem_inqs; q_idx++) {
			giu_gpio_q_p = &(outtc->rem_inqs_params[q_idx]);
			if (giu_gpio_q_p) {
				ret = giu_queue_remove(init_params->mqa, giu_gpio_q_p->rem_q.q,
							HOST_INGRESS_DATA_QUEUE, 0);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", giu_gpio_q_p->rem_q.q_id);
			}
		}
	}

	pr_debug("De-initializing Local Ingress TC queues\n");

	for (tc_idx = 0; tc_idx < init_params->outtcs_params.num_outtcs; tc_idx++) {
		outtc = &(init_params->outtcs_params.outtc_params[tc_idx]);

		for (q_idx = 0; q_idx < outtc->num_outqs; q_idx++) {
			giu_gpio_q_p = &(outtc->outqs_params[q_idx]);
			if (giu_gpio_q_p) {
				ret = giu_queue_remove(init_params->mqa, giu_gpio_q_p->lcl_q.q,
							LOCAL_INGRESS_DATA_QUEUE, init_params->gie->rx_gie);
				if (ret)
					pr_err("Failed to remove queue Idx %x\n", giu_gpio_q_p->lcl_q.q_id);
			}
		}
	}

	pr_debug("De-initializing Local Egress TC queues\n");

	for (tc_idx = 0; tc_idx < init_params->intcs_params.num_intcs; tc_idx++) {
		intc = &(init_params->intcs_params.intc_params[tc_idx]);

		for (q_idx = 0; q_idx < intc->num_inqs; q_idx++) {
			giu_gpio_q_p = &(intc->inqs_params[q_idx]);
			if (giu_gpio_q_p) {
				ret = giu_queue_remove(init_params->mqa, giu_gpio_q_p->lcl_q.q,
							LOCAL_EGRESS_DATA_QUEUE, 0);
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
	int ret;

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
	gpio_array[giu_id].params = giu_gpio_get_topology(giu_id);
	if (gpio_array[giu_id].params == NULL) {
		pr_err("queue topology was not initialized for GIU %d\n", giu_id);
		return -1;
	}

	pr_info("giu_gpio_probe giu_id %d\n", giu_id);


	*gpio = &gpio_array[giu_id];

	return 0;
}

void giu_gpio_remove(struct giu_gpio *gpio)
{
	pr_err("giu_gpio_remove is not implemented\n");
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
	struct giu_gpio_params *giu_params = gpio->params;
	struct giu_gpio_tc *gpio_tc = &giu_params->outqs_params.tcs[tc];
	uint8_t key_size = 0, extracted_key[MAX_EXTRACTION_SIZE];
	const uint8_t *ip_frame;
	uint64_t crc64 = 0;
	enum giu_outq_l3_type l3_info = GIU_TXD_GET_L3_PRS_INFO(desc);
	enum giu_outq_l4_type l4_info = GIU_TXD_GET_L4_PRS_INFO(desc);
	int ipv4;
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
		memcpy(&extracted_key[key_size], ipv4hdr->src_addr, MV_IPV4ADDR_LEN);
		key_size += MV_IPV4ADDR_LEN;
		memcpy(&extracted_key[key_size], ipv4hdr->dst_addr, MV_IPV4ADDR_LEN);
		key_size += MV_IPV4ADDR_LEN;
	} else {
		memcpy(&extracted_key[key_size], ipv6hdr->src_addr, MV_IPV6ADDR_LEN);
		key_size += MV_IPV6ADDR_LEN;
		memcpy(&extracted_key[key_size], ipv6hdr->dst_addr, MV_IPV6ADDR_LEN);
		key_size += MV_IPV6ADDR_LEN;
	}

	if ((gpio_tc->hash_type == GIU_GPIO_HASH_T_5_TUPLE) &&
		(l4_info == GIU_OUTQ_L4_TYPE_TCP || l4_info == GIU_OUTQ_L4_TYPE_UDP)) {
		if (ipv4)
			extracted_key[key_size] = ipv4hdr->proto;
		else
			extracted_key[key_size] = ipv6hdr->next_header;
		key_size += MV_IP_PROTO_NH_LEN;

		struct mv_udphdr *l4hdr = (struct mv_udphdr *)(ip_frame + (GIU_TXD_GET_IPHDR_LEN(desc) * 4));

		memcpy(&extracted_key[key_size], &l4hdr->src_port, MV_L4_PORT_LEN);
		key_size += MV_L4_PORT_LEN;
		memcpy(&extracted_key[key_size], &l4hdr->dst_port, MV_L4_PORT_LEN);
		key_size += MV_L4_PORT_LEN;
	}

	crc64 = crc64_compute(extracted_key, key_size);

	queue_index = crc64 % gpio_tc->dest_num_qs;
	GIU_TXD_SET_DEST_QID(desc, queue_index);

	return 0;
}

#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
static int giu_gpio_send_multi_q(struct giu_gpio *gpio, u8 tc, struct giu_gpio_desc *descs, u16 *num)
{
	struct giu_gpio_params *giu_params = gpio->params;
	struct giu_gpio_tc *gpio_tc = &giu_params->outqs_params.tcs[tc];
	struct giu_gpio_queue *txq;
	struct giu_gpio_desc *tx_ring_base;
	u16 dest_qid;
	int i;

	/* Calculate RSS and update descriptor */
	for (i = 0; i < *num; i++) {
		giu_gpio_update_rss(gpio, tc, &descs[i]);
		dest_qid = GIU_TXD_GET_DEST_QID(&descs[i]);
		/* Get queue params */
		txq = &gpio_tc->queues[dest_qid];

		if (QUEUE_FULL(txq->prod_val_shadow, *txq->cons_addr, txq->desc_total))
			continue;

		/* Get ring base */
		tx_ring_base = (struct giu_gpio_desc *)txq->desc_ring_base;

		/* Copy descriptor to descriptor ring */
		memcpy(&tx_ring_base[txq->prod_val_shadow], &descs[i], sizeof(*tx_ring_base));

		/* Increment producer index, update remaining descriptors count and block size */
		txq->prod_val_shadow = QUEUE_INDEX_INC(txq->prod_val_shadow, 1, txq->desc_total);
	}

	/* make sure all writes are done (i.e. descriptor were copied)
	 * before incrementing the producer index
	 */
	wmb();

	for (i = 0; i < gpio_tc->dest_num_qs; i++) {
		txq = &gpio_tc->queues[i];
		/* Update Producer index in GNPT */
		*txq->prod_addr = txq->prod_val_shadow;
	}

	return 0;
}
#endif

int giu_gpio_send(struct giu_gpio *gpio, u8 tc, u8 qid, struct giu_gpio_desc *descs, u16 *num)
{
	struct giu_gpio_queue *txq;
	struct giu_gpio_desc *tx_ring_base;
	struct giu_gpio_params *giu_params = gpio->params;
	u16 num_txds = *num, desc_remain;
	u16 block_size, index;
	u32 free_count, cons_val;
	int i;

#ifdef GIU_GPIO_DEBUG
	/* Check that the requested TC is supported */
	if (tc >= giu_params.outqs_params.num_tcs) {
		pr_err("GIU GPIO: TC %d is not supported (Max TC is %d)\n", tc, giu_params.outqs_params.num_tcs);
		return -1;
	}

	/* Check that the requested Q ID exists */
	if (qid >= giu_params->outqs_params.tcs[tc].num_qs) {
		pr_err("GIU GPIO: Q %d is not supported (Max Q is %d)\n", qid, giu_params->outqs_params.tcs[tc].num_qs);
		return -1;
	}
#endif

#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
	if (giu_params->outqs_params.tcs[tc].dest_num_qs > 1)
		return giu_gpio_send_multi_q(gpio, tc, descs, num);
#endif
	/* Set number of sent packets to 0 for any case we exit due to error */
	*num = 0;

	/* Get queue params */
	txq = &giu_params->outqs_params.tcs[tc].queues[qid];

	/* Get ring base */
	tx_ring_base = (struct giu_gpio_desc *)txq->desc_ring_base;

	/* Read producer index */
	cons_val = *txq->cons_addr;

	if (!num_txds) {
		pr_debug("num_txds is zero\n");
		return 0;
	} else if (likely(num_txds > txq->desc_total)) {
		pr_debug("More tx_descs(%u) than txq_len(%u)\n",
			num_txds, txq->desc_total);
	}

	/* Calculate number of free descriptors */
	free_count = QUEUE_SPACE(txq->prod_val_shadow, cons_val, txq->desc_total);

	if (free_count == 0) {
		pr_debug("GIU GPIO: No free descriptors for transmitting the packets\n");
		return 0;
	}

	if (unlikely(free_count < num_txds)) {
		pr_debug("num_txds(%d), free_count(%d)\n", num_txds, free_count);

		num_txds = free_count;
	}

	/* In case there is a wrap-around, handle the number of desc till the end of queue */
	block_size = min(num_txds, (u16)(txq->desc_total - txq->prod_val_shadow));

	desc_remain = num_txds;
	index = 0; /* index in source descriptor array */

	/* In case there is a wrap-around, the first iteration will handle the
	 * descriptors till the end of queue. The rest will be handled at the
	 * following iteration.
	 * Note that there should be no more than 2 iterations.
	 **/
	do {
		if (giu_params->outqs_params.tcs[tc].dest_num_qs > 1) {
			/* Calculate RSS and update descriptor */
			for (i = 0; i < block_size; i++)
				giu_gpio_update_rss(gpio, tc, &descs[i + index]);
		}

		/* Copy bulk of descriptors to descriptor ring */
		memcpy(&tx_ring_base[txq->prod_val_shadow], &descs[index], sizeof(*tx_ring_base) * block_size);

		/* Increment producer index, update remaining descriptors count and block size */
		txq->prod_val_shadow = QUEUE_INDEX_INC(txq->prod_val_shadow, block_size, txq->desc_total);
		desc_remain -= block_size;
		index = block_size;	/* next desc index in source array */
		block_size = desc_remain; /* next desc index in target ring */
	} while (desc_remain > 0);

	/* make sure all writes are done (i.e. descriptor were copied)
	 * before incrementing the producer index
	 */
	wmb();

	/* Update Producer index in GNPT */
	*txq->prod_addr = txq->prod_val_shadow;

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
	struct giu_gpio_params *giu_params = gpio->params;
	struct giu_gpio_queue *txq = &giu_params->outqs_params.tcs[tc].queues[qid];

	/* Read consumer index */
	cons_val = *txq->cons_addr;

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
int giu_gpio_get_num_outq_done(struct giu_gpio *gpio, u8 tc, u8 qid, u16 *num)
{
	u16 tx_num = 0;
	u8 i;
	struct giu_gpio_params *giu_params = gpio->params;

	for (*num = 0, i = 0; i < giu_params->outqs_params.tcs[tc].num_qs; i++) {
		giu_gpio_get_num_outq_done_internal(gpio, tc, i, &tx_num);
		*num += tx_num;
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
	struct giu_gpio_params *giu_params = gpio->params;
	u16 recv_req = *num, desc_received, desc_remain = 0;
	u16 block_size, index;
	u32 prod_val;

	*num = 0;
#ifdef GIU_GPIO_DEBUG
	/* Check that the requested TC is supported */
	if (tc >= giu_params->inqs_params.num_tcs) {
		pr_err("GIU GPIO: TC %d is not supported (Max TC is %d)\n", tc, giu_params->outqs_params.num_tcs);
		return -1;
	}

	/* Check that the requested Q ID exists */
	if (qid >= giu_params->inqs_params.tcs[tc].num_qs) {
		pr_err("GIU GPIO: Q %d is not supported (Max Q is %d)\n", qid, giu_params->nqs_params.tcs[tc].num_qs);
		return -1;
	}
#endif

	rxq = &giu_params->inqs_params.tcs[tc].queues[qid];

	/* Get ring base */
	rx_ring_base = (struct giu_gpio_desc *)rxq->desc_ring_base;

	/* Read producer index */
	prod_val = *rxq->prod_addr;

	/* Calculate number of received descriptors in the ring.
	 * Since queue size is a power of 2, we can use below formula.
	 */
	desc_received = QUEUE_OCCUPANCY(prod_val, rxq->cons_val_shadow, rxq->desc_total);
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
	if (unlikely((rxq->cons_val_shadow + recv_req) > rxq->desc_total)) {
		block_size = rxq->desc_total - rxq->cons_val_shadow;
	} else {
		/* No wrap around */
		block_size = recv_req;
	}

	desc_remain = recv_req;
	index = 0; /* index in destination descriptor array */

	/* Note: since we handle wrap-around, the should be no more than 2 iterations */
	do {
		/* Copy bulk of descriptors from the descriptor ring */
		memcpy(&descs[index], &rx_ring_base[rxq->cons_val_shadow], block_size * sizeof(*descs));

		/* Increment consumer index, update remaining descriptors count and block size */
		rxq->cons_val_shadow = QUEUE_INDEX_INC(rxq->cons_val_shadow, block_size, rxq->desc_total);
		desc_remain -= block_size;
		index = block_size; /* next desc index in destination array */
		block_size = desc_remain; /* next desc index in source ring */
	} while (desc_remain);

	/* make sure all writes are done (i.e. descriptor were copied)
	 * before incrementing the consumer index
	 */
	wmb();

	/* Update Consumer index in GNCT */
	*rxq->cons_addr = rxq->cons_val_shadow;

	/* Update number of received descriptors */
	*num = recv_req;

	return 0;
}

#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
int giu_gpio_recv(struct giu_gpio *gpio, u8 tc, u8 qid, struct giu_gpio_desc *descs, u16 *num)
{
	static u8 curr_qid;
	u8 i;
	u16 recv_req = *num, total_got = 0;
	struct giu_gpio_params *giu_params = gpio->params;

	for (i = 0; (i < giu_params->inqs_params.tcs[tc].num_qs) && (total_got != recv_req); i++) {
		*num = recv_req - total_got;
		giu_gpio_recv_internal(gpio, tc, curr_qid, &descs[total_got], num);
		total_got += *num;
		curr_qid++;
		if (curr_qid == giu_params->inqs_params.tcs[tc].num_qs)
			curr_qid = 0;
	}

	*num = total_got;

	return 0;
}
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS */

int giu_gpio_get_capabilities(struct giu_gpio *gpio, struct giu_gpio_capabilities *capa)
{
	struct giu_gpio_params *gpio_params = gpio->params;
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
