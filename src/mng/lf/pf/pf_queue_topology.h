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

#ifndef _PF_QUEUE_TOPOLOGY_H
#define _PF_QUEUE_TOPOLOGY_H

#define QUEUE_FREE_STATUS (-1)

enum queue_type {
	MNG_CMD_QUEUE,
	MNG_NOTIFY_QUEUE,
	LOCAL_INGRESS_DATA_QUEUE,
	LOCAL_EGRESS_DATA_QUEUE,
	LOCAL_BM_QUEUE,
	HOST_INGRESS_DATA_QUEUE,
	HOST_EGRESS_DATA_QUEUE,
	HOST_BM_QUEUE

};

/* RSS Hash Type
 *
 *	HASH_2_TUPLE - IP-src, IP-dst
 *	HASH_5_TUPLE - IP-src, IP-dst, IP-Prot, L4-src, L4-dst
 */
enum rss_hash_type {
	RSS_HASH_NONE = 0,
	RSS_HASH_2_TUPLE,
	RSS_HASH_5_TUPLE

};

/*	Traffic Class information
 *
 *	tc_id	      - traffic class Id
 *	num_queues    - number queues in traffic class
 *	rss_type      - Ingress RSS type (Egress type should be set to HASH_NONE)
 *	tc_queues_idx - array of queue indexs associated with this traffic class
 */
struct tc_params {
	u32 tc_id;
	u32 num_of_queues;
	u32 rss_type;
	struct giu_gpio_q *tc_queue_params;

};

/*	Management Channels information
 *
 *	cmd_queue_id	 - command queue Id
 *	notify_queue_id - notification queue Id
 */
struct mng_ch_params {
	struct giu_gpio_q *cmd_queue;
	struct giu_gpio_q *notify_queue;

};

/* Mng Queue topology */
struct giu_mng_topology {
	struct mng_ch_params lcl_mng_ctrl;
	struct mng_ch_params host_mng_ctrl;

};

/* In TC - Queue topology */
struct giu_gpio_intc_params {
	u32 tc_id;

	/* lcl_eg_tcs */
	u32 num_inqs;
	struct giu_gpio_q *inqs_params;

	/* lcl_bm_qs_num */
	u32 num_inpools;
	/* lcl_bm_qs_params */
	struct giu_gpio_q *pools;

	/* host_eg_tcs */
	u32 num_rem_outqs;
	struct giu_gpio_q *rem_outqs_params;

};

struct giu_gpio_intcs_params {
	u32 num_intcs;
	struct giu_gpio_intc_params *intc_params;

};

/* Out TC - Queue topology */
struct giu_gpio_outtc_params {
	u32 tc_id;

	/* lcl_ing_tcs */
	u32 num_outqs;
	struct giu_gpio_q *outqs_params;

	/* host_ing_tcs */
	u32 num_rem_inqs;
	u8 rss_type;
	struct giu_gpio_q *rem_inqs_params;

	u32 host_bm_qs_num;
	struct giu_gpio_q *rem_poolqs_params;

};

struct giu_gpio_outtcs_params {
	u32 num_outtcs;
	struct giu_gpio_outtc_params *outtc_params;

};

/* Queue topology */
struct giu_queue_topology {
	u8 id;

	struct giu_gpio_intcs_params  intcs_params;
	struct giu_gpio_outtcs_params outtcs_params;

};

#endif /* _PF_QUEUE_TOPOLOGY_H */

