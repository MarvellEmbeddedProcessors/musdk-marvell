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

#ifndef _MV_GIU_GPIO_INIT_H
#define _MV_GIU_GPIO_INIT_H

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

struct lcl_q_params {
	struct mqa_q *q;
	u32 q_id;
	u32 len;

};

struct rem_q_params {
	struct mqa_q *q;
	u32 q_id;
	u32 len;
	u32 size;
	void        *host_remap;
	phys_addr_t  q_base_pa;
	void        *prod_base_va;
	phys_addr_t  prod_base_pa;
	void        *cons_base_va;
	phys_addr_t  cons_base_pa;

};

union giu_gpio_q_params {
	struct lcl_q_params lcl_q;
	struct rem_q_params rem_q;

};

/* In TC - Queue topology */
struct giu_gpio_intc_params {
	u32 tc_id;

	/* lcl_eg_tcs */
	u32 num_inqs;
	union giu_gpio_q_params *inqs_params;

	/* lcl_bm_qs_num */
	u32 num_inpools;
	u32 pool_buf_size;
	/* lcl_bm_qs_params */
	union giu_gpio_q_params *pools;

	/* host_eg_tcs */
	u32 num_rem_outqs;
	union giu_gpio_q_params *rem_outqs_params;

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
	union giu_gpio_q_params *outqs_params;

	/* host_ing_tcs */
	u32 num_rem_inqs;
	u8 rss_type;
	/* The following two parameters are added inorder not to break the init flow
	 * between Netdev and GIU - It should be update at later phase
	 */
	u32	rem_inqs_bpool_num;
	u32	rem_inqs_bpool_list[4];
	union giu_gpio_q_params *rem_inqs_params;

	u32 host_bm_qs_num;
	union giu_gpio_q_params *rem_poolqs_params;

};

struct giu_gpio_outtcs_params {
	u32 num_outtcs;
	struct giu_gpio_outtc_params *outtc_params;

};

/* Queue topology */
struct giu_gpio_init_params {
	u8 id;

	struct mqa *mqa;
	struct gie_data *gie;

	struct giu_gpio_intcs_params  intcs_params;
	struct giu_gpio_outtcs_params outtcs_params;

};

#endif /* _MV_GIU_GPIO_INIT_H */

