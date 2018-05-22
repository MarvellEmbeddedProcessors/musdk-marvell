/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

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
	u32 msix_id;
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
	u8 rss_type;
	u32 num_rem_inqs;
	union giu_gpio_q_params *rem_inqs_params;
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

