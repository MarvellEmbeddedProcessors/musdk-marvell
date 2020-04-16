/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt, ...) "giu: " fmt, ##__VA_ARGS__

#include "std_internal.h"

#include "drivers/mv_giu.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mv_giu_gpio.h"

#include "giu_internal.h"


static int init_gies(struct giu *giu, u8 num_gies, struct giu_emul_params *gies_params)
{
	struct gie_params	 gie_params;
	struct mqa_info		 mqa_info;
	int			 ret;

	mqa_get_info(giu->mqa, &mqa_info);

	memset(&gie_params, 0, sizeof(gie_params));

	/* Mgmt GIE */
	gie_params.gct_base = (u64)mqa_info.qct_va;
	gie_params.gpt_base = (u64)mqa_info.qpt_va;
	gie_params.msi_regs_phys = giu->msi_regs_pa;
	gie_params.msi_regs_virt = giu->msi_regs_va;

	gie_params.dmax_match = gies_params[0].dma_eng_match;
	gie_params.name_match = (char *)"MNG";

	ret = gie_init(&gie_params, &(giu->gies[GIU_ENG_MNG]));
	if (ret) {
		pr_err("Failed to initialize management GIU emulator!\n");
		return -ENODEV;
	}

	if (num_gies == 1) {
		giu->gies[GIU_ENG_OUT] = giu->gies[GIU_ENG_MNG];
		giu->gies[GIU_ENG_IN] = giu->gies[GIU_ENG_MNG];
		return 0;
	}

	/* OUT GIE */
	if (strcmp(gies_params[1].dma_eng_match, gies_params[0].dma_eng_match) == 0)
		giu->gies[GIU_ENG_OUT] = giu->gies[GIU_ENG_MNG];
	else {
		gie_params.dmax_match = gies_params[1].dma_eng_match;
		gie_params.name_match = (char *)"OUT";

		ret = gie_init(&gie_params, &(giu->gies[GIU_ENG_OUT]));
		if (ret) {
			pr_err("Failed to initialize OUT GIU emulator!\n");
			gie_terminate(giu->gies[GIU_ENG_MNG]);
			return -EIO;
		}
	}

	if (num_gies == 2) {
		giu->gies[GIU_ENG_IN] = giu->gies[GIU_ENG_OUT];
		return 0;
	}

	/* IN GIE */
	if (strcmp(gies_params[2].dma_eng_match, gies_params[0].dma_eng_match) == 0)
		giu->gies[GIU_ENG_IN] = giu->gies[GIU_ENG_MNG];
	else if (strcmp(gies_params[2].dma_eng_match, gies_params[1].dma_eng_match) == 0)
		giu->gies[GIU_ENG_IN] = giu->gies[GIU_ENG_OUT];
	else {
		gie_params.dmax_match = gies_params[2].dma_eng_match;
		gie_params.name_match = (char *)"IN";

		ret = gie_init(&gie_params, &(giu->gies[GIU_ENG_IN]));
		if (ret) {
			pr_err("Failed to initialize IN GIU emulator!\n");
			gie_terminate(giu->gies[GIU_ENG_OUT]);
			gie_terminate(giu->gies[GIU_ENG_MNG]);
			return -EIO;
		}
	}

	return 0;
}

struct gie *giu_get_gie_handle(struct giu *giu, enum giu_eng eng)
{
	if (unlikely(!giu)) {
		pr_err("Invalid GIU handle!\n");
		return NULL;
	}

	if (unlikely(eng >= GIU_ENG_OUT_OF_RANGE)) {
		pr_err("Invalid GIU engine!\n");
		return NULL;
	}

	return giu->gies[eng];
}

int giu_get_msi_regs(struct giu *giu, u64 *va, u64 *pa)
{
	if (unlikely(!giu)) {
		pr_err("Invalid GIU handle!\n");
		return -EINVAL;
	}
	*va = giu->msi_regs_va;
	*pa = giu->msi_regs_pa;
	return 0;
}

int giu_init(struct giu_params *params, struct giu **giu)
{
	struct giu		*_giu;
	int			 ret;

	_giu = kmalloc(sizeof(struct giu), GFP_KERNEL);
	if (!_giu)
		return -ENOMEM;
	memset(_giu, 0, sizeof(struct giu));

	_giu->mqa = params->mqa;
	_giu->msi_regs_pa = params->msi_regs_pa;
	_giu->msi_regs_va = params->msi_regs_va;

	ret = init_gies(_giu, params->num_gies, params->gies_params);
	if (ret) {
		pr_err("Failed to initialize OUT GIU emulator!\n");
		return ret;
	}

	*giu = _giu;

	return 0;
}

void giu_deinit(struct giu *giu)
{
	if (!giu)
		return;

	if (giu->gies[GIU_ENG_IN] && (giu->gies[GIU_ENG_IN] != giu->gies[GIU_ENG_OUT]))
		gie_terminate(giu->gies[GIU_ENG_IN]);
	if (giu->gies[GIU_ENG_OUT] && (giu->gies[GIU_ENG_OUT] != giu->gies[GIU_ENG_MNG]))
		gie_terminate(giu->gies[GIU_ENG_OUT]);
	if (giu->gies[GIU_ENG_MNG])
		gie_terminate(giu->gies[GIU_ENG_MNG]);

	kfree(giu);
}

int giu_mng_ch_init(struct giu *giu, struct giu_mng_ch_params *params, struct giu_mng_ch **mng_ch)
{
	struct giu_mng_ch	*_mng_ch;
	struct mqa_queue_params	 q_params;
	int			 ret;

	_mng_ch = kmalloc(sizeof(struct giu_mng_ch), GFP_KERNEL);
	if (!_mng_ch)
		return -ENOMEM;
	memset(_mng_ch, 0, sizeof(struct giu_mng_ch));

	_mng_ch->giu = giu;

	/*  Create Local Queues */
	/* ==================== */

	/* Allocate and Register Local Command queue in MQA */
	pr_debug("Register Local Command Q\n");

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(giu->mqa, &_mng_ch->lcl_cmd_q_idx);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		giu_mng_ch_deinit(_mng_ch);
		return ret;
	}

	memset(&q_params, 0, sizeof(q_params));

	q_params.idx  = _mng_ch->lcl_cmd_q_idx;
	q_params.len  = params->lcl_cmd_q.len;
	q_params.size = params->desc_size;
	q_params.attr = MQA_QUEUE_LOCAL | MQA_QUEUE_EGRESS;
	q_params.prio = 0;

	ret = mqa_queue_create(giu->mqa, &q_params, &_mng_ch->lcl_cmd_q);
	if (ret < 0) {
		pr_err("Failed to register Host Management Q\n");
		giu_mng_ch_deinit(_mng_ch);
		return ret;
	}

	/* Allocate and Register Local Notification queue in MQA */
	pr_debug("Register Local Notification Q\n");

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(giu->mqa, &_mng_ch->lcl_resp_q_idx);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		giu_mng_ch_deinit(_mng_ch);
		return ret;
	}

	memset(&q_params, 0, sizeof(q_params));

	q_params.idx   = _mng_ch->lcl_resp_q_idx;
	q_params.len   = params->lcl_resp_q.len;
	q_params.size  = params->desc_size;
	q_params.attr  = MQA_QUEUE_LOCAL | MQA_QUEUE_INGRESS;
	q_params.prio  = 0;

	ret = mqa_queue_create(giu->mqa, &q_params, &_mng_ch->lcl_resp_q);
	if (ret < 0) {
		pr_err("Failed to register Host Management Q\n");
		giu_mng_ch_deinit(_mng_ch);
		return ret;
	}

	/*  Register Remote Queues */
	/* ======================= */

	/* Register Host Command management queue */
	pr_debug("Register host command queue\n");

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(giu->mqa, &_mng_ch->rem_cmd_q_idx);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		giu_mng_ch_deinit(_mng_ch);
		return ret;
	}

	memset(&q_params, 0, sizeof(q_params));

	q_params.idx             = _mng_ch->rem_cmd_q_idx;
	q_params.len             = params->rem_cmd_q.len;
	q_params.size            = params->desc_size;
	q_params.attr            = MQA_QUEUE_REMOTE | MQA_QUEUE_EGRESS;
	q_params.prio            = 0;
	q_params.remote_phy_addr = (void *)params->rem_cmd_q.pa;
	q_params.prod_phys       = (void *)params->rem_cmd_q.prod_pa;
	q_params.prod_virt       = params->rem_cmd_q.prod_va;
	q_params.cons_phys       = (void *)params->rem_cmd_q.cons_pa;
	q_params.cons_virt       = params->rem_cmd_q.cons_va;
	q_params.host_remap      = (void *)params->rem_base_pa;
	q_params.peer_id         = _mng_ch->lcl_cmd_q_idx;

	/* Allocate queue from MQA */
	ret = mqa_queue_create(giu->mqa, &q_params, &_mng_ch->rem_cmd_q);
	if (ret < 0) {
		pr_err("Failed to register Host Management Q\n");
		giu_mng_ch_deinit(_mng_ch);
		return ret;
	}

	/* Register Host Notification queue */
	pr_debug("Register host notification queue\n");

	/* Allocate queue from MQA */
	ret = mqa_queue_alloc(giu->mqa, &_mng_ch->rem_resp_q_idx);
	if (ret < 0) {
		pr_err("Failed to allocate queue from MQA\n");
		giu_mng_ch_deinit(_mng_ch);
		return ret;
	}

	memset(&q_params, 0, sizeof(q_params));

	q_params.idx             = _mng_ch->rem_resp_q_idx;
	q_params.len             = params->rem_resp_q.len;
	q_params.size            = params->desc_size;
	q_params.attr            = MQA_QUEUE_REMOTE | MQA_QUEUE_INGRESS;
	q_params.prio            = 0;
	q_params.remote_phy_addr = (void *)params->rem_resp_q.pa;
	q_params.prod_phys       = (void *)params->rem_resp_q.prod_pa;
	q_params.prod_virt       = params->rem_resp_q.prod_va;
	q_params.cons_phys       = (void *)params->rem_resp_q.cons_pa;
	q_params.cons_virt       = params->rem_resp_q.cons_va;
	q_params.host_remap      = (void *)params->rem_base_pa;

	ret = mqa_queue_create(giu->mqa, &q_params, &_mng_ch->rem_resp_q);
	if (ret < 0) {
		pr_err("Failed to register Host Management Q\n");
		giu_mng_ch_deinit(_mng_ch);
		return ret;
	}

	ret = mqa_queue_associate_pair(giu->mqa, _mng_ch->lcl_resp_q_idx, _mng_ch->rem_resp_q_idx);
	if (ret < 0) {
		pr_err("Failed to associate Notification queues (Src %d Dest %d)\n",
			_mng_ch->lcl_resp_q_idx, _mng_ch->rem_resp_q_idx);
		giu_mng_ch_deinit(_mng_ch);
		return ret;
	}

	/* Register Qs in GIE */
	/* ================== */

	/* Register Command channel */
	gie_add_queue(giu->gies[GIU_ENG_MNG], _mng_ch->rem_cmd_q_idx, 1);

	/* Register Notification channel */
	gie_add_queue(giu->gies[GIU_ENG_MNG], _mng_ch->lcl_resp_q_idx, 0);

	*mng_ch = _mng_ch;
	return 0;
}

void giu_mng_ch_deinit(struct giu_mng_ch *mng_ch)
{
	int ret;

	if (!mng_ch)
		return;

	if (mng_ch->lcl_cmd_q_idx >= 0) {
		if (mng_ch->lcl_cmd_q) {
			ret = mqa_queue_destroy(mng_ch->giu->mqa, mng_ch->lcl_cmd_q);
			if (ret < 0)
				pr_err("Failed to free Local Cmd Q %d in DB\n", mng_ch->lcl_cmd_q_idx);
		}
		ret = mqa_queue_free(mng_ch->giu->mqa, mng_ch->lcl_cmd_q_idx);
		if (ret < 0)
			pr_err("Failed to free Local Cmd Q %d in MQA\n", mng_ch->lcl_cmd_q_idx);
	}

	if (mng_ch->lcl_resp_q_idx >= 0) {
		gie_remove_queue(mng_ch->giu->gies[GIU_ENG_MNG], mng_ch->lcl_resp_q_idx);
		if (mng_ch->lcl_resp_q) {
			ret = mqa_queue_destroy(mng_ch->giu->mqa, mng_ch->lcl_resp_q);
			if (ret < 0)
				pr_err("Failed to free Local Notify Q %d in DB\n", mng_ch->lcl_resp_q_idx);
		}
		ret = mqa_queue_free(mng_ch->giu->mqa, mng_ch->lcl_resp_q_idx);
		if (ret < 0)
			pr_err("Failed to free Local Notify Q %d in MQA\n", mng_ch->lcl_resp_q_idx);
	}

	if (mng_ch->rem_cmd_q_idx >= 0) {
		gie_remove_queue(mng_ch->giu->gies[GIU_ENG_MNG], mng_ch->rem_cmd_q_idx);
		if (mng_ch->rem_cmd_q) {
			ret = mqa_queue_destroy(mng_ch->giu->mqa, mng_ch->rem_cmd_q);
			if (ret < 0)
				pr_err("Failed to free remote Cmd Q %d in DB\n", mng_ch->rem_cmd_q_idx);
		}
		ret = mqa_queue_free(mng_ch->giu->mqa, mng_ch->rem_cmd_q_idx);
		if (ret < 0)
			pr_err("Failed to free remote Cmd Q %d in MQA\n", mng_ch->rem_cmd_q_idx);
	}

	if (mng_ch->rem_resp_q_idx >= 0) {
		if (mng_ch->rem_resp_q) {
			ret = mqa_queue_destroy(mng_ch->giu->mqa, mng_ch->rem_resp_q);
			if (ret < 0)
				pr_err("Failed to free Remote Notify Q %d in DB\n", mng_ch->rem_resp_q_idx);
		}
		ret = mqa_queue_free(mng_ch->giu->mqa, mng_ch->rem_resp_q_idx);
		if (ret < 0)
			pr_err("Failed to free Remote Notify Q %d in MQA\n", mng_ch->rem_resp_q_idx);
	}

	kfree(mng_ch);
}

int giu_mng_ch_get_qs(struct giu_mng_ch *mng_ch, struct giu_mng_ch_qs *qs)
{
	if (unlikely(!mng_ch)) {
		pr_err("Invalid GIU MNG-CH handle!\n");
		return -EINVAL;
	}

	qs->lcl_cmd_q = mng_ch->lcl_cmd_q;
	qs->lcl_resp_q = mng_ch->lcl_resp_q;
	qs->rem_cmd_q = mng_ch->rem_cmd_q;
	qs->rem_resp_q = mng_ch->rem_resp_q;

	return 0;
}

int giu_schedule(struct giu *giu, enum giu_eng eng, u64 time_limit, u64 qe_limit, u16 *pending)
{
	int gpio_idx;

#ifdef DEBUG
	if (unlikely(!giu)) {
		pr_err("Invalid GIU handle!\n");
		return -EINVAL;
	}

	if (unlikely(eng >= GIU_ENG_OUT_OF_RANGE)) {
		pr_err("Invalid GIU engine!\n");
		return -EINVAL;
	}
#endif /* DEBUG */

	/* TX */
	if (eng == GIU_ENG_OUT) {
		for (gpio_idx = 0; gpio_idx < GIU_MAX_NUM_GPIO; gpio_idx++) {
			if (giu->gpio_list[gpio_idx] != 0)
				giu_gpio_pre_gie(giu->gpio_list[gpio_idx]);
		}
	}

	/* RX */
	/* more exactly post processing for previous iteration of gie_schedule() */
	if (eng == GIU_ENG_IN) {
		for (gpio_idx = 0; gpio_idx < GIU_MAX_NUM_GPIO; gpio_idx++) {
			if (giu->gpio_list[gpio_idx] != 0)
				giu_gpio_post_gie(giu->gpio_list[gpio_idx]);
		}
	}

	return gie_schedule(giu->gies[eng], time_limit, qe_limit, pending);
}


int giu_gpio_register(struct giu *giu, struct giu_gpio *gpio, int gpio_idx)
{
	if (!giu) {
		pr_err("Invalid GIU handle!\n");
		return -EINVAL;
	}

	if (!gpio) {
		pr_err("Invalid GPIO handle!\n");
		return -EINVAL;
	}

	if (gpio_idx >= GIU_MAX_NUM_GPIO) {
		pr_err("Invalid GPIO index %d !\n", gpio_idx);
		return -EINVAL;
	}

	if (giu->gpio_list[gpio_idx] != 0) {
		pr_err("GPIO index %d already registered!\n", gpio_idx);
		return -EINVAL;
	}

	giu->gpio_list[gpio_idx] = gpio;

	return 0;
}

int giu_gpio_unregister(struct giu *giu, struct giu_gpio *gpio, int gpio_idx)
{
	if (!giu) {
		pr_err("Invalid GIU handle!\n");
		return -EINVAL;
	}

	if (!gpio) {
		pr_err("Invalid GPIO handle!\n");
		return -EINVAL;
	}

	if (gpio_idx >= GIU_MAX_NUM_GPIO) {
		pr_err("Invalid GPIO index %d !\n", gpio_idx);
		return -EINVAL;
	}

	if (giu->gpio_list[gpio_idx] == 0) {
		pr_err("GPIO %d is not registered!\n", gpio_idx);
		return -EINVAL;
	}

	giu->gpio_list[gpio_idx] = 0;

	return 0;
}

int giu_get_desc_size(struct giu *giu, enum giu_desc_type type)
{
	if (unlikely(!giu)) {
		pr_err("Invalid GIU handle!\n");
		return -EINVAL;
	}

	return gie_get_desc_size((enum gie_desc_type)type);
}
