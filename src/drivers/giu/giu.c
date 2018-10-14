/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "giu: " fmt

#include "std_internal.h"

#include "drivers/mv_giu.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "drivers/mv_giu_gpio.h"
#include "hw_emul/gie.h"

#include "giu_internal.h"


int giu_destroy_q(struct giu *giu, enum giu_eng eng, struct mqa *mqa,
	struct mqa_q *q, enum queue_type queue_type)
{
	struct gie *gie = NULL;
	int ret = 0;
	u32 qid;

	if (unlikely(!giu)) {
		pr_err("Invalid GIU handle!\n");
		return -EINVAL;
	}

	if (eng < GIU_ENG_OUT_OF_RANGE)
		gie = giu->gies[eng];

	mqa_queue_get_id(q, &qid);

	pr_debug("Remove queue %d (type %d)\n", qid, queue_type);

	if (gie) {
		/* Un-register Q from GIU */
		if (queue_type == LOCAL_BM_QUEUE ||
		    queue_type == HOST_BM_QUEUE)
			ret = gie_remove_bm_queue(gie, qid);
		else
			ret = gie_remove_queue(gie, qid);
		if (ret)
			pr_err("Failed to remove queue Idx %x from GIU\n", qid);
	}

	/* For local queue: destroy the queue (as it was allocated by the NIC */
	if (queue_type == LOCAL_INGRESS_DATA_QUEUE ||
	    queue_type == LOCAL_EGRESS_DATA_QUEUE ||
	    queue_type == LOCAL_BM_QUEUE) {
		ret = mqa_queue_destroy(mqa, q);
		if (ret)
			pr_err("Failed to free queue Idx %x in DB\n", qid);
	}

	/* Free the MQA resource */
	ret = mqa_queue_free(mqa, qid);
	if (ret)
		pr_err("Failed to free queue Idx %x in MQA\n", qid);

	return ret;
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
	struct gie_params	 gie_params;
	struct mqa_info		 mqa_info;
	int			 ret;

	_giu = kmalloc(sizeof(struct giu), GFP_KERNEL);
	if (!_giu)
		return -ENOMEM;
	memset(_giu, 0, sizeof(struct giu));

	_giu->mqa = params->mqa;

	mqa_get_info(_giu->mqa, &mqa_info);

	memset(&gie_params, 0, sizeof(gie_params));

	/* Mgmt GIE */
	gie_params.gct_base = (u64)mqa_info.qct_va;
	gie_params.gpt_base = (u64)mqa_info.qpt_va;
	_giu->msi_regs_pa = params->msi_regs_pa;
	_giu->msi_regs_va = params->msi_regs_va;
	gie_params.msi_regs_phys = _giu->msi_regs_pa;
	gie_params.msi_regs_virt = _giu->msi_regs_va;

	gie_params.dmax_match = params->mng_gie_params.dma_eng_match;
	gie_params.name_match = (char *)"MNG";

	ret = gie_init(&gie_params, &(_giu->gies[GIU_ENG_MNG]));
	if (ret) {
		pr_err("Failed to initialize management GIU emulator!\n");
		return -ENODEV;
	}

	/* OUT GIE */
	gie_params.dmax_match = params->out_gie_params.dma_eng_match;
	gie_params.name_match = (char *)"OUT";

	ret = gie_init(&gie_params, &(_giu->gies[GIU_ENG_OUT]));
	if (ret) {
		pr_err("Failed to initialize OUT GIU emulator!\n");
		gie_terminate(_giu->gies[GIU_ENG_MNG]);
		return -EIO;
	}

	/* IN GIE */
	gie_params.dmax_match = params->in_gie_params.dma_eng_match;
	gie_params.name_match = (char *)"IN";

	ret = gie_init(&gie_params, &(_giu->gies[GIU_ENG_IN]));
	if (ret) {
		pr_err("Failed to initialize IN GIU emulator!\n");
		gie_terminate(_giu->gies[GIU_ENG_OUT]);
		gie_terminate(_giu->gies[GIU_ENG_MNG]);
		return -EIO;
	}

	*giu = _giu;

	return 0;
}

void giu_deinit(struct giu *giu)
{
	if (!giu)
		return;

	if (giu->gies[GIU_ENG_IN])
		gie_terminate(giu->gies[GIU_ENG_IN]);
	if (giu->gies[GIU_ENG_OUT])
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
	q_params.cons_phys       = (void *)(params->rem_base_pa + params->rem_cmd_q.cons_offs);
	q_params.cons_virt       = params->rem_base_va + params->rem_cmd_q.cons_offs;
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
	q_params.prod_phys       = (void *)(params->rem_base_pa + params->rem_resp_q.prod_offs);
	q_params.prod_virt       = params->rem_base_va + params->rem_resp_q.prod_offs;
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

	return gie_schedule(giu->gies[eng], time_limit, qe_limit, pending);
}

int giu_get_desc_size(struct giu *giu, enum giu_desc_type type)
{
	if (unlikely(!giu)) {
		pr_err("Invalid GIU handle!\n");
		return -EINVAL;
	}

	return gie_get_desc_size((enum gie_desc_type)type);
}

int giu_set_remote_index_mode(struct giu *giu, enum giu_indices_copy_mode mode)
{
	if (unlikely(!giu)) {
		pr_err("Invalid GIU handle!\n");
		return -EINVAL;
	}

	giu->indices_mode = mode;

	return gie_set_remote_index_mode((enum gie_copy_mode_type)mode);
}

enum giu_multi_qs_mode giu_get_multi_qs_mode(struct giu *giu)
{
	if (unlikely(!giu)) {
		pr_err("Invalid GIU handle!\n");
		return -EINVAL;
	}

#ifdef GIE_NO_MULTI_Q_SUPPORT_FOR_RSS
	return GIU_MULTI_QS_MODE_VIRT;
#else
	return GIU_MULTI_QS_MODE_REAL;
#endif /* GIE_NO_MULTI_Q_SUPPORT_FOR_RSS */
}
