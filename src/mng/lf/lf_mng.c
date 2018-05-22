/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#define log_fmt(fmt) "lf_mng " fmt

#include "std_internal.h"
#include "env/trace/trc_pf.h"
#include "mng/db.h"
#include "mng/dispatch.h"
#include "pf/pf.h"
#include "custom/custom.h"
#include "lf_mng.h"


int lf_init(struct nmp *nmp)
{
	int ret;
	struct nmcstm_params params;

	/* TODO: init nmp-nicpf only in case it is supported in the nmp_params! */
	ret = nmnicpf_init(&nmp->nmnicpf);
	if (ret)
		return ret;

	/* TODO: init nmp-customer only in case it is supported in the nmp_params! */
	params.id = nmp->guest_id;
	params.mqa = nmp->mqa;
	params.nmdisp = nmp->nmdisp;
	params.pf_id = nmp->nmnicpf.pf_id;
	ret = nmcstm_init(&params, &(nmp->nmcstm));
	if (ret) {
		pr_err("Custom init failed\n");
		return ret;
	}

	nmdisp_dispatch_dump(nmp->nmdisp);

	return 0;
}


int lf_deinit(struct nmp *nmp)
{
	int ret;

	ret = nmcstm_deinit(nmp->nmcstm);
	if (ret) {
		pr_err("Custom init failed\n");
		return ret;
	}

	ret = nmnicpf_deinit(&nmp->nmnicpf);
	if (ret)
		return ret;

	return 0;
}

