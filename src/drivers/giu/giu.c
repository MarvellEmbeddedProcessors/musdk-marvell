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

#define log_fmt(fmt) "giu: " fmt

#include "std_internal.h"
#include "drivers/giu_internal.h"
#include "hw_emul/gie.h"

/* TODO - Currently MUSDK only supports AP DMAs so we assign them in
 * sequential order. Once CP DMAs are supported, we will need to
 * somehow specify from which die to instantiate a DMA
 */
int instance;

void *giu_open(struct giu_params *giu_pars, char *name)
{
	struct gie_regfile *regs;

	/* allocate pseudo regfile for the GIU emulator */
	regs = kcalloc(1, sizeof(*regs), GFP_KERNEL);
	if (regs == NULL)
		return NULL;

	regs->gct_base  = giu_pars->gct_base;
	regs->gpt_base  = giu_pars->gpt_base;
	regs->gncs_base = giu_pars->gncs_base;
	regs->gnps_base = giu_pars->gnps_base;
	regs->msi_base  = giu_pars->msi_base;
	regs->msix_base = giu_pars->msix_base;

	return gie_init(regs, instance++, name);
}

int giu_close(void *giu)
{
	int ret;
	struct gie_regfile *regs = gie_regs(giu);

	ret = gie_terminate(giu);

	kfree(regs);
	return ret;
}

int giu_add_queue(void *giu, u16 qid, int is_remote)
{
	return gie_add_queue(giu, qid, is_remote);
}

int giu_add_bm_queue(void *giu, u16 qid, int buf_size, int is_remote)
{
	return gie_add_bm_queue(giu, qid, buf_size, is_remote);
}

int giu_remove_queue(void *giu, u16 qid)
{
	return gie_remove_queue(giu, qid);
}

int giu_remove_bm_queue(void *giu, u16 qid)
{
	return gie_remove_bm_queue(giu, qid);
}

int giu_schedule(void *giu, u64 time_limit, u64 qe_limit)
{
	return gie_schedule(giu, time_limit, qe_limit);
}
