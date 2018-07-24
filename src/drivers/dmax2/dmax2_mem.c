/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
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

#include "dmax2.h"


struct dmax2_lnx_format {
	enum musdk_lnx_id ver;
	char *uio_format;
};

struct dmax2_lnx_format dmax2_frm[] = {
		{LNX_4_4_x, "dma_xor"},
		{LNX_OTHER, "xor"},
};

int init_dmax2_mem(struct dmax2 *dmax2)
{
	struct sys_iomem_params	 iomem_params;
	dma_addr_t		 addr;
	int			 err;
	enum musdk_lnx_id lnx_id = lnx_id_get();

	iomem_params.devname = dmax2_frm[lnx_id].uio_format;
	iomem_params.index = dmax2->id;
	iomem_params.type = SYS_IOMEM_T_UIO;

	err = sys_iomem_init(&iomem_params, &dmax2->iomem);
	if (err) {
		pr_err("failed to created IOMEM!\n");
		return err;
	}

	err = sys_iomem_map(dmax2->iomem, "0", &addr, &dmax2->dma_base);
	if (err)
		return err;

	pr_debug("DMA %d address: pa %lx, va %p\n", dmax2->id, addr, dmax2->dma_base);

	err = sys_iomem_map(dmax2->iomem, "1", &addr, &dmax2->glob_base);
	if (err)
		return err;

	pr_debug("DMA %d global address: pa %lx, va %p\n", dmax2->id, addr, dmax2->glob_base);

	return 0;
}

void deinit_dmax2_mem(struct dmax2 *dmax2)
{
	sys_iomem_unmap(dmax2->iomem, "1");
	sys_iomem_unmap(dmax2->iomem, "0");
	sys_iomem_deinit(dmax2->iomem);
}
