/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"

#include "dmax2.h"


struct dmax2_lnx_format {
	enum musdk_lnx_id ver;
	char *uio_format;
};

struct dmax2_lnx_format dmax2_frm[] = {
		{LNX_4_4_x, "dma_xor"},
		{LNX_4_14_x, "xor"},
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
