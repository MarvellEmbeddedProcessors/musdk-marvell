/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __HIF_H__
#define __HIF_H__

#include "drivers/mv_pp2_hif.h"

struct pp2_hif {
	int regspace_slot;
	struct pp2_ppio_desc *rel_descs;
};

#if 0
struct pp2_io_base_addr {
	uintptr_t va;
};

struct mv_pp2x_hw {
	/* Shared registers' base addresses */
	void __iomem *base;	/* PPV22 base_address as received in
				 *devm_ioremap_resource().
				 */
	void __iomem *cpu_base[MVPP2_MAX_CPUS];
};
#endif

#endif /* __HIF_H__ */
