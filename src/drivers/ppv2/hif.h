/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#ifndef __HIF_H__
#define __HIF_H__

#include "mv_std.h"

#include "mv_pp2_hif.h"


#define MVPP2_MAX_CPUS	9


struct mv_pp2x_hw {
	/* Shared registers' base addresses */
	void __iomem *base;	/* PPV22 base_address as received in
				 *devm_ioremap_resource().
				 */
	void __iomem *cpu_base[MVPP2_MAX_CPUS];
};

#endif /* __HIF_H__ */
