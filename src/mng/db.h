/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _DB_H
#define _DB_H

#include "std_internal.h"

#include "mng/mv_nmp.h"
#include "drivers/mv_mqa.h"
#include "drivers/mv_giu.h"
#include "drivers/mv_pp2.h"

struct uio_mem {
	void *phys_addr;
	void *virt_addr;
};

struct nmpp2 {
	int pp2_en;				/* Flag inidicating PP2 interface is present*/
	u16 bm_pool_reserved_map;
	u16 used_bpools[PP2_NUM_PKT_PROC];
};

/* Main PF data structure
 *
 *  nic-pf	all NIC-PF related data
 *
 *		Holds the mapping of the platform device configuration
 *		registers, associated with the platform device uio file.
 *  msi_regs	Mapping of the MSI-X registers.
 *		This ,apping is used for signaling the host if Ingress packets.
 *  giu		stores GIU related data
 *  mqa_global	MQA tables parameters
 */
struct nmp {
	struct sys_iomem *msi_iomem;
	struct uio_mem msi_regs;
	int *emul_bars_avail_tbl;
	void *emul_bars_mem;

	struct nmdisp *nmdisp;
	struct lf_mng *lf_mng;
	struct mqa *mqa;
	struct giu *giu;
	struct nmpp2 nmpp2;
};

#endif /* _DB_H */
