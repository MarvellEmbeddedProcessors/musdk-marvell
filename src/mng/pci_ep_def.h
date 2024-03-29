/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PCI_EP_DEF_H
#define _PCI_EP_DEF_H

#define NMP_PCI_MAX_NUM_BARS	(32) /* this is the maximum BARs we allow for now */

#define PCI_EP_UIO_MEM_NAME	"pci_ep"
#define PCI_EP_UIO_REGION_BAR0_NAME	"bar0"
#define PCI_EP_UIO_REGION_BAR2_NAME	"bar2"

#define PCI_CONFIG_BAR_ID	0

#define PCI_CONFIG_BAR_SIZE	(0x400)

#define PCI_BAR0_DEV_RES_BASE	(PCI_CONFIG_BAR_SIZE)
#define PCI_BAR0_DEV_RES_SIZE	(0xC00)

#define PCI_BAR0_MSI_X_TBL_BASE	(PCI_BAR0_DEV_RES_BASE + PCI_BAR0_DEV_RES_SIZE)
#define PCI_BAR0_MSI_X_TBL_SIZE	(0x2000)	/* 256 messages */

/* total size used on BAR; with some extra */
#define PCI_BAR0_CALC_SIZE	(PCI_BAR0_MSI_X_TBL_BASE + PCI_BAR0_MSI_X_TBL_SIZE)
#define PCI_BAR0_ALLOC_SIZE	(0x4000)
#define PCI_BAR0_ALLOC_ALIGN	(0x1000)

/* PCI BAR emulation definitions */
#define CFG_MEM_AP8xx_OFFS		(0xA0)

#define CFG_MEM_VALID			(1 << 0)
#define CFG_MEM_BIDX_MASK		(0x1F)
#define CFG_MEM_BIDX_SHIFT		(1)
#define CFG_MEM_BAR_MIN_ALIGN_BITS	(12)
#define CFG_MEM_BAR_MIN_ALIGN		(1 << CFG_MEM_BAR_MIN_ALIGN_BITS)
#define CFG_MEM_64B_HI_MAGIC_MASK	(0xFFFF << 16)
#define CFG_MEM_64B_HI_MAGIC_VAL	(0xCAFE << 16)
/* in 64bits reg, we allow address of 44bits with 12bits alignment */
#define CFG_MEM_64B_ADDR_MASK		\
	(0x00000FFFFFFFFFFFLL & ~(CFG_MEM_BAR_MIN_ALIGN - 1))
/* in 32bits reg, we allow address of 36bits with 12bits alignment */
#define CFG_MEM_32B_ADDR_MASK		\
	(0x0000000FFFFFFFFFLL & ~(CFG_MEM_BAR_MIN_ALIGN - 1))
#define CFG_MEM_32B_ADDR_SHIFT		(4)

#define NMP_EMUL_BARS_TBL_SIZE		(NMP_PCI_MAX_NUM_BARS * sizeof(u32))

#endif /* _PCI_EP_DEF_H */
