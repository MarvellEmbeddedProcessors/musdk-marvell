/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _PCI_EP_DEF_H
#define _PCI_EP_DEF_H

#define PCI_EP_UIO_MEM_NAME "pci_ep"
#define PCI_EP_UIO_REGION_NAME "bar0"

#define PCI_CONFIG_BAR_ID	0

#define PCI_BAR0_MNG_CH_BASE	(0)
#define PCI_BAR0_MNG_CH_SIZE	(0x1000)

#define PCI_BAR0_MQA_QNPT_BASE	(PCI_BAR0_MNG_CH_BASE + PCI_BAR0_MNG_CH_SIZE)
#define PCI_BAR0_MQA_QNPT_SIZE	(0x1000)

#define PCI_BAR0_MQA_QNCT_BASE	(PCI_BAR0_MQA_QNPT_BASE + PCI_BAR0_MQA_QNPT_SIZE)
#define PCI_BAR0_MQA_QNCT_SIZE	(0x1000)

#define PCI_BAR0_MSI_X_TBL_BASE (PCI_BAR0_MQA_QNCT_BASE + PCI_BAR0_MQA_QNCT_SIZE)
#define PCI_BAR0_MSI_X_TBL_SIZE (0x2000)	/* 128 CPUs * 2 interrupts per CPU) */

#define PCI_BAR0_MSI_X_PBA_BASE (PCI_BAR0_MSI_X_TBL_BASE + PCI_BAR0_MSI_X_TBL_SIZE)
#define PCI_BAR0_MSI_X_PBA_SIZE (PCI_BAR0_MSI_X_TBL_SIZE)

#define PCI_BAR0_CALC_SIZE	(PCI_BAR0_MSI_X_PBA_BASE + PCI_BAR0_MSI_X_PBA_SIZE)
#define PCI_BAR0_ALLOC_SIZE	(0x100000)
#define PCI_BAR0_ALLOC_ALIGN	(0x1000)

#endif /* _PCI_EP_DEF_H */
