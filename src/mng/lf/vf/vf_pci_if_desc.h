/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _HOST_PCI_IF_H_
#define _HOST_PCI_IF_H_

#include "std_internal.h"

/*
** Configuration space definition
*/

/*
** configuration structure defined by the device, and used for
** initial communication between the host driver and the NIC.
** The below is still preliminary, till we finalize the interface between
** the host and the NIC.
** - q_addr: Physical address of the queue in host's memory.
** - q_prod_offs: Producer offset from BAR.
** - q_cons_offs: Consumer offset from BAR.
** - len: Number of elements in the queue.
*/
struct q_hw_info {
	u64	q_addr;
	u32	q_prod_offs;
	u32	q_cons_offs;
	u32	len;
	u32	res;
} __packed;

#define PCI_EP_HOST_MSIX_BASE_ADDR	0x8400000000LL
#define PCI_EP_VF_HOST_MSIX_SIZE	0x100000000LL
#define PCI_EP_VF_HOST_MSIX_BASE_ADDR(vf_id) (PCI_EP_HOST_MSIX_BASE_ADDR + vf_id * PCI_EP_VF_HOST_MSIX_SIZE)
#define PCI_EP_VF_HOST_MSIX_NUM		128
#define PCI_EP_VF_HOST_MSIX_MASK_SIZE	32
#define PCI_EP_VF_HOST_MSIX_GET_MASK_ARR_INDEX(msix_id) \
	(msix_id / PCI_EP_VF_HOST_MSIX_MASK_SIZE)
#define PCI_EP_VF_HOST_MSIX_GET_MASK(msix_id) \
	(BIT(msix_id % PCI_EP_VF_HOST_MSIX_MASK_SIZE))

struct pcie_config_mem {
#define PCIE_CFG_STATUS_DEV_READY	(1 << 0)
#define PCIE_CFG_STATUS_HOST_MGMT_READY	(1 << 1)
#define PCIE_CFG_STATUS_DEV_MGMT_READY	(1 << 2)
#define PCIE_CFG_STATUS_HOST_MGMT_CLOSE_REQ	(1 << 29)
#define PCIE_CFG_STATUS_HOST_MGMT_CLOSE_DONE	(1 << 30)
#define PCIE_CFG_STATUS_HOST_RESET	(1 << 31)
	u32	status;
	u32	vf_id;
	u8	mac_addr[6];
	u8	res[2];
	struct q_hw_info cmd_q;
	struct q_hw_info notif_q;
	u8	res2[24];

	u32	dev_use_size;
	/* MSI-X table offset at BAR0 */
	u32	msi_x_tbl_offset;
	u32	msi_x_mask[ceil(PCI_EP_VF_HOST_MSIX_NUM, PCI_EP_VF_HOST_MSIX_MASK_SIZE)];

	u8	res3[896]; /* complete to 1KB */
} __packed;

#endif /* _HOST_PCI_IF_H_ */
