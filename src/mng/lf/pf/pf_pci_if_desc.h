/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _HOST_PCI_IF_H_
#define _HOST_PCI_IF_H_

/*
** Configuration space definition
*/

/*
** configuration structure defined by the device, and used for
** initial communication between the host driver and the NIC.
** The below is still preliminary, till we finalize the interface between
** the host and the NIC.
** - q_addr: Physical address of the queue in host's memory.
** - consumer_idx_addr: Physical address of the queue consumer index.
** - producer_idx_addr: Physical address of the queue producer index.
** - len: Number of elements in the queue.
*/
struct q_hw_info {
	u64	q_addr;
	u64	consumer_idx_addr;
	u64	producer_idx_addr;
	u32	len;
	u32	res;
} __packed;

struct pcie_config_mem {
#define PCIE_CFG_STATUS_DEV_READY	(1 << 0)
#define PCIE_CFG_STATUS_HOST_MGMT_READY	(1 << 1)
#define PCIE_CFG_STATUS_DEV_MGMT_READY	(1 << 2)
	u32	status;
	u8	mac_addr[6];
	u8	res[6];
	struct q_hw_info cmd_q;
	struct q_hw_info notif_q;
	/* Meanwhile, assume SNIC's notification table is part of BAR-0.
	 * This is actually the offset of the prod / cons notification tables
	 * inside BAR0.
	 * Value N, means that the respective notification table starts at
	 * offset N-Bytes from the beginning of BAR-0.
	 * Consecutively, the _size parameter holds the size in bytes of the
	 * notification tables.
	 */
	u32	cons_notif_tbl_offset;
	u32	cons_notif_tbl_size;
	u32	prod_notif_tbl_offset;
	u32	prod_notif_tbl_size;

	u32	remote_index_location;

	/* MSI-X table offset at BAR0 */
	u32	msi_x_tbl_offset;
} __packed;

#endif /* _HOST_PCI_IF_H_ */
