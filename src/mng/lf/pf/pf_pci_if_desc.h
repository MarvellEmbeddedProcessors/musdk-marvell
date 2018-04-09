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
#pragma pack(1)
struct q_hw_info {
	u64	q_addr;
	u64	consumer_idx_addr;
	u64	producer_idx_addr;
	u32	len;
	u32	res;
};

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
};
#pragma pack()

#endif /* _HOST_PCI_IF_H_ */
