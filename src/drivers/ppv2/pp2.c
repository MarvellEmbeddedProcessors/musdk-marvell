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

/**
 * @file pp2.c
 *
 * PPDK container structures and packet processor initialization
 */

#include "std_internal.h"

#include "pp2_types.h"
#include "../../modules/include/mv_pp_uio.h"

#include "pp2.h"
#include "pp2_dm.h"
#include "pp2_port.h"
#include "pp2_bm.h"
#include "pp2_gop_dbg.h"
#include "pp2_hw_cls.h"
#include "cls/pp2_cls_mng.h"

struct pp2 *pp2_ptr;
struct netdev_if_params *netdev_params;

/* TBD: remove hc_gop_mac_data variable after port/mac is read from device tree*/
static struct pp2_mac_data hc_gop_mac_data[6] = {
	{
		.gop_index = 0,
		.flags = 0,
		.phy_addr = 0,
		.phy_mode = 18,
		.force_link = 0,
		.autoneg = 0,
		.link = 0,
		.duplex = 0,
		.speed = 0,
		.mac = {0, 0, 0, 0, 0, 1},
	},
	{
		.gop_index = 2,
		.flags = 0,
		.phy_addr = 0,
		.phy_mode = 7,
		.force_link = 0,
		.autoneg = 0,
		.link = 0,
		.duplex = 0,
		.speed = 0,
		.mac = {0, 0, 0, 0, 0, 2},
	},
	{
		.gop_index = 3,
		.flags = 0,
		.phy_addr = 0,
		.phy_mode = 7,
		.force_link = 0,
		.autoneg = 0,
		.link = 0,
		.duplex = 0,
		.speed = 0,
		.mac = {0, 0, 0, 0, 0, 3},
	},
	{
		.gop_index = 0,
		.flags = 0,
		.phy_addr = 0,
		.phy_mode = 18,
		.force_link = 0,
		.autoneg = 0,
		.link = 0,
		.duplex = 0,
		.speed = 0,
		.mac = {0, 0, 0, 0, 0, 4},
	},
	{
		.gop_index = 2,
		.flags = 0,
		.phy_addr = 0,
		.phy_mode = 7,
		.force_link = 0,
		.autoneg = 0,
		.link = 0,
		.duplex = 0,
		.speed = 0,
		.mac = {0, 0, 0, 0, 0, 5},
	},
	{
		.gop_index = 3,
		.flags = 0,
		.phy_addr = 0,
		.phy_mode = 7,
		.force_link = 0,
		.autoneg = 0,
		.link = 0,
		.duplex = 0,
		.speed = 0,
		.mac = {0, 0, 0, 0, 0, 6},
	}
};

static const char *pp2_id_uio_name(u8 pp2_id)
{
	static const char * const pp2_names[] = {UIO_PP_0, UIO_PP_1};

	if (pp2_id <= PP2_ID1)
		return pp2_names[pp2_id];

	pr_err("%s Invalid pp2_id(%d)\n", __func__, pp2_id);
	return NULL;
}

static void pp2_init_rxfhindir(struct pp2_inst *inst)
{
	/* Init RXFHINDIR table */
	for (u32 i = 0; i < MVPP22_RSS_TBL_LINE_NUM; i++)
		inst->rx_table[i] = i % PP2_NUM_CPUS;
}

/* Internal. Called once per packet processor initialization*/
static void pp2_bm_flush_pools(uintptr_t cpu_slot, uint16_t bm_pool_reserved_map)
{
	u32 pool_id;
	u32 resid_bufs = 0;

	/* Iterate through all the pools. Clean and reset registers */
	for (pool_id = 0; pool_id < PP2_BPOOL_NUM_POOLS; pool_id++) {
		if (bm_pool_reserved_map & (1 << pool_id))
			continue;
		/* Discard residual buffers */
		resid_bufs = pp2_bm_pool_flush(cpu_slot, pool_id);
		if (resid_bufs) {
			pr_warn("BM: could not clear all buffers from pool ID=%u\n", pool_id);
			pr_warn("BM: residual bufs : %u\n", resid_bufs);
		}
		/* Stop and clear pool internals */
		pp2_bm_hw_pool_destroy(cpu_slot, pool_id);
		/* Mask BM all interrupts */
		pp2_reg_write(cpu_slot, MVPP2_BM_INTR_MASK_REG(pool_id), 0x00);
		/* Clear BM cause register */
		pp2_reg_write(cpu_slot, MVPP2_BM_INTR_CAUSE_REG(pool_id), 0x00);
	}
	/* Disable the priority algorithm for buffer alloc/release */
	pp2_reg_write(cpu_slot, MVPP2_BM_PRIO_CTRL_REG, 0x00);

	pp2_reg_write(cpu_slot, MVPP22_BM_PHY_VIRT_HIGH_RLS_REG, 0x00);
}

/* Initializes a packet processor control handle and its resources */
static void pp2_inst_init(struct pp2_inst *inst)
{
	u32 val, i, rc;
	uintptr_t cpu_slot;
	struct pp2_hw *hw = &inst->hw;
	u32 admin_status;

	/* Master thread initializes common part of HW.
	* This will probably get deprecated by KS driver for the initialization
	* part, but keep it for now
	*/
	cpu_slot = hw->base[PP2_DEFAULT_REGSPACE].va;

	/* Clear BM */
	pp2_bm_flush_pools(cpu_slot, inst->parent->init.bm_pool_reserved_map);

	pp2_cls_mng_init(inst);

	/* Disable RXQs, only for ethernet ports (not loopback_port) */
	for (i = 0; i < PP2_NUM_ETH_PPIO; i++) {
		struct ppio_init_params	*ppio_param = &inst->parent->init.ppios[inst->id][i];

		if (!ppio_param->is_enabled)
			continue;

		rc = pp2_netdev_if_admin_status_get(inst->id, i, &admin_status);
		if (rc) {
			pr_warn("Unable to find admin status for ppio %d:%d. Set to disabled\n", inst->id, i);
			ppio_param->is_enabled = 0;
			continue;
		}

		if (admin_status != PP2_PORT_MUSDK && admin_status != PP2_PORT_SHARED) {
			pr_warn("Port %d:%d is not reserved for MUSDK usage in Linux DTS file\n", inst->id, i);
			ppio_param->is_enabled = 0;
			continue;
		}

		for (int j = 0; j < PP2_HW_PORT_NUM_RXQS; j++) {
			int rxq;

			if (j < ppio_param->first_inq)
				continue;
			rxq = i * PP2_HW_PORT_NUM_RXQS + j;
			val = pp2_reg_read(cpu_slot, MVPP2_RXQ_CONFIG_REG(rxq));
			val |= MVPP2_RXQ_DISABLE_MASK;
			pp2_reg_write(cpu_slot, MVPP2_RXQ_CONFIG_REG(rxq), val);
		}
	}

	/* GOP early activation */
	/* TODO: Revise after device tree adaptation */
}

static int pp2_get_hw_data(struct pp2_inst *inst)
{
	int err = 0;
	u32 reg_id, i;
	uintptr_t mem_base;
	struct pp2_hw *hw = &inst->hw;
	struct sys_iomem_params iomem_params;
	struct sys_iomem *pp2_sys_iomem;

	hw->tclk = PP2_TCLK_FREQ;

	iomem_params.type = SYS_IOMEM_T_UIO;
	iomem_params.devname = PP_UIO_MEM_NAME;
	iomem_params.index = inst->id;

	err = sys_iomem_init(&iomem_params, &pp2_sys_iomem);
	inst->pp2_sys_iomem = pp2_sys_iomem;
	if (err) {
		pr_err(" No device found\n");
		return err;
	}

	/* Map the whole physical Packet Processor physical address */
	err = sys_iomem_map(pp2_sys_iomem, "pp", &hw->phy_address_base, (void **)(&mem_base));
	if (err) {
		sys_iomem_deinit(pp2_sys_iomem);
		return err;
	}

	/* Assign each CPU (thread) slot its mapped address space. */

	for (reg_id = 0; reg_id < ARRAY_SIZE(hw->base); reg_id++)
		hw->base[reg_id].va = mem_base + (reg_id * PP2_REGSPACE_SIZE);

	err = sys_iomem_map(pp2_sys_iomem, "serdes", &hw->gop.serdes.base.pa, (void **)(&mem_base));
	if (err) {
		sys_iomem_unmap(pp2_sys_iomem, "pp");
		sys_iomem_deinit(pp2_sys_iomem);
		return err;
	}
	hw->gop.serdes.base.va = mem_base;
	hw->gop.serdes.obj_size = 0x1000;

	err = sys_iomem_map(pp2_sys_iomem, "xmib", &hw->gop.xmib.base.pa, (void **)(&mem_base));
	if (err) {
		sys_iomem_unmap(pp2_sys_iomem, "serdes");
		sys_iomem_unmap(pp2_sys_iomem, "pp");
		sys_iomem_deinit(pp2_sys_iomem);
		return err;
	}
	hw->gop.xmib.base.va = mem_base;
	hw->gop.xmib.obj_size = 0x0100;

	err = sys_iomem_map(pp2_sys_iomem, "smi", &hw->gop.smi.pa, (void **)(&mem_base));
	if (err) {
		sys_iomem_unmap(pp2_sys_iomem, "xmib");
		sys_iomem_unmap(pp2_sys_iomem, "serdes");
		sys_iomem_unmap(pp2_sys_iomem, "pp");
		sys_iomem_deinit(pp2_sys_iomem);
		return err;
	}
	hw->gop.smi.va = mem_base;
	hw->gop.smi.va += 0x200;
	hw->gop.smi.pa += 0x200;

	err = sys_iomem_map(pp2_sys_iomem, "mspg", &hw->gop.mspg.pa, (void **)(&mem_base));
	if (err) {
		sys_iomem_unmap(pp2_sys_iomem, "smi");
		sys_iomem_unmap(pp2_sys_iomem, "xmib");
		sys_iomem_unmap(pp2_sys_iomem, "serdes");
		sys_iomem_unmap(pp2_sys_iomem, "pp");
		sys_iomem_deinit(pp2_sys_iomem);
		return err;
	}
	hw->gop.mspg.va = mem_base;

	err = sys_iomem_map(pp2_sys_iomem, "rfu1", &hw->gop.rfu1.pa, (void **)(&mem_base));
	if (err) {
		sys_iomem_unmap(pp2_sys_iomem, "mspg");
		sys_iomem_unmap(pp2_sys_iomem, "smi");
		sys_iomem_unmap(pp2_sys_iomem, "xmib");
		sys_iomem_unmap(pp2_sys_iomem, "serdes");
		sys_iomem_unmap(pp2_sys_iomem, "pp");
		sys_iomem_deinit(pp2_sys_iomem);
		return err;
	}
	hw->gop.rfu1.va = mem_base;

	/**
	* Only memory maps aligned with PAGE_SIZE (ARM64 arch 0x1000) can be
	* mapped. Hence, the registers base address lower than PAGE_SIZE
	* alignment will be computed here and not extracted from device tree.
	*/
	hw->gop.xsmi.va = hw->gop.smi.va + 0x400;
	hw->gop.xsmi.pa = hw->gop.smi.pa + 0x400;

	hw->gop.xpcs.va = hw->gop.mspg.va + 0x400;
	hw->gop.xpcs.pa = hw->gop.mspg.pa + 0x400;

	hw->gop.ptp.base.va = hw->gop.mspg.va + 0x800;
	hw->gop.ptp.base.pa = hw->gop.mspg.pa + 0x800;
	hw->gop.ptp.obj_size = 0x1000;

	hw->gop.gmac.base.va = hw->gop.mspg.va + 0xE00;
	hw->gop.gmac.base.pa = hw->gop.mspg.pa + 0xE00;
	hw->gop.gmac.obj_size = 0x1000;

	hw->gop.xlg_mac.base.va = hw->gop.mspg.va + 0xF00;
	hw->gop.xlg_mac.base.pa = hw->gop.mspg.pa + 0xF00;
	hw->gop.xlg_mac.obj_size = 0x1000;

	/* Get MAC data for all available ethernet ports (not loopback port) based on dts GOP entries */
	/* TODO: Revise this after GOP dev tree support */
	for (i = 0; i < PP2_NUM_ETH_PPIO; i++) {
		struct pp2_port *port = inst->ports[i];
		struct pp2_mac_data *mac = &port->mac_data;
		u32 id = port->id + (inst->id * PP2_NUM_ETH_PPIO);

		/* TBD(DevTree): replace with data read from Device tree */
		mac->gop_index = hc_gop_mac_data[id].gop_index;
		mac->flags     = hc_gop_mac_data[id].flags;
		mac->phy_addr  = hc_gop_mac_data[id].phy_addr;
		mac->phy_mode  = hc_gop_mac_data[id].phy_mode;
		mac->force_link = hc_gop_mac_data[id].force_link;
		mac->autoneg   = hc_gop_mac_data[id].autoneg;
		mac->link      = hc_gop_mac_data[id].link;
		mac->duplex    = hc_gop_mac_data[id].duplex;
		mac->speed     = hc_gop_mac_data[id].speed;
		memcpy(&mac->mac, &hc_gop_mac_data[id].mac, PP2_ETHADDR_LEN);
	}

	return err;
}

static struct pp2_inst *pp2_inst_create(struct pp2 *pp2, uint32_t pp2_id)
{
	u32 i;
	struct pp2_inst *inst;

	if (unlikely(!pp2)) {
		pr_err("Invalid ppdk handle\n");
		return NULL;
	}

	inst = kcalloc(1, sizeof(struct pp2_inst), GFP_KERNEL);
	if (unlikely(!inst)) {
		pr_err("%s out of memory pp2_inst alloc\n", __func__);
		return NULL;
	}
	memset(inst, 0, sizeof(struct pp2_inst));

	/* Early allocate and get MAC data for available ports since GOP
	* sub-system needs to be initialized once per packet processor
	* Later, when ports get opened/started, only per-MAC
	* initializations shall be done
	*/
	for (i = 0 ; i < PP2_NUM_PORTS; i++) {
		struct pp2_port *port = kcalloc(1, sizeof(struct pp2_port), GFP_KERNEL);

		if (unlikely(!port)) {
			pr_err("%s out of memory pp2_port alloc\n", __func__);
			break;
		}
		/* Static ID assignment */
		port->id = i;
		inst->ports[i] = port;
	}

	inst->parent = pp2;
	inst->id = pp2_id;

	/* Get static device tree data */
	if (pp2_get_hw_data(inst)) {
		pr_err("cannot populate hardware data\n");
		for (i = 0; i < PP2_NUM_PORTS; i++)
			kfree(inst->ports[i]);
		kfree(inst);
		return NULL;
	}

	return inst;
}

u8 pp2_get_num_inst(void)
{
	u8 pp2_num_inst = 0;
	struct sys_iomem_params  iomem_params;

	iomem_params.type = SYS_IOMEM_T_UIO;
	iomem_params.devname = PP_UIO_MEM_NAME;

	iomem_params.index = PP2_ID0;
	pp2_num_inst += sys_iomem_exists(&iomem_params);
	iomem_params.index = PP2_ID1;
	pp2_num_inst += sys_iomem_exists(&iomem_params);

	pr_debug("pp2_num_inst=%d\n", pp2_num_inst);
	return pp2_num_inst;
}

static void pp2_destroy(struct pp2_inst *inst)
{
	u32 i;

	sys_iomem_unmap(inst->pp2_sys_iomem, "pp");
	sys_iomem_unmap(inst->pp2_sys_iomem, "serdes");
	sys_iomem_unmap(inst->pp2_sys_iomem, "xmib");
	sys_iomem_unmap(inst->pp2_sys_iomem, "smi");
	sys_iomem_unmap(inst->pp2_sys_iomem, "mspg");
	sys_iomem_unmap(inst->pp2_sys_iomem, "rfu1");
	sys_iomem_deinit(inst->pp2_sys_iomem);

	/* No dangling handles */
	for (i = 0; i < PP2_NUM_PORTS; i++)
		kfree(inst->ports[i]);

	for (i = 0; i < PP2_NUM_REGSPACES; i++)
		kfree(inst->dm_ifs[i]);

	for (i = 0; i < PP2_BPOOL_NUM_POOLS; i++)
		kfree(inst->bm_pools[i]);

	kfree(inst);
}

/* Check ppio status configured by applications is coherent with dts file */
static int pp2_status_check(struct netdev_if_params *netdev_params, struct pp2_init_params *params)
{
	int i, j;

	for (i = 0; i < PP2_NUM_PKT_PROC; i++) {
		for (j = 0; j < PP2_NUM_ETH_PPIO; j++) {
			if ((netdev_params[i * PP2_NUM_ETH_PPIO + j].admin_status == PP2_PORT_DISABLED) &&
			    (params->ppios[i][j].is_enabled == 1)) {
				pr_err("configured status does not match dts file for ppio %d:%d\n", i, j);
				return -EFAULT;
			}
		}
	}
	return 0;
}

int pp2_init(struct pp2_init_params *params)
{
	u32 pp2_id, pp2_num_inst;
	struct pp2_ppio_params lb_port_params;
	int rc;

	pp2_ptr = kcalloc(1, sizeof(struct pp2), GFP_KERNEL);
	if (unlikely(!pp2_ptr)) {
		pr_err("%s out of memory pp2 alloc\n", __func__);
		return -ENOMEM;
	}
	memcpy(&pp2_ptr->init, params, sizeof(*params));
	pp2_ptr->pp2_common.hif_slot_map = 0;
	pp2_ptr->pp2_common.rss_tbl_map  = 0;
	/* TODO: Check first_inq params are valid */

	/* Retrieve netdev if information */
	pp2_num_inst = pp2_get_num_inst();
	netdev_params = kmalloc(sizeof(*netdev_params) * pp2_num_inst * PP2_NUM_ETH_PPIO, GFP_KERNEL);
	if (!netdev_params)
		return -ENOMEM;

	memset(netdev_params, 0, sizeof(*netdev_params) * pp2_num_inst * PP2_NUM_ETH_PPIO);
	pp2_netdev_if_info_get(netdev_params);

	rc = pp2_status_check(netdev_params, params);
	if (rc)
		return -EFAULT;

	/* Initialize in an opaque manner from client,
	* depending on HW, one or two packet processors.
	*/
	for (pp2_id = 0; pp2_id < pp2_num_inst; pp2_id++) {
		struct pp2_inst *inst;

		inst = pp2_inst_create(pp2_ptr, pp2_id);
		if (!inst) {
			pr_err("cannot create PP%u\n", pp2_id);

			if (pp2_id == PP2_ID1) {
				/* Also destroy the previous instance */
				pp2_destroy(pp2_ptr->pp2_inst[PP2_ID0]);
			}
			kfree(pp2_ptr);

			return -ENOMEM;
		}
		/* Store the PPDK handle as parent and store
		 * this instance as child handle for the PPDK
		 */
		pp2_ptr->pp2_inst[pp2_id] = inst;

		/* Initialize this packet processor */
		pp2_inst_init(inst);
		pp2_ptr->num_pp2_inst++;
	}

	pr_debug("PackProcs   %2u\n", pp2_num_inst);
	lb_port_params.type = PP2_PPIO_T_NIC;
	lb_port_params.inqs_params.num_tcs = 0;
	lb_port_params.outqs_params.num_outqs = PP2_LPBK_PORT_NUM_TXQ;
	lb_port_params.outqs_params.outqs_params[0].size = PP2_LPBK_PORT_TXQ_SIZE;

	/* Initialize the loopback port */
	for (pp2_id = 0; pp2_id < pp2_num_inst; pp2_id++) {
		struct pp2_port *port;

		rc = pp2_port_open(pp2_ptr, &lb_port_params, pp2_id, PP2_LOOPBACK_PORT, &port);
		if (rc) {
			pr_err("[%s] ppio init failed.\n", __func__);
			return(-EFAULT); /* TODO: pp2_destroy on error (currently error_ret not possible) */
		}
		pp2_port_config_outq(port);
		pp2_port_start(port, PP2_TRAFFIC_EGRESS);
	}

	return 0;
}

void pp2_deinit(void)
{
	for (u32 pp2_id = 0; pp2_id < pp2_ptr->num_pp2_inst; pp2_id++) {
		struct pp2_inst *inst = pp2_ptr->pp2_inst[pp2_id];
		struct pp2_port *lpbk_port;

		lpbk_port = inst->ports[PP2_LOOPBACK_PORT];
		/* Close internal loopback port */
		pp2_port_stop(lpbk_port);
		pp2_port_close(lpbk_port);

		/* Deinit cls manager */
		pp2_cls_mng_deinit(inst);

		pp2_destroy(inst);
	}

	/* Destroy the PPDK handle */
	kfree(pp2_ptr);

	/* Destroy the netdev handle */
	kfree(netdev_params);
}

int pp2_netdev_ifname_get(u32 pp_id, u32 ppio_id, char *ifname)
{
	int i;

	if (!netdev_params)
		return -EFAULT;

	for (i = 0 ; i < PP2_MAX_NUM_PACKPROCS * PP2_NUM_ETH_PPIO; i++) {
		if (netdev_params[i].pp_id == pp_id &&
		    netdev_params[i].ppio_id == ppio_id) {
			strcpy(ifname, netdev_params[i].if_name);
			return 0;
		}
	}
	return -EFAULT;
}

int pp2_netdev_if_admin_status_get(u32 pp_id, u32 ppio_id, u32 *admin_status)
{
	int i;

	if (!netdev_params)
		return -EFAULT;

	for (i = 0 ; i < PP2_MAX_NUM_PACKPROCS * PP2_NUM_ETH_PPIO; i++) {
		if (netdev_params[i].pp_id == pp_id &&
		    netdev_params[i].ppio_id == ppio_id) {
			*admin_status = netdev_params[i].admin_status;
			return 0;
		}
	}
	return -EFAULT;
}

/* pp2_netdev_get_port_info()
 * Find  pp_id and port_id parameters from ifname.
 * Description: loop through all packet processors and ports in each packet processor
 * and compare the interface name to the one configured for each port. If there is a match,
 * the pp_id and port_id are returned.
 * This function should be called after pp2_init() and before ppio_init().
 */
int pp2_netdev_get_port_info(char *ifname, u8 *pp_id, u8 *ppio_id)
{
	int i;

	if (!netdev_params)
		return -EFAULT;

	for (i = 0 ; i < PP2_MAX_NUM_PACKPROCS * PP2_NUM_ETH_PPIO; i++) {
		if (strcmp(netdev_params[i].if_name, ifname) == 0) {
			*pp_id = netdev_params[i].pp_id;
			*ppio_id = netdev_params[i].ppio_id;
			pr_info("%s: ppio-%d,%d\n", ifname, *pp_id, *ppio_id);
			return 0;
		}
	}
	return -EFAULT;
}

