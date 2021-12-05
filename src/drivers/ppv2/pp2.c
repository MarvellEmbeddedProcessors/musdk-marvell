/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
#include "cls/pp2_prs.h"
#include "cls/pp2_hw_cls.h"
#include "cls/pp2_cls_mng.h"


struct pp2 *pp2_ptr;
struct netdev_if_params netdev_params[PP2_MAX_NUM_PACKPROCS * PP2_NUM_ETH_PPIO];



struct pp2_lnx_format pp2_frm[] = {
				{
					.ver = LNX_4_4_x,
					.devtree_path = "/proc/device-tree/cp%u/config-space/ppv22@000000/",
					.eth_format = "eth%d@0%d0000",
				},
				{
					.ver = LNX_4_14_x,
					.devtree_path = "/proc/device-tree/cp%u/config-space/ethernet@0/",
					.eth_format = "eth%d",
				},
				{
					.ver = LNX_OTHER,
					.devtree_path = "/proc/device-tree/cp%u/config-space@%x/ethernet@0/",
					.eth_format = "eth%d",
				}
};


/* TBD: remove hc_gop_mac_data variable after port/mac is read from device tree*/
static struct pp2_mac_data hc_gop_mac_data[12] = {
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
		.mac = {0, 0, 0, 0, 0, 7},
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
		.mac = {0, 0, 0, 0, 0, 8},
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
		.mac = {0, 0, 0, 0, 0, 9},
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
		.mac = {0, 0, 0, 0, 0, 0xa},
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
		.mac = {0, 0, 0, 0, 0, 0xb},
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
		.mac = {0, 0, 0, 0, 0, 0xc},
	}

};



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
void pp2_inst_init(struct pp2_inst *inst)
{
		uintptr_t cpu_slot;
	struct pp2_hw *hw = &inst->hw;

	/* Master thread initializes common part of HW.
	* This will probably get deprecated by KS driver for the initialization
	* part, but keep it for now
	*/
	cpu_slot = hw->base[PP2_DEFAULT_REGSPACE].va;

	if (!inst->skip_hw_init) {
		/* Clear BM */
		pp2_bm_flush_pools(cpu_slot, inst->parent->init.bm_pool_reserved_map);

		pp2_cls_mng_init(inst);
	}

	/* GOP early activation */
	/* TODO: Revise after device tree adaptation */
}

static int pp2_get_hw_data(struct pp2_inst *inst)
{
	int err = 0;
	u32 i, reg_id;
	uintptr_t mem_base;
	struct pp2_hw *hw = &inst->hw;
	struct sys_iomem_params iomem_params;
	struct sys_iomem *pp2_sys_iomem;

	hw->tclk = PP2_TCLK_FREQ;
	iomem_params.type = SYS_IOMEM_T_UIO;
	iomem_params.devname = UIO_PP2_STRING;
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


	err = sys_iomem_map(pp2_sys_iomem, "mspg", &hw->gop.mspg.pa, (void **)(&mem_base));
	if (err) {
		sys_iomem_unmap(pp2_sys_iomem, "pp");
		sys_iomem_deinit(pp2_sys_iomem);
		return err;
	}
	hw->gop.mspg.va = mem_base;

	/* Map the Cm3 physical address */
	err = sys_iomem_map(pp2_sys_iomem, "cm3", &hw->cm3_base.pa, (void **)(&mem_base));
	if (err) {
		/* Not all systems support cm3 */
		pr_warn("tx_pause not supported\n");
		err = 0;
	} else {
		hw->cm3_base.va = mem_base;
	}

	/**
	* Only memory maps aligned with PAGE_SIZE (ARM64 arch 0x1000) can be
	* mapped. Hence, the registers base address lower than PAGE_SIZE
	* alignment will be computed here and not extracted from device tree.
	*/

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

struct pp2_inst *pp2_inst_create(struct pp2 *pp2, uint32_t pp2_id)
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
	u8 i, pp2_num_inst = 0;
	struct sys_iomem_params  iomem_params;

	iomem_params.type = SYS_IOMEM_T_UIO;
	iomem_params.devname = UIO_PP2_STRING;
	for (i = 0; i < PP2_MAX_NUM_PACKPROCS; i++) {
		iomem_params.index = i;
		pp2_num_inst += sys_iomem_exists(&iomem_params);
	}
	pr_debug("pp2_num_inst=%d\n", pp2_num_inst);

	return pp2_num_inst;
}

/* TODO: Add call to hw_registers, after support is added. */
#define MV_PP2_NUM_HIFS_RSRV	4
#define MV_PP2_HIFS_RSRV_MASK	((1 << MV_PP2_NUM_HIFS_RSRV) - 1)

u16 pp2_get_used_hif_map(void)
{
	uintptr_t cpu_slot;
	u32 hif_map;
	struct pp2_inst *pp2_first_inst = pp2_ptr->pp2_inst[0];

	cpu_slot = pp2_first_inst->hw.base[PP2_DEFAULT_REGSPACE].va;
	hif_map = pp2_reg_read(cpu_slot, MVPP22_HIF_ALLOCATION_REG);
	hif_map = (hif_map) ? hif_map : MV_PP2_HIFS_RSRV_MASK;

	pr_info("%s: hif_map(0x%x)\n", __func__, hif_map);
	return hif_map;
}


/* Number of BM pools reserved by kernel */
#define PP2_NUM_BPOOLS_RSRV	3
/* Reserved BM pools mask */
#define PP2_BPOOLS_RSRV_MASK		((1 << PP2_NUM_BPOOLS_RSRV) - 1)
u16 pp2_get_used_bm_pool_map(void)
{
	u32 bm_pool_map;

	/* TODO : Replace with pp2_reg_read that will read actual bm_map,
	 *        after supported in kernel is added
	 */
	bm_pool_map = PP2_BPOOLS_RSRV_MASK;

	pr_info("%s: bm_pool_map(0x%x)\n", __func__, bm_pool_map);
	return bm_pool_map;
}


void pp2_destroy(struct pp2_inst *inst)
{
	u32 i;

	sys_iomem_unmap(inst->pp2_sys_iomem, "pp");
	sys_iomem_unmap(inst->pp2_sys_iomem, "mspg");
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
int pp2_ppio_available(int pp_id, int ppio_id)
{
	u32 admin_status;
	int err;


	err = pp2_netdev_if_admin_status_get(pp_id, ppio_id, &admin_status);

	if (!err && (admin_status != PP2_PORT_DISABLED))
		return true;
	return false;
}

int pp2_ppio_get_l4_cksum_max_frame_size(int pp_id, int ppio_id, uint16_t *max_frame_size)
{
	uintptr_t cpu_slot;
	struct pp2_inst *pp2_inst = pp2_ptr->pp2_inst[pp_id];

	cpu_slot = pp2_inst->hw.base[PP2_DEFAULT_REGSPACE].va;
	*max_frame_size = (MVPP22_TX_FIFO_THRESH_MASK & pp2_reg_read(cpu_slot, MVPP22_TX_FIFO_THRESH_REG(ppio_id)));

	return 0;
}

void pp2_deinit(void)
{
	u32 pp2_id;

	for (pp2_id = 0; pp2_id < pp2_ptr->num_pp2_inst; pp2_id++) {
		struct pp2_inst *inst = pp2_ptr->pp2_inst[pp2_id];
		struct pp2_port *lpbk_port;

		if (!inst->skip_hw_init) {
			lpbk_port = inst->ports[PP2_LOOPBACK_PORT];
			/* Close internal loopback port */
			pp2_port_stop(lpbk_port);
			pp2_port_close(lpbk_port);
			/* Deinit cls manager */
			pp2_cls_mng_deinit(inst);
		}
		pp2_destroy(inst);
	}

	/* Destroy the PPDK handle */
	kfree(pp2_ptr);

	pp2_ptr = NULL;
}

int pp2_netdev_get_ifname(u8 pp_id, u8 ppio_id, char *ifname)
{
	int i;

	/* Retrieve netdev if information, only for first time */
	pp2_netdev_if_info_get(netdev_params);

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

	/* Retrieve netdev if information, only for first time */
	pp2_netdev_if_info_get(netdev_params);

	for (i = 0 ; i < PP2_MAX_NUM_PACKPROCS * PP2_NUM_ETH_PPIO; i++) {
		if (netdev_params[i].pp_id == pp_id &&
		    netdev_params[i].ppio_id == ppio_id) {
			*admin_status = netdev_params[i].admin_status;
			return 0;
		}
	}
	return -EFAULT;
}

/* pp2_netdev_get_ppio_info()
 * Find  pp_id and port_id parameters from ifname.
 * Description: loop through all packet processors and ports in each packet processor
 * and compare the interface name to the one configured for each port. If there is a match,
 * the pp_id and port_id are returned.
 * This function should be called after pp2_init() and before ppio_init().
 */
int pp2_netdev_get_ppio_info(char *ifname, u8 *pp_id, u8 *ppio_id)
{
	int i;

	/* Retrieve netdev if information, only for first time */
	pp2_netdev_if_info_get(netdev_params);

	for (i = 0 ; i < PP2_MAX_NUM_PACKPROCS * PP2_NUM_ETH_PPIO; i++) {
		if (strcmp(netdev_params[i].if_name, ifname) == 0) {
			*pp_id = netdev_params[i].pp_id;
			*ppio_id = netdev_params[i].ppio_id;
			pr_debug("%s: %s: ppio-%d,%d\n", __func__, ifname, *pp_id, *ppio_id);
			return 0;
		}
	}
	return -EFAULT;
}

int pp2_init(struct pp2_init_params *params)
{
	u32 pp2_id, lp_pp2_id, pp2_num_inst, i;
	int rc;

	pp2_ptr = kcalloc(1, sizeof(struct pp2), GFP_KERNEL);
	if (unlikely(!pp2_ptr)) {
		pr_err("%s out of memory pp2 alloc\n", __func__);
		return -ENOMEM;
	}
	memcpy(&pp2_ptr->init, params, sizeof(*params));
	pp2_ptr->pp2_common.hif_slot_map = 0;
	pp2_ptr->pp2_common.rss_tbl_map = params->rss_tbl_reserved_map;
	/* TODO: Check first_inq params are valid */

	pp2_num_inst = pp2_get_num_inst();

	/* Initialize in an opaque manner from client,
	* depending on HW, one or two packet processors.
	*/
	for (pp2_id = 0; pp2_id < pp2_num_inst; pp2_id++) {
		struct pp2_inst *inst;

		inst = pp2_inst_create(pp2_ptr, pp2_id);
		if (!inst) {
			pr_err("cannot create PP%u\n", pp2_id);
			rc = -ENOMEM;
			goto pp2_init_err;
		}
		/* Store the PPDK handle as parent and store
		 * this instance as child handle for the PPDK
		 */
		pp2_ptr->pp2_inst[pp2_id] = inst;

		/* Initialize this packet processor */
		inst->skip_hw_init = params->skip_hw_init;

		/* Retrieve reserved_maps for auto_detect requests, only required to perform once.
		 * bm_pool_map must be valid before call to pp2_inst_init().
		 */
		if (pp2_id == 0) {
			if (pp2_ptr->init.res_maps_auto_detect_map & PP2_RSRVD_MAP_HIF_AUTO) {
				pp2_ptr->init.hif_reserved_map = pp2_get_used_hif_map();
				params->hif_reserved_map = pp2_ptr->init.hif_reserved_map;
			}
			if (pp2_ptr->init.res_maps_auto_detect_map & PP2_RSRVD_MAP_BM_POOL_AUTO) {
				pp2_ptr->init.bm_pool_reserved_map = pp2_get_used_bm_pool_map();
				params->bm_pool_reserved_map = pp2_ptr->init.bm_pool_reserved_map;
			}
		}

		pp2_inst_init(inst);

		if (params->prs_udfs.num_udfs > 0) {
			rc = pp2_prs_udf_init(inst, &params->prs_udfs);
			if (rc) {
				pr_err("[%s] failed to init parser udfs.\n", __func__);
				rc = -EFAULT;
				goto pp2_init_err;
			}
		}

		pp2_ptr->num_pp2_inst++;
	}

	pr_debug("PackProcs   %2u\n", pp2_num_inst);
	if (!params->skip_hw_init) {
		struct pp2_ppio_params *lb_port_params;

		lb_port_params = kmalloc(sizeof(struct pp2_ppio_params), GFP_KERNEL);
		if (!lb_port_params) {
			rc = -ENOMEM;
			goto pp2_init_err;
		}

		memset(lb_port_params, 0, sizeof(*lb_port_params));
		lb_port_params->type = PP2_PPIO_T_NIC;
		lb_port_params->inqs_params.num_tcs = 0;
		lb_port_params->outqs_params.num_outqs = PP2_LPBK_PORT_NUM_TXQ;
		lb_port_params->outqs_params.outqs_params[0].size = PP2_LPBK_PORT_TXQ_SIZE;

		/* Initialize the loopback port */
		for (lp_pp2_id = 0; lp_pp2_id < pp2_num_inst; lp_pp2_id++) {
			struct pp2_port *port;

			rc = pp2_port_open(pp2_ptr, lb_port_params, lp_pp2_id, PP2_LOOPBACK_PORT, &port);
			if (rc) {
				pr_err("[%s] ppio init failed.\n", __func__);
				kfree(lb_port_params);
				rc = -EFAULT;
				/* TODO: in case of error during the loop, need to revert pp2_port_config_outq and
				 * pp2_port_start
				 */
				goto pp2_init_err;
			}
			pp2_port_config_outq(port);
			pp2_port_start(port, PP2_TRAFFIC_EGRESS);
		}
		kfree(lb_port_params);
	}

	return 0;

pp2_init_err:
	/* Rollback creation of pp2 instances */
	for (i = 0; i < pp2_id; i++)
		pp2_destroy(pp2_ptr->pp2_inst[i]);
	kfree(pp2_ptr);
	return rc;
}

int pp2_is_sysfs_avail(void)
{
	enum musdk_lnx_id lnx_id = lnx_id_get();

	return (!lnx_is_mainline(lnx_id));
}

