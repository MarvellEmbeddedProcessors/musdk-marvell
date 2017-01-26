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

struct pp2 * pp2_ptr;

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


static const char * pp2_id_uio_name(u8 pp2_id)
{
	static const char *pp2_names[] = {UIO_PP_0, UIO_PP_1};

	if (pp2_id <= PP2_ID1)
		return pp2_names[pp2_id];

	pp2_err("%s Invalid pp2_id(%d)\n", __func__, pp2_id);
	return NULL;
}

static void pp2_init_rxfhindir(struct pp2_inst *inst)
{
	/* Init RXFHINDIR table */
	for (uint32_t i = 0; i < MVPP22_RSS_TBL_LINE_NUM; i++)
		inst->rx_table[i] = i % PP2_NUM_CPUS;
}

/* Internal. Called once per packet processor initialization*/
static void pp2_bm_flush_pools(uintptr_t cpu_slot, uint16_t bm_pool_reserved_map)
{
    uint32_t pool_id;

    /* Iterate through all the pools. Clean and reset registers */
    for (pool_id = 0; pool_id < PP2_NUM_BMPOOLS; pool_id++) {
        if (bm_pool_reserved_map & (1 << pool_id))
            continue;
        /* Discard residual buffers */
        pp2_bm_pool_flush(cpu_slot, pool_id);
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
    uint32_t val, i;
    uintptr_t cpu_slot;
    struct pp2_hw *hw = &inst->hw;

    /* Master thread initializes common part of HW.
     * This will probably get deprecated by KS driver for the initialization
     * part, but keep it for now
     */
    cpu_slot = hw->base[PP2_DEFAULT_REGSPACE].va;

    /* Clear BM */
    pp2_bm_flush_pools(cpu_slot, inst->parent->init.bm_pool_reserved_map);

	ppdk_cls_default_config_set(inst);
	pp2_cls_mng_init(cpu_slot);

    /* Disable RXQs */
    for (i = 0; i < PP2_NUM_PORTS; i++) {
        struct ppio_init_params	*ppio_param = &(inst->parent->init.ppios[inst->id][i]);
        if (ppio_param->is_enabled == false)
            continue;
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
    uint32_t reg_id, i;
    uintptr_t mem_base;
    struct pp2_hw *hw = &inst->hw;

    hw->tclk = PP2_TCLK_FREQ;

    err = pp2_sys_ioinit(&inst->pp2_maps_hdl, pp2_id_uio_name((u8)inst->id));
    if (err) {
        pp2_err("PPDK: No device found\n");
        return err;
    }

    /* Map the whole physical Packet Processor physical address */
    mem_base = pp2_sys_iomap(inst->pp2_maps_hdl, (uint32_t *)&hw->phy_address_base, "pp");
    if (!mem_base) {
        err = -EIO;
        pp2_sys_iodestroy(inst->pp2_maps_hdl);
        return err;
    }

    /* Assign each CPU (thread) slot its mapped address space. */

    for (reg_id = 0; reg_id < ARRAY_SIZE(hw->base); reg_id++) {
        hw->base[reg_id].va = mem_base + (reg_id * PP2_REGSPACE_SIZE);
    }
    mem_base = pp2_sys_iomap(inst->pp2_maps_hdl, (uint32_t *)&hw->gop.serdes.base.pa, "serdes");
    if (!mem_base) {
        err = -EIO;
        pp2_sys_iounmap(inst->pp2_maps_hdl, "pp");
        pp2_sys_iodestroy(inst->pp2_maps_hdl);
        return err;
    }
    hw->gop.serdes.base.va = mem_base;
    hw->gop.serdes.obj_size = 0x1000;

    mem_base = pp2_sys_iomap(inst->pp2_maps_hdl, (uint32_t *)&hw->gop.xmib.base.pa, "xmib");
    if (!mem_base) {
        err = -EIO;
        pp2_sys_iounmap(inst->pp2_maps_hdl, "serdes");
        pp2_sys_iounmap(inst->pp2_maps_hdl, "pp");
        pp2_sys_iodestroy(inst->pp2_maps_hdl);
        return err;
    }
    hw->gop.xmib.base.va = mem_base;
    hw->gop.xmib.obj_size = 0x0100;

    mem_base = pp2_sys_iomap(inst->pp2_maps_hdl, (uint32_t *)&hw->gop.smi.pa, "smi");
    if (!mem_base) {
        err = -EIO;
        pp2_sys_iounmap(inst->pp2_maps_hdl, "xmib");
        pp2_sys_iounmap(inst->pp2_maps_hdl, "serdes");
        pp2_sys_iounmap(inst->pp2_maps_hdl, "pp");
        pp2_sys_iodestroy(inst->pp2_maps_hdl);
        return err;
    }
    hw->gop.smi.va = mem_base;
    hw->gop.smi.va += 0x200;
    hw->gop.smi.pa += 0x200;

    mem_base = pp2_sys_iomap(inst->pp2_maps_hdl, (uint32_t *)&hw->gop.mspg.pa, "mspg");
    if (!mem_base) {
        err = -EIO;
        pp2_sys_iounmap(inst->pp2_maps_hdl, "smi");
        pp2_sys_iounmap(inst->pp2_maps_hdl, "xmib");
        pp2_sys_iounmap(inst->pp2_maps_hdl, "serdes");
        pp2_sys_iounmap(inst->pp2_maps_hdl, "pp");
        pp2_sys_iodestroy(inst->pp2_maps_hdl);
        return err;
    }
    hw->gop.mspg.va = mem_base;

    mem_base = pp2_sys_iomap(inst->pp2_maps_hdl, (uint32_t *)&hw->gop.rfu1.pa, "rfu1");
    if (!mem_base) {
        err = -EIO;
        pp2_sys_iounmap(inst->pp2_maps_hdl, "mspg");
        pp2_sys_iounmap(inst->pp2_maps_hdl, "smi");
        pp2_sys_iounmap(inst->pp2_maps_hdl, "xmib");
        pp2_sys_iounmap(inst->pp2_maps_hdl, "serdes");
        pp2_sys_iounmap(inst->pp2_maps_hdl, "pp");
        pp2_sys_iodestroy(inst->pp2_maps_hdl);
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

    /* Get MAC data for all available ports based on dts GOP entries */
    /* TODO: Revise this after GOP dev tree support */
    for (i = 0; i < PP2_NUM_PORTS; i++) {
        struct pp2_port *port = inst->ports[i];
        struct pp2_mac_data *mac = &port->mac_data;
        uint32_t id = port->id + (inst->id * PP2_NUM_PORTS);

        /* TBD(DevTree): replace with data read from Device tree */
        mac->gop_index = hc_gop_mac_data[id].gop_index;
        mac->flags     = hc_gop_mac_data[id].flags;
        mac->phy_addr  = hc_gop_mac_data[id].phy_addr;
        mac->phy_mode  = hc_gop_mac_data[id].phy_mode;
        mac->force_link= hc_gop_mac_data[id].force_link;
        mac->autoneg   = hc_gop_mac_data[id].autoneg;
        mac->link      = hc_gop_mac_data[id].link;
        mac->duplex    = hc_gop_mac_data[id].duplex;
        mac->speed     = hc_gop_mac_data[id].speed;
        memcpy(&mac->mac, &hc_gop_mac_data[id].mac, PP2_ETHADDR_LEN);
    }

    return err;
}

static int pp2_get_devtree_port_data(struct pp2_inst *inst)
{
	FILE *fp;
	char path[256], subpath[256];
	char buf[50];
	int i;

	for (i = 0; i < PP2_NUM_PORTS; i++) {
		struct pp2_port *port = inst->ports[i];

		if (inst->id == 0) {
			/* TODO -We assume that the path is static.
			* Need to substitute this with a function that searches for the following string:
			* <.compatible = "marvell,mv-pp22"> in /proc/device-tree directories and returns the path
			*/
			sprintf(path, "/proc/device-tree/cpn-110-master/config-space/ppv22@000000/");
		} else if (inst->id == 1) {
			sprintf(path, "/proc/device-tree/cpn-110-slave/config-space/ppv22@000000/");
		} else {
			pp2_err("wrong instance id.\n");
			return -EEXIST;
		}

		/* Get port status info */
		sprintf(subpath, "eth%d@0%d0000/status", i, i+1);
		strcat(path, subpath);

		fp = fopen(path, "r");
		if (!fp) {
			pp2_err("error opening device tree status file.\n");
			return -EEXIST;
		} else {
			fgets(buf, sizeof(buf), fp);
			if (strcmp("disabled", buf) == 0) {
				pp2_dbg("port %d:%d is disabled\n", inst->id,i);
				port->admin_status = PP2_PORT_DISABLED;
			} else if (strcmp("non-kernel", buf) == 0) {
				pp2_dbg("port %d:%d is MUSDK\n", inst->id,i);
				port->admin_status = PP2_PORT_MUSDK_ENABLED;
			} else {
				pp2_dbg("port %d:%d is kernel\n", inst->id,i);
				port->admin_status = PP2_PORT_KERNEL_ENABLED;
			}
			fclose (fp);
		}
	}
	return 0;
}

static struct pp2_inst * pp2_inst_create(struct pp2 *pp2, uint32_t pp2_id)
{
   uint32_t i;
   struct pp2_inst *inst;

   if (unlikely(!pp2)) {
       pp2_err("PPDK: Invalid ppdk handle\n");
       return NULL;
   }

   inst = kcalloc(1, sizeof(struct pp2_inst), GFP_KERNEL);
   if (unlikely(!inst)) {
      pp2_err("PPDK: %s out of memory pp2_inst alloc\n",__func__);
      return NULL;
   }

   /* Early allocate and get MAC data for available ports since GOP
    * sub-system needs to be initialized once per packet processor
    * Later, when ports get opened/started, only per-MAC
    * initializations shall be done
    */
   for (i = 0 ; i < PP2_NUM_PORTS; i++) {
      struct pp2_port *port = kcalloc(1, sizeof(struct pp2_port), GFP_KERNEL);
      if (unlikely(!port)) {
         pp2_err("PPDK: %s out of memory pp2_port alloc\n", __func__);
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
       pp2_err("PPDK: cannot populate hardware data\n");
       for (i = 0; i < PP2_NUM_PORTS; i++)
           kfree(inst->ports[i]);
       kfree(inst);
       return NULL;
   }

   if (pp2_get_devtree_port_data(inst)) {
       pp2_err("PPDK: cannot populate device tree port data\n");
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

	pp2_num_inst += pp2_sys_io_exists(pp2_id_uio_name(PP2_ID0));
	pp2_num_inst += pp2_sys_io_exists(pp2_id_uio_name(PP2_ID1));

	return(pp2_num_inst);
}

static void pp2_destroy(struct pp2_inst *inst)
{
    uint32_t i;

    pp2_sys_iounmap(inst->pp2_maps_hdl, "pp");
    pp2_sys_iounmap(inst->pp2_maps_hdl, "serdes");
    pp2_sys_iounmap(inst->pp2_maps_hdl, "xmib");
    pp2_sys_iounmap(inst->pp2_maps_hdl, "smi");
    pp2_sys_iounmap(inst->pp2_maps_hdl, "mspg");
    pp2_sys_iounmap(inst->pp2_maps_hdl, "rfu1");
    pp2_sys_iodestroy(inst->pp2_maps_hdl);

    /* No dangling handles */
    for (i = 0; i < PP2_NUM_PORTS; i++) {
        if (inst->ports[i])
            kfree(inst->ports[i]);
    }
    for (i = 0; i < PP2_NUM_REGSPACES; i++) {
        if (inst->dm_ifs[i])
            kfree(inst->dm_ifs[i]);
    }
    for (i = 0; i < PP2_NUM_BMPOOLS; i++) {
        if (inst->bm_pools[i])
            kfree(inst->bm_pools[i]);
    }

    kfree(inst);
}

int pp2_init(struct pp2_init_params *params)
{
    uint32_t pp2_id;
    uint32_t pp2_num_inst;

    pp2_ptr = kcalloc(1, sizeof(struct pp2), GFP_KERNEL);
    if (unlikely(!pp2_ptr)) {
        pp2_err("PPDK: %s out of memory pp2 alloc\n",__func__);
        return -ENOMEM;
    }
    memcpy(&pp2_ptr->init, params, sizeof(*params));
    /* Copy reserved instances to used ones */
    pp2_ptr->pp2_common.hif_slot_map = pp2_ptr->init.hif_reserved_map;
    pp2_ptr->pp2_common.rss_tbl_map = pp2_ptr->init.rss_tbl_reserved_map;
    /* TODO: Check first_inq params are valid */

    /* Initialize in an opaque manner from client,
     * depending on HW, one or two packet processors.
     */
    pp2_num_inst = pp2_get_num_inst();
    for (pp2_id = 0; pp2_id < pp2_num_inst; pp2_id++) {
        struct pp2_inst *inst;

        inst = pp2_inst_create(pp2_ptr, pp2_id);
        if (NULL == inst) {
            pp2_err("PPDK: cannot create PP%u\n", pp2_id);

            if (PP2_ID1 == pp2_id) {
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

    pp2_dbg("PPDK: PackProcs   %2u\n", pp2_num_inst);

    return 0;
}

void pp2_deinit(void)
{
	for (uint32_t pp2_id = 0; pp2_id < pp2_ptr->num_pp2_inst; pp2_id++) {
		struct pp2_inst *inst = pp2_ptr->pp2_inst[pp2_id];

		pp2_destroy(inst);
	}

    /* Destroy the PPDK handle */
	kfree(pp2_ptr);
}

/* Find  pp_id and port_id parameters from ifname.
* Description: loop through all packet processors and ports in each packet processor
* and compare the interface name to the one configured for each port. If there is a match,
* the pp_id and port_id are returned.
* This function should be called after pp2_init() and before ppio_init().
 */
int pp2_netdev_get_port_info(char *ifname, u8 *pp_id, u8 *port_id)
{
	struct pp2_port *port;
	int i, j, parent_changed = 0, found = 0;

	for (i = 0; i < PP2_MAX_NUM_PACKPROCS; i++) {
		if (!pp2_ptr->pp2_inst[i])
			continue;

		for (j = 0; j < PP2_NUM_PORTS; j++) {
			port = pp2_ptr->pp2_inst[i]->ports[j];
			if (!port->parent) {
				port->parent = pp2_ptr->pp2_inst[i];
				parent_changed = 1;
			}

			pp2_port_get_if_name(port);

			if (strcmp(port->linux_name, ifname) == 0)
				found = 1;

			if (parent_changed)
				port->parent = NULL;

			if (found) {
				*pp_id = i;
				*port_id = j;
				pp2_info("%s: ppio-%d,%d\n", ifname, *pp_id, *port_id);
				return 0;
			}
		}
	}
	return -EEXIST;
}

