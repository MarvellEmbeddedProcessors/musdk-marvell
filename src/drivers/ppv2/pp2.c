/**
 * @file pp2.c
 *
 * PPDK container structures and packet processor initialization
 */

#include <stdint.h>
#include <stdlib.h>


#include <mv_pp_uio.h>

#include "drivers/mv_pp2.h"
#include "pp2.h"
#include <pp2_dm.h>
#include <pp2_port.h>
#include <pp2_bm.h>

#include "pp2_gop_dbg.h"
#include "pp2_qos.h"
#include "int/xflags.h"


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

    /*AXI Bridge Configuration */

    /* BM */
    pp2_reg_write(cpu_slot, MVPP22_AXI_BM_WR_ATTR_REG,
            MVPP22_AXI_ATTR_HW_COH_WRITE);
    pp2_reg_write(cpu_slot, MVPP22_AXI_BM_RD_ATTR_REG,
            MVPP22_AXI_ATTR_HW_COH_READ);

    /* Descriptors */
    pp2_reg_write(cpu_slot, MVPP22_AXI_AGGRQ_DESCR_RD_ATTR_REG,
            MVPP22_AXI_ATTR_HW_COH_READ);
    pp2_reg_write(cpu_slot, MVPP22_AXI_TXQ_DESCR_WR_ATTR_REG,
            MVPP22_AXI_ATTR_HW_COH_WRITE);
    pp2_reg_write(cpu_slot, MVPP22_AXI_TXQ_DESCR_RD_ATTR_REG,
            MVPP22_AXI_ATTR_HW_COH_READ);
    pp2_reg_write(cpu_slot, MVPP22_AXI_RXQ_DESCR_WR_ATTR_REG,
            MVPP22_AXI_ATTR_HW_COH_WRITE);

    /* Buffer Data */
    pp2_reg_write(cpu_slot, MVPP22_AXI_TX_DATA_RD_ATTR_REG,
            MVPP22_AXI_ATTR_HW_COH_READ);
    pp2_reg_write(cpu_slot, MVPP22_AXI_RX_DATA_WR_ATTR_REG,
            MVPP22_AXI_ATTR_HW_COH_WRITE);

    val = MVPP22_AXI_CODE_CACHE_NON_CACHE << MVPP22_AXI_CODE_CACHE_OFFS;
    val |= MVPP22_AXI_CODE_DOMAIN_SYSTEM << MVPP22_AXI_CODE_DOMAIN_OFFS;
    pp2_reg_write(cpu_slot, MVPP22_AXI_RD_NORMAL_CODE_REG, val);
    pp2_reg_write(cpu_slot, MVPP22_AXI_WR_NORMAL_CODE_REG, val);

    val = MVPP22_AXI_CODE_CACHE_RD_CACHE << MVPP22_AXI_CODE_CACHE_OFFS;
    val |= MVPP22_AXI_CODE_DOMAIN_OUTER_DOM << MVPP22_AXI_CODE_DOMAIN_OFFS;
    pp2_reg_write(cpu_slot, MVPP22_AXI_RD_SNOOP_CODE_REG, val);

    val = MVPP22_AXI_CODE_CACHE_WR_CACHE << MVPP22_AXI_CODE_CACHE_OFFS;
    val |= MVPP22_AXI_CODE_DOMAIN_OUTER_DOM << MVPP22_AXI_CODE_DOMAIN_OFFS;
    pp2_reg_write(cpu_slot, MVPP22_AXI_WR_SNOOP_CODE_REG, val);

    /* Set cache snoop when transmitting packets */
    pp2_reg_write(cpu_slot, MVPP2_TX_SNOOP_REG, 0x01);

    /*     Classifier    */
    /* default classifier setup */
    ppdk_cls_default_config_set(inst);

    /* TODO(RX): Parser flow ID, Parser/Classifier/C2 config init */
    mv_pp2x_prs_flow_id_attr_init();
    mv_pp2x_prs_default_init(hw);
    mv_pp2x_cls_init(hw);
    mv_pp2x_c2_init(hw);

    /* TBD(RX): Init PP22 rxfhindir(RSS) table evenly */
    pp2_init_rxfhindir(inst);

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
    for (i = 0; i < PP2_NUM_PORTS; i++)
    {
        uint32_t net_comp_config;
        struct pp2_port *port = inst->ports[i];
        struct gop_hw *gop   = &inst->hw.gop;
        struct ppio_init_params	*ppio_param = &(inst->parent->init.ppios[inst->id][i]);

        if (ppio_param->is_enabled == false)
            continue;

        net_comp_config = pp2_gop_netc_cfg_create(&port->mac_data);
        pp2_gop_netc_init(gop, net_comp_config, PP2_NETC_FIRST_PHASE);
        pp2_gop_netc_init(gop, net_comp_config, PP2_NETC_SECOND_PHASE);
    }
}

static int pp2_get_hw_data(struct pp2_inst *inst)
{
    int err = 0;
    uint32_t reg_id, i;
    uintptr_t mem_base;
    struct pp2_hw *hw = &inst->hw;

    hw->tclk = PP2_TCLK_FREQ;

    err = pp2_sys_ioinit(&inst->pp2_maps_hdl,
            (PP2_ID0 == inst->id) ? UIO_PP_0 : UIO_PP_1);
    if (err) {
        pp2_err("PPDK: No device found\n");
        return err;
    }

    /* Map the whole physical Packet Processor physical address */
    mem_base = pp2_sys_iomap(inst->pp2_maps_hdl, NULL, "pp");
    if (!mem_base) {
        err = -EIO;
        pp2_sys_iodestroy(inst->pp2_maps_hdl);
        return err;
    }

    /* Assign each CPU (thread) slot its mapped address space. */

    for (reg_id = 0; reg_id < ELEM_OF(hw->base); reg_id++) {
        if (1<<reg_id & inst->parent->init.hif_reserved_map)
            continue;
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

static struct pp2_inst * pp2_inst_create(struct pp2 *pp2, uint32_t pp2_id)
{
   uint32_t i;
   struct pp2_inst *inst;

   if (unlikely(!pp2)) {
       pp2_err("PPDK: Invalid ppdk handle\n");
       return NULL;
   }

   inst = calloc(1, sizeof(struct pp2_inst));
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
      struct pp2_port *port = calloc(1, sizeof(struct pp2_port));
      if (unlikely(!port)) {
         pp2_err("PPDK: %s out of memory pp2_port alloc\n", __func__);
         break;
      }
      /* Static ID assignment */
      port->id = i;
      inst->ports[i] = port;
   }

   inst->id = pp2_id;

   /* Get static device tree data */
   if (pp2_get_hw_data(inst)) {
       pp2_err("PPDK: cannot populate hardware data\n");
       for (i = 0; i < PP2_NUM_PORTS; i++)
           free(inst->ports[i]);
       free(inst);
       return NULL;
   }

   return inst;
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
            free(inst->ports[i]);
    }
    for (i = 0; i < PP2_NUM_REGSPACES; i++) {
        if (inst->dm_ifs[i])
            free(inst->dm_ifs[i]);
    }
    for (i = 0; i < PP2_NUM_BMPOOLS; i++) {
        if (inst->bm_pools[i])
            free(inst->bm_pools[i]);
    }

    free(inst);
}

int pp2_init(struct pp2_init_params *params)
{
    uint32_t pp2_id;
    uint32_t pp2_num_inst;

    pp2_ptr = calloc(1, sizeof(struct pp2));
    if (unlikely(!pp2_ptr)) {
        pp2_err("PPDK: %s out of memory pp2 alloc\n",__func__);
        return -ENOMEM;
    }
    memcpy(&pp2_ptr->init, params, sizeof(*params));
    /* Copy reserved instances to used ones */
    pp2_ptr->pp2_common.hif_slot_map = pp2_ptr->init.hif_reserved_map;
    pp2_ptr->pp2_common.rss_tbl_map = pp2_ptr->init.rss_tbl_reserved_map;


    /* TODO: Check first_inq params are valid */

    /* Initialize the contiguous memory allocator */
    if (pp2_cma_init()) {
        pp2_err("PPDK: %s CMA init failure\n",__func__);
        free(pp2_ptr);
        return -ENOMEM;
    }

    /* Initialize in an opaque manner from client,
     * depending on HW, one or two packet processors.
     */
    pp2_num_inst = PP2_SOC_NUM_PACKPROCS;
    for (pp2_id = 0; pp2_id < pp2_num_inst; pp2_id++) {
        struct pp2_inst *inst;

        inst = pp2_inst_create(pp2_ptr, pp2_id);
        if (NULL == inst) {
            pp2_err("PPDK: cannot create PP%u\n", pp2_id);

            if (PP2_ID1 == pp2_id) {
                /* Also destroy the previous instance */
                pp2_destroy(pp2_ptr->pp2_inst[PP2_ID0]);
            }
            pp2_cma_deinit();
            free(pp2_ptr);

            return -ENOMEM;
        }
        /* Store the PPDK handle as parent and store
         * this instance as child handle for the PPDK
         */
        inst->parent = pp2_ptr;
        pp2_ptr->pp2_inst[pp2_id] = inst;

        /* Initialize this packet processor */
        pp2_inst_init(inst);
        pp2_ptr->num_pp2_inst++;
    }
    pp2_dbg("PPDK: PackProcs   %2u\n", PP2_SOC_NUM_PACKPROCS);
    pp2_dbg("PPDK:   Ports     %2u\n", PP2_NUM_PORTS);
    pp2_dbg("PPDK:   Regspaces %2u\n", PP2_NUM_REGSPACES);
    pp2_dbg("PPDK:   BM Pools  %2u\n", PP2_NUM_BMPOOLS);

    return 0;
}

void pp2_deinit(void)
{
	for (uint32_t pp2_id = 0; pp2_id < pp2_ptr->num_pp2_inst; pp2_id++) {
		struct pp2_inst *inst = pp2_ptr->pp2_inst[pp2_id];

		pp2_destroy(inst);
	}

	/* De-initialize the contiguous memory allocator */
	pp2_cma_deinit();

    /* Destroy the PPDK handle */
	free(pp2_ptr);
}

