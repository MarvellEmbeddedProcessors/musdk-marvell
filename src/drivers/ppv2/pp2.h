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
 * @file pp2.h
 *
 * Packet Processor core structures
 * Implements key init, config and control routines for the PPDK instance
 *
 */

#ifndef _PP2_H_
#define _PP2_H_

#include "pp2_types.h"

#include "drivers/mv_pp2.h"
#include "drivers/mv_pp2_bpool.h"
#include "drivers/mv_pp2_hif.h"
#include "drivers/mv_pp2_ppio.h"
#include "std_internal.h"
#include "lib/net.h"

#include "pp2_hw_type.h"
#include "pp2_gop.h"
#include "pp2_plat.h"
#include "pp2_mem.h"

#define PP_UIO_DEV_NAME "uio_mv_pp"
#define PP_UIO_MEM_NAME "pp"

#define PP2_NETDEV_PATH		"/sys/class/net/"
#define PP2_NETDEV_MASTER_PATH	"/proc/device-tree/cpn-110-master/config-space/ppv22@000000/"
#define PP2_NETDEV_SLAVE_PATH	"/proc/device-tree/cpn-110-slave/config-space/ppv22@000000/"

#define PP2_MAX_BUF_STR_LEN	256

#define GET_PPIO_PORT(ppio) ((struct pp2_port *)(ppio)->internal_param)
#define GET_PPIO_PORT_PTR(ppio) ((struct pp2_port **)&(ppio).internal_param)

/* TODO: Decide if to duplicate the below defines, under different name,
 * or split the definitions from the mvpp2io.h, and other header files, and use them here.
 */
#if 0
#define PP2_PPIO_MAX_NUM_TCS	8
#define PP2_PPIO_MAX_NUM_OUTQS	8
#define PP2_PPIO_TC_MAX_POOLS	2
#define PP2_PPIO_MAX_NUM_HASH	4
#endif
struct pp2;
extern struct pp2 *pp2_ptr;
/**
 * Descriptor Manager object (DM-IF) internal layout
 *
 * Up to 9 per packet processor. At run-time, each
 * DM uses a dedicated CPU slot. Each egress thread
 * should use a dedicated DM-IF in order to avoid
 * locks on data-path.
 */

struct netdev_if_params {
	char if_name[16];
	u32 admin_status;
	u8 ppio_id;
	u8 pp_id;
};

struct pp2_match_param {
	int pp2_id;
	int id;
};

struct pp2_dm_if {
	/* Aggregator Queue physical ID */
	u32 id;
	/* Number of descriptors in the egress ring */
	u32 desc_total;
	/* Number of free descriptors */
	u32 free_count;
	/* Index of the next TXD descriptor to process */
	u32 desc_next_idx;
	/* Physical addr of the first TXD of the TXD array */
	uintptr_t desc_phys_arr;
	/* Virtual addr of the first TXD of the TXD array */
	struct pp2_desc *desc_virt_arr;
	/* Parent packet processor */
	struct pp2_inst *parent;
	/* CPU slot address assigned to this DM object */
	uintptr_t cpu_slot;
};

/* Number of descriptors to prefetch from DRAM for this TXQ */
#define PP2_TXQ_PREFETCH_4       (4)
#define PP2_TXQ_PREFETCH_16     (16)
#define PP2_TXQ_PREFETCH_32     (32)

struct pp2_txq_dm_if {
	u32 desc_rsrvd;
};

/* Automatic statistics update threshold (in received packetes) */
#define PP2_STAT_UPDATE_THRESHOLD	(0xEFFFFFFF)

#define PP2_UPDATE_CNT32(counter, value) {						\
	counter = ((u64)(counter + value) > UINT_MAX) ? UINT_MAX : counter + value;	\
	}

#define PP2_READ_UPDATE_CNT32(counter, cpu_slot, reg)	{	\
	u32 value = pp2_relaxed_reg_read(cpu_slot, reg);	\
	PP2_UPDATE_CNT32(counter, value);			\
}

#define PP2_UPDATE_CNT64(counter, value) \
	{ counter += value; }

#define PP2_READ_UPDATE_CNT64(counter, cpu_slot, reg) \
	{ counter += pp2_relaxed_reg_read(cpu_slot, reg); }

/**
 * Transmit Queue (TXQ) internal layout
 *
 * Up to 8 per port. At run-time, it can be shared by
 * multiple DM-IF objects, each object reserving a
 * number of descriptors for egress.
 */
struct pp2_tx_queue {
	/* TXQ physical ID */
	u32 id;
	/* TXQ logical ID */
	u32 log_id;
	/* Number of descriptors in this TXQ */
	u32 desc_total;
	/* Physical addr of the first TXD of the TXD array */
	uintptr_t desc_phys_arr;
	/* Virtual addr of the first TXD of the TXD array */
	struct pp2_desc *desc_virt_arr;
	struct pp2_txq_dm_if txq_dm_if[PP2_NUM_REGSPACES];
	/* TXQ statistics */
	struct pp2_ppio_outq_statistics stats;
	/* TXQ statistics update threshold */
	u32 threshold_tx_pkts;
};

/**
 * Reception Queue (RXQ) internal layout
 *
 * Up to 32 per port. At run-time, a thread can poll
 * an RXQ to receive data via RX descriptors (RXDs).
 */
struct pp2_rx_queue {
	/* RXQ physical ID */
	u32 id;
	/* RXQ logical ID */
	u32 log_id;
	/* Number of descriptors in this RXQ */
	u32 desc_total;
	/* Number of descriptors that are known to be occupied */
	u32 desc_received;
	/* Index of the last RXD */
	u32 desc_last_idx;
	/* Index of the next RXD descriptor to process */
	u32 desc_next_idx;
	/* Lock per RX queue*/
	u32 rxq_lock;
	/* Physical addr of the first RXD of the RXD array */
	uintptr_t desc_phys_arr;
	/* Virtual addr of the first RXD of the RXD array */
	struct pp2_desc *desc_virt_arr;
	/* RXQ statistics */
	struct pp2_ppio_inq_statistics stats;
	/* RXQ statistics update threshold */
	u32 threshold_rx_pkts;
};

/**
 * Buffer Manager object (BM-IF) internal layout
 *
 * Up to 9 per packet processor. At run-time, each
 * BM uses a dedicated CPU slot. Each thread that uses
 * BM pools should use a dedicated BM-IF in order to
 * avoid locks on data-path. Multiple BM-IFs can
 * share a BM pool
 */
struct pp2_bm_if {
	/* Parent packet processor */
	struct pp2_inst *parent;
	/* CPU slot address assigned to this BM object */
	uintptr_t cpu_slot;
	/* Slot number relativ to Packet Processor */
	u32 slot_id;
};

#define PP2_BM_BUF_DEBUG             (FALSE)

/* Per-BP mandatory alignment */
#define BM_BUF_ALIGN                (32)
#define BM_BPPESIZE_SHIFT            (3)
#define MVPP2_BM_POOL_PTR_ALIGN     (128)
#define MVPP2_BM_POOL_SIZE_MAX      (16 * 1024 - (MVPP2_BM_POOL_PTR_ALIGN / 4))
/**
 * Buffer Manager Pool (BM Pool) internal layout
 *
 * Up to 16 per packet processor. At run-time, a thread
 * can create and use one or more BM pools, each pool
 * sharable by multiple threads.
 */
struct pp2_bm_pool {
	/* BM pool physcal ID */
	u32 bm_pool_id;
	/* BM pool number of buffers */
	u32 bm_pool_buf_num;
	/* BM pool buffer size */
	u32 bm_pool_buf_sz;
	/* Parent packet processor */
	/* struct pp2_inst *parent; */
	/* Parent packet processor ID */
	u32 pp2_id;
	/* BM pool logical address (BPPE) */
	uintptr_t bm_pool_virt_base;
	/* BM pool physical address (BPPE) */
	uintptr_t bm_pool_phys_base;
};

/* Port minimum MTU in bytes */
#define PP2_PORT_MIN_MTU         (68) /* Required to support IPV4, per RFC791 */
#define PP2_PORT_MIN_MRU         (MVPP2_MTU_TO_MRU(PP2_PORT_MIN_MTU))

/* Port default MTU in bytes */
#define PP2_PORT_DEFAULT_MTU     (1500)
/* Port TX FIFO constants */

/* Minimum threshold 256 bytes */
#define PP2_TX_FIFO_THRS_MIN_SUBSTRACTION	(256)

#define PP2_PORT_TX_FIFO_KB_TO_THRESH(fifo)	((fifo) * 1024 - PP2_TX_FIFO_THRS_MIN_SUBSTRACTION)

/* MAC address length */
#define PP2_ETHADDR_LEN          (6)

#define PP2_PORT_FLAGS_L4_CHKSUM (0x1)

struct pp2_ppio_tc_config {
	u16 pkt_offset;    /* Must be multiple of 32 bytes.*/
	u8 use_hash;
	u8 num_in_qs;
	u8 first_rxq;	/* First physical rx_queue for this TC */
	struct pp2_bm_pool *pools[PP2_PPIO_TC_MAX_POOLS];
/* TODO: future:
 *	int							 qos;
 */
};

struct pp2_tc {
	u32 first_log_rxq;
	u32 rx_ring_size;
	struct pp2_ppio_tc_config tc_config;
};

struct pp2_txq_config {
	u16 size;
	u16 weight;
};

enum port_status {
	PP2_PORT_DISABLED,
	PP2_PORT_KERNEL_ENABLED,
	PP2_PORT_MUSDK_ENABLED,
};

/* PP Port internal structure */
struct pp2_port {
	/* Port ID */
	u32 id;
	/* Port status */
	u32 admin_status;
	/* Port Flags */
	u32 flags;
	/* Number of RXQs used by this port */
	u32 num_rx_queues;
	/* Number of TXQs used by this port */
	u32 num_tx_queues;
	struct pp2_txq_config txq_config[PP2_PPIO_MAX_NUM_OUTQS];
	/* Number of TCs used by this port */
	u32 num_tcs;
	/* MRU */
	u16 port_mru;
	/* MTU */
	u16 port_mtu;
	/* First RXQ physical ID (i.e. 0, 32, 64) */
	u32 first_rxq;
	/* MAC loopback configuration */
	u32 use_mac_lb;
	/* Type of traffic enabled: ingress, egress or both */
	u32 t_mode;
	/* Parent packet processor */
	struct pp2_inst *parent;
	/* Port default CPU slot for initialization phase */
	uintptr_t cpu_slot;
	/* Array of egress queues */
	struct pp2_tx_queue **txqs;
	/* Array of ingress queues */
	struct pp2_rx_queue **rxqs;
	/* Array of Traffic Classes */
	struct pp2_tc tc[PP2_PPIO_MAX_NUM_TCS];
	/* Hash types for his port */
	enum pp2_ppio_hash_type hash_type[PP2_PPIO_MAX_NUM_HASH];
	/* MAC data for this port */
	struct pp2_mac_data mac_data;
	/* Linux interface name for this port */
	char linux_name[16];
	/* tx_fifo_size in KB */
	u32 tx_fifo_size;
	/* Flag to indicate the automatic statistics update */
	int maintain_stats;
};

/**
 * Base physical address map:
 * Packet Processors
 *   SB0: 0xF2000000 : PP[0xF2000000 - 0xF2090000] (9 ASs 0x10000)
 *   SB1: 0xF4000000 : PP[0xF4000000 - 0xF4090000] (9 ASs 0x10000)
 *
 * GMACs
 *   SB0: 0xF2130000
 *   SB1: 0xF4130000
 */
struct pp2_hw {
	/* Register space slot for software thread */
	struct base_addr base[PP2_NUM_REGSPACES];
	/* Register space for GOPs */
	struct gop_hw gop;
	u32 tclk;
	/* PRS shadow table */
	struct mv_pp2x_prs_shadow *prs_shadow;
	/* PRS auxiliary table for double vlan entries control */
	bool *prs_double_vlans;
	/* CLS shadow info for update in running time */
	struct mv_pp2x_cls_shadow *cls_shadow;
	/* C2 shadow info */
	struct mv_pp2x_c2_shadow *c2_shadow;
	/*physical base address */
	u32	phy_address_base;
};

enum mv_pp2x_queue_distribution_mode {
	/* All queues are shared.
	 * PPv2.1: this is the only supported mode.
	 * PPv2.2: Requires (N+1) interrupts. All rx_queues are
	 * configured on the additional interrupt.
	 */
	MVPP2_QDIST_SINGLE_MODE,
	MVPP2_QDIST_MULTI_MODE	/* PPv2.2 only requires N interrupts */
};

struct mv_pp2x_cos {
	u8 cos_classifier;	/* CoS based on VLAN or DSCP */
	u8 num_cos_queues;	/* number of queue to do CoS */
	u8 default_cos;	/* Default CoS value for non-IP or non-VLAN */
	u8 reserved;
	u32 pri_map;	/* 32 bits, each nibble maps a cos_value(0~7)
				 * to a queue.
				 */
};

struct mv_pp2x_rss {
	u8 rss_mode;	/* UDP packet */
	u8 dflt_cpu;	/* non-IP packet */
	u8 rss_en;     /* RSS enable/disable */
};

struct mv_pp2x_param_config {
	struct mv_pp2x_cos cos_cfg;
	struct mv_pp2x_rss rss_cfg;
	u8 first_bm_pool;
	u8 first_sw_thread;	/* The index of the first PPv2.2
					 * sub-address space for this NET_INSTANCE.
					 */
	u8 first_log_rxq;	/* The first cos rx queue used in the port */
	u8 cell_index;	/* The cell_index of the PPv22
				 * (could be 0,1, set according to dtsi)
				 */
	enum mv_pp2x_queue_distribution_mode queue_mode;
	u32 rx_cpu_map;	/* The CPU that port bind, each port has
				 * a nibble indexed by port_id,
				 * nibble value is CPU id
				 */
};

struct mv_pp2x_platform_data {
	u8 pp2x_max_port_rxqs;
	u8 num_port_irq;
	bool multi_addr_space;
	bool interrupt_tx_done;
	bool multi_hw_instance;
};

struct pp2_uio {
	struct uio_info_t *uio_info;
	struct uio_mem_t *mem;
};

/* Main control structure private per Packet Processor */
struct pp2_inst {
	/* This packet processor's ID */
	u32 id;
	/* HW data */
	struct pp2_hw hw;
	struct mv_pp2x_platform_data pp2xdata;
	struct mv_pp2x_param_config pp2_cfg;
	/* Bitmap of the participating cpu's */
	u16 cpu_map;
	/* Port objects associated */
	u32 num_ports;
	struct pp2_port *ports[PP2_NUM_PORTS];
	/* BM Pools associated */
	u32 num_bm_pools;
	struct pp2_bm_pool *bm_pools[PP2_BPOOL_NUM_POOLS];
	/* DM objects associated */
	u32 num_dm_ifs;
	struct pp2_dm_if *dm_ifs[PP2_NUM_REGSPACES];
	/* CPU index according to weight. */
	u32 rx_table[MVPP22_RSS_TBL_LINE_NUM];
	/* UIO context */
	struct pp2_uio uio;
	/* Handle to parent instance */
	struct pp2 *parent;
	/* UIO handle for mapping PP and GOP register spaces */
	pp2_maps_handle_t pp2_maps_hdl;
	struct pp2_cls_db_t *cls_db;
};

struct pp2_common_cfg {
	u16 hif_slot_map;
	u16 rss_tbl_map;
};

/* Main control structure for the PPDK */
struct pp2 {
	struct pp2_init_params init;
	struct pp2_common_cfg pp2_common;
	u32 num_pp2_inst;
	struct pp2_inst *pp2_inst[PP2_MAX_NUM_PACKPROCS];
};

/* BM Internal stuff */
void pp2_bm_hw_pool_destroy(uintptr_t cpu_slot, uint32_t pool_id);
uint32_t pp2_bm_pool_flush(uintptr_t cpu_slot, uint32_t pool_id);

/* TODO : Do not delete, used by flush_pools */
static inline uintptr_t pp2_bm_hw_buf_get(uintptr_t cpu_slot, uint32_t pool_id)
{
	uintptr_t vaddr;
#if PP2_BM_BUF_DEBUG
	u32 phys_lo;

	phys_lo =  pp2_reg_read(cpu_slot, MVPP2_BM_PHY_ALLOC_REG(pool_id));
	if (unlikely(!phys_lo)) {
		pr_err("BM: BufGet failed! (Pool ID=%d)\n", pool_id);
		return 0;
	}
#else
	/* Trigger BM allocation by reading from BM_PHY_ALLOC and after
	 * BM pops data into the lo and hi virt addr registers, construct
	 * the full 40-bit virtual address
	 */
	pp2_reg_read(cpu_slot, MVPP2_BM_PHY_ALLOC_REG(pool_id));
#endif

	vaddr = pp2_reg_read(cpu_slot, MVPP22_BM_PHY_VIRT_HIGH_ALLOC_REG);
	vaddr &= MVPP22_BM_VIRT_HIGH_ALLOC_MASK;
	vaddr <<= (32 - MVPP22_BM_VIRT_HIGH_ALLOC_OFFSET);
	vaddr |= pp2_reg_read(cpu_slot, MVPP2_BM_VIRT_ALLOC_REG);

#if PP2_BM_BUF_DEBUG
	pr_info("BM: BufGet %p\n", (void *)vaddr);
#endif

	return vaddr;
}

static inline int pp2_is_init(void)
{
	return (pp2_ptr) ? 1 : 0;
}

static inline uintptr_t pp2_default_cpu_slot(struct pp2_inst *inst)
{
	return inst->hw.base[PP2_DEFAULT_REGSPACE].va;
}

int pp2_netdev_if_info_get(struct netdev_if_params *netdev_params);
int pp2_netdev_ifname_get(u32 pp_id, u32 ppio_id, char *ifname);
int pp2_netdev_if_admin_status_get(u32 pp_id, u32 ppio_id, u32 *admin_status);

#endif /* _PP2_H_ */
