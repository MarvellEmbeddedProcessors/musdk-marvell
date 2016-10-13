
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

#include "lib/uio_helper.h"
#include "drivers/mv_pp2.h"
#include "drivers/mv_pp2_bpool.h"
#include "drivers/mv_pp2_hif.h"
#include "drivers/mv_pp2_ppio.h"

#include "mv_pp2x_hw_type.h"
#include "pp2_gop.h"
#include "pp2_util.h"
#include "pp2_plat.h"
#include "pp2_mem.h"

#define PP_UIO_DEV_NAME "uio_mv_pp"
#define PP_UIO_MEM_NAME "pp"

/*TODO: Decide if to duplicate the below defines, under different name,
or split the definitions from the mvpp2io.h, and other header files, and use them here. */
#if 0
#define PP2_PPIO_MAX_NUM_TCS	8
#define PP2_PPIO_MAX_NUM_OUTQS	8
#define PP2_PPIO_TC_MAX_POOLS	2
#define PP2_PPIO_MAX_NUM_HASH	4
#endif
struct pp2;
extern struct pp2 * pp2_ptr;
/**
 * Descriptor Manager object (DM-IF) internal layout
 *
 * Up to 9 per packet processor. At run-time, each
 * DM uses a dedicated CPU slot. Each egress thread
 * should use a dedicated DM-IF in order to avoid
 * locks on data-path.
 */

struct pp2_match_param {
	int pp2_id;
	int id;
};

struct pp2_dm_if {
    /* Aggregator Queue physical ID */
    uint32_t id;
    /* Number of descriptors in the egress ring */
    uint32_t desc_total;
    /* Number of free descriptors */
    uint32_t free_count;
    /* Index of the next TXD descriptor to process */
    uint32_t desc_next_idx;
    /* Physical addr of the first TXD of the TXD array */
    uintptr_t desc_phys_arr;
    /* Virtual addr of the first TXD of the TXD array */
    struct pp2_desc *desc_virt_arr;
    /* Parent packet processor */
    struct pp2_inst *parent;
    /* CPU slot address assigned to this DM object */
    uintptr_t cpu_slot;
    /* CMA handle */
    uintptr_t cma_hdl;
};

/* Number of descriptors to prefetch from DRAM for this TXQ */
#define PP2_TXQ_PREFETCH_4       (4)
#define PP2_TXQ_PREFETCH_16     (16)
#define PP2_TXQ_PREFETCH_32     (32)


struct pp2_txq_dm_if {
	uint32_t desc_rsrvd;
};
/**
 * Transmit Queue (TXQ) internal layout
 *
 * Up to 8 per port. At run-time, it can be shared by
 * multiple DM-IF objects, each object reserving a
 * number of descriptors for egress.
 */
struct pp2_tx_queue {
    /* TXQ physical ID */
    uint32_t id;
    /* TXQ logical ID */
    uint32_t log_id;
    /* Number of descriptors in this TXQ */
    uint32_t desc_total;
    /* Physical addr of the first TXD of the TXD array */
    uintptr_t desc_phys_arr;
    /* CMA handle */
    uintptr_t cma_hdl;
    /* Virtual addr of the first TXD of the TXD array */
    struct pp2_desc *desc_virt_arr;
    struct pp2_txq_dm_if txq_dm_if[PP2_NUM_REGSPACES];
};

/**
 * Reception Queue (RXQ) internal layout
 *
 * Up to 32 per port. At run-time, a thread can poll
 * an RXQ to receive data via RX descriptors (RXDs).
 */
struct pp2_rx_queue {
    /* RXQ physical ID */
    uint32_t id;
    /* RXQ logical ID */
    uint32_t log_id;
    /* Number of descriptors in this RXQ */
    uint32_t desc_total;
    /* Number of descriptors that are known to be occupied */
    uint32_t desc_received;
    /* Index of the last RXD */
    uint32_t desc_last_idx;
    /* Index of the next RXD descriptor to process */
    uint32_t desc_next_idx;
    /* Lock per RX queue*/
    uint32_t rxq_lock;
    /* Physical addr of the first RXD of the RXD array */
	uintptr_t desc_phys_arr;
    /* Virtual addr of the first RXD of the RXD array */
    struct pp2_desc *desc_virt_arr;
    /* CMA handle */
    uintptr_t cma_hdl;
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
    uint32_t slot_id;
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
    uint32_t bm_pool_id;
    /* BM pool number of buffers */
    uint32_t bm_pool_buf_num;
    /* BM pool buffer size */
    uint32_t bm_pool_buf_sz;
    /* Parent packet processor
    struct pp2_inst *parent; */
    /* Parent packet processor ID */
    uint32_t pp2_id;
    /* BM pool logical address (BPPE) */
    uintptr_t bm_pool_virt_base;
    /* BM pool physical address (BPPE) */
    uintptr_t bm_pool_phys_base;
    /* BM CMA BPPE handler */
    uintptr_t bppe_mem;
};

/* Port minimum MTU in bytes */
#define PP2_PORT_MIN_MTU         (68)
/* Port default MTU in bytes */
#define PP2_PORT_DEFAULT_MTU     (1500)
/* Port TX FIFO constants */
#define PP2_TX_FIFO_SIZE_2KB     (0x02)
#define PP2_TX_FIFO_SIZE_3KB     (0x03)
#define PP2_TX_FIFO_SIZE_4KB     (0x04)
#define PP2_TX_FIFO_SIZE_5KB     (0x05)
#define PP2_TX_FIFO_SIZE_6KB     (0x06)
#define PP2_TX_FIFO_SIZE_7KB     (0x07)
#define PP2_TX_FIFO_SIZE_8KB     (0x08)
#define PP2_TX_FIFO_SIZE_9KB     (0x09)
#define PP2_TX_FIFO_SIZE_10KB    (0x0A)

/* Minimum threshold 256 bytes */
#define PP2_TX_FIFO_THRS_MIN     (0x100)
/* Port TX FIFO threshold constants */
#define PP2_TX_FIFO_THRS_2KB     (PP2_TX_FIFO_SIZE_2KB * 1024 - \
                                        PP2_TX_FIFO_THRS_MIN)
#define PP2_TX_FIFO_THRS_3KB     (PP2_TX_FIFO_SIZE_3KB * 1024 - \
                                        PP2_TX_FIFO_THRS_MIN)
#define PP2_TX_FIFO_THRS_4KB     (PP2_TX_FIFO_SIZE_4KB * 1024 - \
                                        PP2_TX_FIFO_THRS_MIN)
#define PP2_TX_FIFO_THRS_5KB     (PP2_TX_FIFO_SIZE_5KB * 1024 - \
                                        PP2_TX_FIFO_THRS_MIN)
#define PP2_TX_FIFO_THRS_6KB     (PP2_TX_FIFO_SIZE_6KB * 1024 - \
                                        PP2_TX_FIFO_THRS_MIN)
#define PP2_TX_FIFO_THRS_7KB     (PP2_TX_FIFO_SIZE_7KB * 1024 - \
                                        PP2_TX_FIFO_THRS_MIN)
#define PP2_TX_FIFO_THRS_8KB     (PP2_TX_FIFO_SIZE_8KB * 1024 - \
                                        PP2_TX_FIFO_THRS_MIN)
#define PP2_TX_FIFO_THRS_9KB     (PP2_TX_FIFO_SIZE_9KB * 1024 - \
                                        PP2_TX_FIFO_THRS_MIN)
#define PP2_TX_FIFO_THRS_10KB    (PP2_TX_FIFO_SIZE_10KB * 1024 - \
                                        PP2_TX_FIFO_THRS_MIN)

/* MAC address length */
#define PP2_ETHADDR_LEN          (6)


struct pp2_ppio_tc_config {
	uint16_t pkt_offset;    /* Must be multiple of 32 bytes.*/
	uint8_t use_hash;
	uint8_t num_in_qs;
	uint8_t first_rxq;	/* First physical rx_queue for this TC */
	uint8_t pools[PP2_PPIO_TC_MAX_POOLS];
/* TODO: future:
	int							 qos;
*/
};

struct pp2_tc {
    uint32_t first_log_rxq;
    uint32_t rx_ring_size;
    struct pp2_ppio_tc_config tc_config;
};

struct pp2_txq_config {
    uint16_t size;
    uint16_t weight;
};

/* PP Port internal structure */
struct pp2_port {
    /* Port ID */
    uint32_t id;
    /* Number of RXQs used by this port */
    uint32_t num_rx_queues;
    /* Number of TXQs used by this port */
    uint32_t num_tx_queues;
    struct pp2_txq_config txq_config[PP2_PPIO_MAX_NUM_OUTQS];
    /* Number of TCs used by this port */
    uint32_t num_tcs;
    /* MRU */
    uint32_t port_mru;
    /* MTU */
    uint32_t port_mtu;
    /* First RXQ physical ID (i.e. 0, 32, 64) */
    uint32_t first_rxq;
    /* MAC loopback configuration */
    uint32_t use_mac_lb;
    /* Type of traffic enabled: ingress, egress or both */
    uint32_t t_mode;
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
	uint32_t tclk;
	/* PRS shadow table */
	struct mv_pp2x_prs_shadow *prs_shadow;
	/* PRS auxiliary table for double vlan entries control */
	bool *prs_double_vlans;
	/* CLS shadow info for update in running time */
	struct mv_pp2x_cls_shadow *cls_shadow;
	/* C2 shadow info */
	struct mv_pp2x_c2_shadow *c2_shadow;
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
	uint8_t cos_classifier;	/* CoS based on VLAN or DSCP */
	uint8_t num_cos_queues;	/* number of queue to do CoS */
	uint8_t default_cos;	/* Default CoS value for non-IP or non-VLAN */
	uint8_t reserved;
	uint32_t pri_map;	/* 32 bits, each nibble maps a cos_value(0~7)
				 * to a queue.
				 */
};

struct mv_pp2x_rss {
	uint8_t rss_mode;	/* UDP packet */
	uint8_t dflt_cpu;	/* non-IP packet */
	uint8_t rss_en;     /* RSS enable/disable */
};

struct mv_pp2x_param_config {
	struct mv_pp2x_cos cos_cfg;
	struct mv_pp2x_rss rss_cfg;
	uint8_t first_bm_pool;
	uint8_t first_sw_thread;	/* The index of the first PPv2.2
					 * sub-address space for this NET_INSTANCE.
					 */
	uint8_t first_log_rxq;	/* The first cos rx queue used in the port */
	uint8_t cell_index;	/* The cell_index of the PPv22
				 * (could be 0,1, set according to dtsi)
				 */
	enum mv_pp2x_queue_distribution_mode queue_mode;
	uint32_t rx_cpu_map;	/* The CPU that port bind, each port has
				 * a nibble indexed by port_id,
				 * nibble value is CPU id
				 */
};

struct mv_pp2x_platform_data {
	uint8_t pp2x_max_port_rxqs;
	uint8_t num_port_irq;
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
	uint32_t id;
	/* HW data */
	struct pp2_hw hw;
	struct mv_pp2x_platform_data pp2xdata;
	struct mv_pp2x_param_config pp2_cfg;
	/* Bitmap of the participating cpu's */
	uint16_t cpu_map;
	/* Port objects associated */
	uint32_t num_ports;
	struct pp2_port *ports[PP2_NUM_PORTS];
	/* BM Pools associated */
	uint32_t num_bm_pools;
	struct pp2_bm_pool *bm_pools[PP2_NUM_BMPOOLS];
	/* DM objects associated */
	uint32_t num_dm_ifs;
	struct pp2_dm_if *dm_ifs[PP2_NUM_REGSPACES];
	/* CPU index according to weight. */
	uint32_t rx_table[MVPP22_RSS_TBL_LINE_NUM];
	/* UIO context */
	struct pp2_uio uio;
	/* Handle to parent instance */
	struct pp2 *parent;
    /* UIO handle for mapping PP and GOP register spaces */
    pp2_maps_handle_t pp2_maps_hdl;
};

struct pp2_common_cfg {
	u16 hif_slot_map;
	u16 rss_tbl_map;
};
/* Main control structure for the PPDK */
struct pp2 {
	struct pp2_init_params init;
        struct pp2_common_cfg pp2_common;
	uint32_t num_pp2_inst;
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
    uint32_t phys_lo;

    phys_lo =  pp2_reg_read(cpu_slot, MVPP2_BM_PHY_ALLOC_REG(pool_id));
    if (unlikely(!phys_lo)) {
        pp2_err("BM: BufGet failed! (Pool ID=%d)\n", pool_id);
        return 0;
    }
#else
    /* Trigger BM allocation by reading from BM_PHY_ALLOC and after
     * BM pops data into the lo and hi virt addr registers, construct
     * the full 40-bit virtual address */
    pp2_reg_read(cpu_slot, MVPP2_BM_PHY_ALLOC_REG(pool_id));
#endif

    vaddr = pp2_reg_read(cpu_slot, MVPP22_BM_PHY_VIRT_HIGH_ALLOC_REG);
    vaddr &= MVPP22_BM_VIRT_HIGH_ALLOC_MASK;
    vaddr <<= (32 - MVPP22_BM_VIRT_HIGH_ALLOC_OFFSET);
    vaddr |= pp2_reg_read(cpu_slot, MVPP2_BM_VIRT_ALLOC_REG);

#if PP2_BM_BUF_DEBUG
    pp2_info("BM: BufGet %p\n", (void*)vaddr);
#endif

    return vaddr;
}


static inline int pp2_is_init(void)
{
    return (pp2_ptr != NULL);
}


#endif /* _PP2_H_ */
