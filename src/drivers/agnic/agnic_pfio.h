/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef __AGNIC_PFIO_H__
#define __AGNIC_PFIO_H__

#include "std_internal.h"

#include "drivers/mv_agnic_pfio.h"

#include "agnic_pfio_hw.h"


#define AGNIC_COMPATIBLE_STR		"marvell,armada-giu-nic"
/* right now, we assume the BAR is allocated on the 'main' MUSDK CMA memory
 * (i.e. it must not be mapped on cma-region)
 */
#define MUSDK_CMA_DEV_STR		"/dev/musdk-cma"

#define CFG_MEM_AP8xx_OFFS		(0xA0)

#define CFG_MEM_VALID			(1 << 0)
#define CFG_MEM_BIDX_MASK		(0x1F)
#define CFG_MEM_BIDX_SHIFT		(1)
#define CFG_MEM_BAR_MIN_ALIGN_BITS	(12)
#define CFG_MEM_BAR_MIN_ALIGN		(1 << CFG_MEM_BAR_MIN_ALIGN_BITS)
#define CFG_MEM_64B_MAGIC_MASK		(0xFFFFLL << 48)
#define CFG_MEM_64B_MAGIC_VAL		(0xCAFELL << 48)
/* in 64bits reg, we allow address of 44bits with 12bits alignment */
#define CFG_MEM_64B_ADDR_MASK		\
	(0x00000FFFFFFFFFFFLL & ~(CFG_MEM_BAR_MIN_ALIGN - 1))
/* in 32bits reg, we allow address of 36bits with 12bits alignment */
#define CFG_MEM_32B_ADDR_MASK		\
	(0x0000000FFFFFFFFFLL & ~(CFG_MEM_BAR_MIN_ALIGN - 1))
#define CFG_MEM_32B_ADDR_SHIFT		(4)

#define AGNIC_BAR_ALLOC_SIZE		(0x4000)

#define AGNIC_CONFIG_BAR_SIZE		(AGNIC_BAR_ALLOC_SIZE)

#define AGNIC_MAX_QUEUES		(AGNIC_PFIO_MAX_NUM_QS_PER_TC*AGNIC_PFIO_MAX_NUM_TCS)
#define AGNIC_MAX_RXQ_COUNT		(AGNIC_MAX_QUEUES)
#define AGNIC_MAX_TXQ_COUNT		(AGNIC_MAX_QUEUES)
#define AGNIC_BPOOLS_COUNT		(AGNIC_MAX_QUEUES)

#define AGNIC_RING_INDICES_ADDR_ALIGN	(1<<6)
#define AGNIC_RINGS_ADDR_ALIGN		(1<<10)

/* MGMT Desc defines */
#define AGNIC_CMD_Q_LEN			256
#define AGNIC_CMD_Q_MAX_COOKIE_LEN	(AGNIC_CMD_Q_LEN<<1)
#define AGNIC_NOTIF_Q_LEN		(AGNIC_CMD_Q_LEN)

#define MGMT_CMD_MAX_IDX		(1 << 16)
#define MGMT_CMD_IDX_FREE		(0)
#define MGMT_CMD_IDX_OCCUPY		(1)

#define AGNIC_MAX_MTU			2000

/* All ring sizes are a power of 2.
 * This is checked by the init functions.
 */
#define AGNIC_RING_PTR_INC(idx, num, ring_sz)		\
	((idx) = (((idx) + (num)) & ((ring_sz) - 1)))

#define AGNIC_RING_IS_FULL(prod, cons, ring_sz)		\
	((((prod) + 1) & ((ring_sz) - 1)) == (cons))

#define AGNIC_RING_IS_EMPTY(prod, cons)			\
	((prod) == (cons))

#define AGNIC_RING_NUM_OCCUPIED(prod, cons, ring_sz)	\
	(((prod) - (cons) + (ring_sz)) & ((ring_sz) - 1))

#define AGNIC_RING_FREE(prod, cons, ring_sz)		\
	((ring_sz) - AGNIC_RING_NUM_OCCUPIED(prod, cons, ring_sz) - 1)

/* Calc physical address of a local producer / consumer pointer. */
#define AGNIC_RING_PROD_INDX_LOCAL_PHYS(ring)		\
	((ring)->pfio->ring_indices_arr_phys + ((ring)->prod_idx * sizeof(u32)))
#define AGNIC_RING_CONS_INDX_LOCAL_PHYS(ring)		\
	((ring)->pfio->ring_indices_arr_phys + ((ring)->cons_idx * sizeof(u32)))

enum agnic_ring_type {
	agnic_rx_ring	 = 0x01,
	agnic_tx_ring	 = 0x02,
	agnic_cmd_ring	 = 0x04,
	agnic_notif_ring = 0x08,
	agnic_bpool_ring = 0x10,
	agnic_all_rings	 = 0xff
};
#define AGNIC_IS_INGRESS_RING(type) ((type) & (agnic_rx_ring | agnic_notif_ring))

/* Message Parameters
 * cmd_idx	- Command Identifier, this field is OUTPUT param which will be will be
 *		  set by the lower layer.
 * cmd_code	- Command to be executed (out of enum agnic_cmd_codes)
 * client_id	- Client ID - PF / VF Id
 * client_type	- Client type - PF / VF
 * msg		- Message data (command parameter)
 * msg_len	- Message data size
 * timeout	- (not supported) Timeout for receiving reply
 * resp_msg	- Message response
 * resp_msg_len - Message response size
 *     Array of bytes, holding the serialized parameters/response list for a specific command.
 */
/* Make sure structure is portable along different systems. */
struct agnic_msg_params {
	u16	 cmd_idx;
	u8	 cmd_code;
	u8	 client_id;
	u8	 client_type;
	void	*msg;
	u16	 msg_len;
	u32	 timeout;
	void	*resp_msg;
	u16	 resp_msg_len;
};

/*
** agnic_mgmt_cookie: Pointed by the command descriptor (and copied to the respective
**  notification desc), provides the condition & operation result to be read by
**  command issuer.
** agnic_tx_cookie: Holds Tx buffers specific information, to be associated with each
**  transmitted buffer.
** agnic_bp_cookie: Holds BP buffers specific info, held along with Rx B-Pool
**  elements. And received back through the Rx descriptor cookie field.
*/
struct agnic_mgmt_cookie {
	enum {
		mgmt_buff_cmd_sent = 0,
		mgmt_buff_notif_rcv = 1
	} condition;	/* The condition on which the command issuer will wait */
	int result;	/* cmd result, as received by the notification message */
	void *buf;	/* Command response buffer */
	int buf_len;	/* Command response buffer length */
};

struct agnic_bp_cookie {
	void *buff_virt_addr;
	struct agnic_ring *bp_ring;
};

struct agnic_cookie {
	union {
		struct agnic_bp_cookie   bp;
		struct agnic_mgmt_cookie mgmt;
	};
};

struct agnic_pfio;

/*
 * agnic_ring: Rx, Tx, mgmt, B-pool ring definition.
 *  - next: Next ring in q_vectors.
 *  - adapter: back pointer to adapter.
 *  - netdev: back pointer to net-device.
 *  - q_vector: The q_vector this ring belong to, NULL in case the ring does
 *  not belong to a specific q-vector (e.g. b-pool rings).
 *  - dev: Device for DMA mappings.
 *  - type: Type of ring (see agnic_ring_type).
 *  - desc_size: Size of a single descriptor.
 *  - prod_idx: Offset of the Q control producer pointer in the q_indeces_arr array
 *  - cons_idx: Offset of the Q control consumer pointer in the q_indeces_arr array
 *  - tc: the TC this ring is assigned to.
 *  - desc: Pointer to ring memory (Array).
 *  - dma: Physical address of desc.
 *  - count: # of descriptors in ring.
 *  Queue management "pointers": queue control indexes, we use pointers
 *  because the location of these control values will be communicated &
 *  shared with the HIU (In order for it to read & update them).
 *  On the egress direction, produce_p will be located on the
 *  end-point memory, and consumer_p will be located on the host
 *  dram (The other way around for the ingress side).
 *  We always want the RO variable to be on the local memory in order to prevent
 *  read accesses over PCIe (high latency). On the other side, write transactions
 *  over PCIe are less expensive as they are posted transactions.
 *  - Tx:
 *    o producer (WO): The descriptor that will hold the next transmitted packet.
 *    o consumer (RO): Next descritor to be fetched by HW (HIU).
 *    o tx_next_to_reclaim: Next descritor to free, after being sent by HW (i.e. first
 *      descriptor to be handled by the tx-done event.) - This is a local ring mgmt index
 *      that does not get read / written by the HIU.
 *    o tx_prod_shadow: A local shadow we operate on, before writing final value
 *      to producer pointer.
 *  - Rx:
 *    o producer (RO): The next descriptor to be filled with Rx data.
 *    o consumer (WO): The descriptor from which to receive the next packet.
 *    o rx_cons_shadow: A local shadow we operate on, before writing final value
 *      to consumer pointer.
 *  - cookie_list: Array of cookies info structs. This is actually a wrapper
 *    around 3 types of cookie info structures: Tx, Rx & Mgmt.
 *  - cookie_count: # of entries in cookie list.
 *  - q_idx: For data rings (Rx / Tx) - Holds the queue index.
 *    This is the absolute index of the queue in the system.
 *    Example: 8 Tx rings might be split into 3 different q-vectors, the
 *    q_idx of each ring will hold it's sequence number in the system
 *    (regardless of the q-vector it belongs to).
 *  - total_packets: # of rx/tx descriptors which were copied to the ring.
 *  - prev_counter: since we don't have hw counters, this field is used to allow
 *                  "reset" of the counters, when presented to the used.
 */
struct agnic_ring {
	u32 *producer_p;
	u32 *consumer_p;
	u16 tx_next_to_reclaim;

	union {
		u16 tx_prod_shadow;
		u16 rx_cons_shadow;
	};

	void *desc;
	dma_addr_t dma;
	u32 count;

	/* TODO: get rid of this list */
	struct agnic_cookie *cookie_list;
	u32 cookie_count;

	struct agnic_ring *next;
	struct agnic_pfio *pfio;
	enum agnic_ring_type type;
	int desc_size;

	u8 prod_idx;
	u8 cons_idx;
	u8 tc;
	u8 q_idx;

	u32 bp_frag_sz;
};

struct agnic_intc {
	u8			 num_qs;
	u16			 buff_size;
	u16			 pkt_offset;

	struct agnic_ring	*rings[AGNIC_PFIO_MAX_NUM_QS_PER_TC];
	struct agnic_ring	*bp_rings[AGNIC_PFIO_MAX_NUM_QS_PER_TC];
};

struct agnic_outtc {
	u8			 num_qs;

	struct agnic_ring	*rings[AGNIC_PFIO_MAX_NUM_QS_PER_TC];
};

struct agnic_cb_msg_params {
	u32 status;
	u32 timeout; /* timeout in msec */
	u64 cookie;
};

struct agnic_custom {
	u8 id;
	void *arg;
	void (*f_recv_custom_msg_cb)(void *arg, u8 code, u64 cookie, void *msg, u16 len);

	struct agnic_cb_msg_params cb_msg_params[MGMT_CMD_MAX_IDX];
};

struct agnic_pfio {
	int			 id;

	struct sys_iomem	*reg_iomem;
	void			*sh_mem_base_reg;
	struct sys_iomem	*bar_iomem;
	dma_addr_t		 nic_cfg_phys_base;
	struct agnic_config_mem *nic_cfg;

	/* Management command & notification Rings. */
	struct agnic_ring	 cmd_ring;
	struct agnic_ring	 notif_ring;
	spinlock_t		 mgmt_lock;

	/* custom information */
	struct agnic_custom	 custom_info;

	u8			 mac[ETH_ALEN];

	/* RSS mode */
	enum agnic_pfio_hash_type hash_type;
	u16			 mtu;
	u16			 mru;

#define AGNIC_FLAG_MSI_ENABLED			BIT(1)
#define AGNIC_FLAG_MSIX_ENABLED			BIT(2)
#define AGNIC_FLAG_MGMT_POLL			BIT(3)
#define AGNIC_FLAG_RX_POLL			BIT(4)
#define AGNIC_FLAG_IRQPOLL			(AGNIC_FLAG_RX_POLL | AGNIC_FLAG_MGMT_POLL)
	u32			 flags;
	int			 link;

	/* A page to hold the consumer / producer indexes per queue.
	 * the consumer_p or producer_p pointers will actually point
	 * an entry in this array (depending on the direction of the queue).
	 */
	u32			*ring_indices_arr;
	dma_addr_t		 ring_indices_arr_phys;
	u32			 ring_indices_arr_len;

	/* Tx - one ring per active queue */
	u16			 num_tx_queues;
	u16			 tx_ring_size;
	struct agnic_ring	*tx_ring[AGNIC_MAX_TXQ_COUNT];

	/* Rx */
	u16			 num_rx_queues;
	u16			 rx_ring_size;
	struct agnic_ring	*rx_ring[AGNIC_MAX_RXQ_COUNT];

	/* BP */
	u16			 num_rx_pools;
	u16			 buff_size;
	u16			 pkt_offset;
	struct agnic_ring	 bp_ring[AGNIC_BPOOLS_COUNT];

	u8			 num_in_tcs;
	u8			 num_out_tcs;
	u8			 num_qs_per_tc;

	struct agnic_intc	 in_tcs[AGNIC_PFIO_MAX_NUM_TCS];
	struct agnic_outtc	 out_tcs[AGNIC_PFIO_MAX_NUM_TCS];
};

int agnic_init_pfio(struct agnic_pfio *pfio);
void agnic_deinit_pfio(struct agnic_pfio *pfio);

int agnic_mgmt_notif_process(struct agnic_pfio *pfio, u16 cmd_code, void *msg, u16 len);

#endif /* __AGNIC_PFIO_H__ */
