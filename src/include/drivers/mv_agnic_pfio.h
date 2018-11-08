/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef __MV_AGNIC_PFIO_H__
#define __MV_AGNIC_PFIO_H__

#include "mv_std.h"
#include "mv_net.h"

/** @addtogroup grp_agnic_io Armada GIU NIC: I/O
 *
 *  Armada GIU NIC I/O API documentation
 *
 *  @{
 */

#define AGNIC_PFIO_MAX_NUM_TCS			8 /**< Max. number of TCs per pfio. */
#define AGNIC_PFIO_MAX_NUM_QS_PER_TC		8 /**< Max. number of Qs per TC. */
#define AGNIC_PFIO_DESC_NUM_WORDS		8

#define AGNIC_PFIO_MD_MODE_16B_LEN		16

#define AGNIC_PFIO_CUSTOM_CODE_NOTIFY_DOWN	(0xFF)

typedef u64	agnic_dma_addr_t;
typedef u64	agnic_cookie_t;

enum agnic_pfio_hash_type {
	AGNIC_PFIO_HASH_T_NONE = 0,	/* Invalid hash type */
	AGNIC_PFIO_HASH_T_2_TUPLE,	/* IP-src, IP-dst */
	AGNIC_PFIO_HASH_T_5_TUPLE,	/* IP-src, IP-dst, IP-Prot, L4-src, L4-dst */
	AGNIC_PFIO_HASH_T_OUT_OF_RANGE
};

struct agnic_pfio;

struct agnic_buff_inf {
	agnic_dma_addr_t	addr;
	agnic_cookie_t		cookie;
};

struct agnic_pfio_desc {
	u32			cmds[AGNIC_PFIO_DESC_NUM_WORDS];
};

struct agnic_pfio_statistics {
	/* IN port statistics */
	u64	in_packets;	/**< IN Packets Counter */
	/* OUT port statistics */
	u64	out_packets;	/**< OUT Packets Counter */
};

struct agnic_pfio_inband_mng_msg_params {
	u8	 msg_code;
	void	*msg;
	u16	 msg_len;
	u32	 timeout; /* timeout in msec */
	u64	 cookie; /*< user cookie. Use '0' if no response is needed.
			  * Value '-1' should not be used as it represents 'notification' message.
			  */
	void	*resp_msg;
	u16	 resp_msg_len;
};

struct agnic_pfio_pci_bar_inf {
	void		*va;
	dma_addr_t	 pa;
	size_t		 size;
};

/**
 * agnic init parameters
 *
 */
struct agnic_pfio_init_params {
	int				 pci_mode;
	struct agnic_pfio_pci_bar_inf	 pci_bar_inf;

	u8				 num_in_tcs;
	u8				 num_out_tcs;
	u8				 num_qs_per_tc;
	enum agnic_pfio_hash_type	 hash_type;
	u16				 in_qs_size;
	u16				 out_qs_size;
	u16				 buff_size;
	u16				 pkt_offset;
};

/**
 * Initialize a AGNIC pfio
 *
 * @param[in]	params	A pointer to structure that contains all relevant parameters.
 * @param[out]	pfio	A pointer to opaque pfio handle of type 'struct agnic_pfio *'.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int agnic_pfio_init(struct agnic_pfio_init_params *params, struct agnic_pfio **pfio);

/**
 * Destroy a AGNIC pfio
 *
 * @param[in]	pfio	A pfio handle.
 *
 */
void agnic_pfio_deinit(struct agnic_pfio *pfio);

/**
 * TODO
 *
 * @param[in]	pfio	A pfio handle.
 * @param[in]	id	TODO
 * @param[in]	arg	TODO
 * @param[in]	recv_custom_msg_cb	TODO
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int agnic_pfio_register_custom_msg_cb(struct agnic_pfio *pfio,
				      u8 id,
				      void *arg,
				      void (*recv_custom_msg_cb)(void *arg, u8 code, u64 cookie, void *msg, u16 len));

/**
 * Send inband custom message through AGNIC pfio
 *
 * @param[in]	pfio	A pfio handle.
 * @param[in]	msg	TODO
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int agnic_pfio_send_custom_msg(struct agnic_pfio *pfio,
			       struct agnic_pfio_inband_mng_msg_params *msg);

/**
 * TODO
 *
 * @param[in]	pfio	A pfio handle.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int agnic_pfio_poll_mgmt(struct agnic_pfio *pfio);

/******************* OutQ descriptor *****************/
/* cmd 0 */
#define AGNIC_OUT_D_FIRST		(0x2)
#define AGNIC_OUT_D_LAST		(0x1)
#define AGNIC_OUT_D_FIRST_LAST		(AGNIC_OUT_D_FIRST | AGNIC_OUT_D_LAST)
#define AGNIC_OUT_D_L_MASK		(0x10000000)
#define AGNIC_OUT_D_F_MASK		(0x20000000)
#define AGNIC_OUT_D_FL_MASK		(AGNIC_OUT_D_F_MASK | AGNIC_OUT_D_L_MASK)
#define AGNIC_OUT_D_L4_TYPE_GET(word)	((word >> 24) & 0x3)
#define AGNIC_OUT_D_L4_TCP		(0 << 24)
#define AGNIC_OUT_D_L4_UDP		(1 << 24)
#define AGNIC_OUT_D_L4_OTHER		(2 << 24)
#define AGNIC_OUT_D_L3_INFO_GET(word)	((word >> 26) & 0x3)
#define AGNIC_OUT_D_L3_INFO_IPV4	(0 << 26)
#define AGNIC_OUT_D_L3_INFO_IPV6	(1 << 26)
#define AGNIC_OUT_D_L3_INFO_OTHER	(2 << 26)
#define AGNIC_OUT_D_MD_MOD_MASK		(0x00400000)
#define AGNIC_OUT_D_GEN_L4_CSUM_NOT	(2 << 13)
#define AGNIC_OUT_D_GEN_IPV4_CSUM_DIS	(1 << 15)
#define AGNIC_OUT_D_L3_OFFSET_SET(word, val)	((word) |= ((val & 0x7F) << 0))
#define AGNIC_OUT_D_IP_HDR_LEN_SET(word, val)	((word) |= ((val & 0x1F) << 8))
/* cmd 1 */
#define AGNIC_OUT_D_PKT_OFF_MASK	(0x000000FF)
#define AGNIC_OUT_D_BYTE_COUNT_MASK	(0xFFFF0000)

#define AGNIC_OUT_D_SET_FIRST_LAST(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~AGNIC_OUT_D_FL_MASK) | (data << 28 & AGNIC_OUT_D_FL_MASK))

/******************** InQ descriptor *****************/
/* cmd 0 */
#define AGNIC_IN_D_L3_INFO_GET(word)	((word >> 28) & 0x7)
#define AGNIC_IN_D_L3_IP4		(1 << 28)
#define AGNIC_IN_D_L3_IP6		(1 << 30)
#define AGNIC_IN_D_MD_MOD_MASK		(0x00400000)
#define AGNIC_IN_D_L4_TYPE_GET(word)	((word >> 25) & 0x7)
#define AGNIC_IN_D_L4_TCP		(1 << 25)
#define AGNIC_IN_D_L4_UDP		(1 << 26)
#define AGNIC_IN_D_IP_HDR_LEN_GET(word)	((word >> 8) & 0x1F)
#define AGNIC_IN_D_L3_OFFSET_GET(word)	((word >> 0) & 0x7F)
/* cmd 1 */
#define AGNIC_IN_D_PKT_OFF_MASK		(0x000000FF)
#define AGNIC_IN_D_BYTE_COUNT_MASK	(0xFFFF0000)

/****************************************************************************
 *	Desc inspections APIs
 ****************************************************************************/

/******** OutQ  ********/

/**
 * Reset an outq packet descriptor to default value.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 *
 */
static inline void agnic_pfio_outq_desc_reset(struct agnic_pfio_desc *desc)
{
	/* Note: no need to 'zeroed' cmds[4] as they will be overridden
	 *	 when preparing the descriptor
	 */
	desc->cmds[0] = desc->cmds[1] = desc->cmds[2] = desc->cmds[3] =
	desc->cmds[5] = 0;
	AGNIC_OUT_D_SET_FIRST_LAST(desc, AGNIC_OUT_D_FIRST_LAST);
}

/**
 * Set the physical address in an outq packet descriptor.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	addr	Physical DMA address containing the packet to be sent.
 *
 */
static inline void agnic_pfio_outq_desc_set_phys_addr(struct agnic_pfio_desc *desc, dma_addr_t addr)
{
	/* cmd[4] and cmd[5] holds the buffer physical address (Low and High parts) */
	desc->cmds[4] = (u32)addr;
	desc->cmds[5] = (u64)addr >> 32;
}

/**
 * Set the user specified cookie in an outq packet descriptor.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	cookie	User specified cookie.
 *
 */
static inline void agnic_pfio_outq_desc_set_cookie(struct agnic_pfio_desc *desc, u64 cookie)
{
	desc->cmds[6] = (u32)cookie;
	desc->cmds[7] = (u64)cookie >> 32;
}

static inline void agnic_pfio_outq_desc_set_pkt_offset(struct agnic_pfio_desc *desc, u8  offset)
{
	desc->cmds[1] = (u32)offset;
}

/**
 * Set the packet length in an outq packet descriptor.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	len	The packet length, not including CRC.
 *
 */
static inline void agnic_pfio_outq_desc_set_pkt_len(struct agnic_pfio_desc *desc, u16 len)
{
	desc->cmds[1] = (desc->cmds[1] & ~AGNIC_OUT_D_BYTE_COUNT_MASK) | (len << 16 & AGNIC_OUT_D_BYTE_COUNT_MASK);
}

/**
 * Set the metadata mode in an outq packet descriptor.
 *
 * @param[out]	desc		A pointer to a packet descriptor structure.
 * @param[in]	md_mode		'1' for setting metadata mode
 *
 */
static inline void agnic_pfio_outq_desc_set_md_mode(struct agnic_pfio_desc *desc, int md_mode)
{
	desc->cmds[0] = (desc->cmds[0] & ~AGNIC_OUT_D_MD_MOD_MASK) | (md_mode << 22 & AGNIC_OUT_D_MD_MOD_MASK);
}

/******** InQ  ********/

enum agnic_inq_desc_l3_type {
	AGNIC_IN_D_L3_TYPE_NA = 0,
	AGNIC_IN_D_L3_TYPE_IPV4_NO_OPTS,/* IPv4 with IHL=5, TTL>0 */
	AGNIC_IN_D_L3_TYPE_IPV4_OPTS,	/* IPv4 with IHL>5, TTL>0 */
	AGNIC_IN_D_L3_TYPE_IPV4_OTHER,	/* Other IPV4 packets */
	AGNIC_IN_D_L3_TYPE_IPV6_NO_EXT,	/* IPV6 without extensions */
	AGNIC_IN_D_L3_TYPE_IPV6_EXT,	/* IPV6 with extensions */
	AGNIC_IN_D_L3_TYPE_ARP,		/* ARP */
	AGNIC_IN_D_L3_TYPE_USER_DEFINED	/* User defined */
};

enum agnic_inq_desc_l4_type {
	AGNIC_IN_D_L4_TYPE_NA = 0,
	AGNIC_IN_D_L4_TYPE_TCP,
	AGNIC_IN_D_L4_TYPE_UDP,
	AGNIC_IN_D_L4_TYPE_OTHER
};

/**
 * Get the physical DMA address from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	physical dma address
 */
static inline u64 agnic_pfio_inq_desc_get_phys_addr(struct agnic_pfio_desc *desc)
{
	/* cmd[4] and cmd[5] holds the buffer physical address (Low and High parts) */
	return ((u64)desc->cmds[5] << 32) | (u64)desc->cmds[4];
}

/**
 * Get the user defined cookie from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	cookie
 */
static inline u64 agnic_pfio_inq_desc_get_cookie(struct agnic_pfio_desc *desc)
{
	/* cmd[6] and cmd[7] holds the cookie (Low and High parts) */
	return ((u64)desc->cmds[7] << 32) | (u64)desc->cmds[6];
}

/**
 * Get the packet length from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	packet length
 */
static inline u16 agnic_pfio_inq_desc_get_pkt_len(struct agnic_pfio_desc *desc)
{
	u16 len = (desc->cmds[1] & AGNIC_IN_D_BYTE_COUNT_MASK) >> 16;
	return len;
}

/**
 * Get the metadata mode from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	metadata mode
 */
static inline u8 agnic_pfio_inq_desc_get_md(struct agnic_pfio_desc *desc)
{
	return (desc->cmds[0] & AGNIC_IN_D_MD_MOD_MASK) >> 22;
}

/**
 * Send a batch of frames (single dscriptor) on an OutQ of PP-IO.
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[in]		tc	out-TC id on which to send the frames.
 * @param[in]		qid	Q id within the out-TC on which to send the frames.
 * @param[in]		descs	A pointer to an array of descriptors representing the
 *				frames to be sent.
 * @param[in,out]	num	input: number of frames to be sent; output: number of frames sent.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_send(struct agnic_pfio		*pfio,
		  u8				 tc,
		  u8				 qid,
		  struct agnic_pfio_desc	*descs,
		  u16				*num);

/**
 * Get number of packets sent on a queue, since last call of this API.
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[in]		hif	A hif handle.
 * @param[in]		tc	out-TC id to get the number of packets.
 * @param[in]		qid	Q id within the out-TC on which to get the number of packets.
 * @param[out]		num	Number of frames that were sent.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_get_num_outq_done(struct agnic_pfio	*pfio,
				 u8			 tc,
				 u8			 qid,
				 u16			*num);

/**
 * Receive packets on a pfio.
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[in]		tc	in-TC id on which to receive the frames.
 * @param[in]		qid	Q id within the in-TC on which to receive the frames.
 * @param[in]		descs	A pointer to an array of descriptors represents the
 *				received frames.
 * @param[in,out]	num	input: Max number of frames to receive;
 *				output: number of frames received.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_recv(struct agnic_pfio		*pfio,
		    u8				 tc,
		    u8				 qid,
		    struct agnic_pfio_desc	*descs,
		    u16				*num);

/**
 * Fill RX descriptors ring with buffer pointers
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[in]		tc	in-TC id to fill.
 * @param[in]		qid	Q id within the in-TC to fill.
 * @param[in]		bufs	A pointer to an array of buffers to put to descriptors.
 * @param[in,out]	num	input: number of buffers in array;
 *				output: number of buffers were put to descriptors.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_inq_put_buffs(struct agnic_pfio		*pfio,
			     u8				 tc,
			     u8				 qid,
			     struct agnic_buff_inf	*bufs,
			     u16			*num);

/**
 * Get all free buffers found in InQ.
 * Tis routine shal be used only to cleanup InQ.
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[in]		tc	in-TC id to get the buffer from.
 * @param[in]		qid	Q id within the in-TC to get the buffer from.
 * @param[out]		bufs	A pointer to an array of buffers to free.
 * @param[in,out]	num_of_buffs	input: number of buffers in array
 *					output: number of buffers to free
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_inq_get_all_buffs(struct agnic_pfio	*pfio,
				 u8			 tc,
				 u8			 qid,
				 struct agnic_buff_inf	*bufs,
				 u16			*num_of_buffs);


/* Run-time Control API */

/**
 * Enable a pfio
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_enable(struct agnic_pfio *pfio);

/**
 * Disable a pfio
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_disable(struct agnic_pfio *pfio);

/**
 * Set pfio Ethernet MAC address
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[in]		addr	Ethernet MAC address to configure .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_set_mac_addr(struct agnic_pfio *pfio, const eth_addr_t addr);

/**
 * Get pfio Ethernet MAC address
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[out]		addr	Configured pfio Ethernet MAC address .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_get_mac_addr(struct agnic_pfio *pfio, eth_addr_t addr);

/**
 * Set pfio to promiscuous mode
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[in]		enr	1 - enable, 0 - disable.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_set_promisc(struct agnic_pfio *pfio, int en);

/**
 * Get pfio promiscuous mode
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[out]		enr	1 - enabled, 0 - disabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_get_promisc(struct agnic_pfio *pfio, int *en);

/**
 * Set pfio to listen to all multicast mode
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[in]		enr	1 - enable, 0 - disable.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_set_mc_promisc(struct agnic_pfio *pfio, int en);

/**
 * Get pfio all multicast mode
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[out]		enr	1 - enabled, 0 - disabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_get_mc_promisc(struct agnic_pfio *pfio, int *en);

/**
 * Add pfio ethernet multicast MAC address
 *
 * Can support up to:
 *	- 256 MC address filtering of IP multicast packets and
 *	  256 other MAC multicast addresses
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[in]		addr	Ethernet MAC address to add .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_add_mac_addr(struct agnic_pfio *pfio, const eth_addr_t addr);

/**
 * Remove pfio Ethernet multicast MAC address
 *
 * Allows to remove the mac address added by agnic_pfio_add_mac_addr()
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[in]		addr	Ethernet MAC address to remove .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_remove_mac_addr(struct agnic_pfio *pfio, const eth_addr_t addr);

/**
 * Flush pfio all ethernet MAC addresses
 *
 * NOTE: Does not flush the mac_address set by agnic_pfio_set_mac_addr().
.*
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[in]		uc	1 - flush unicast list.
 * @param[in]		mc	1 - flush multicast list .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int agnic_pfio_flush_mac_addrs(struct agnic_pfio *pfio, int uc, int mc);

/**
 * Get pfio statistics
 *
 * @param[in]		pfio	A pointer to a PP-IO object.
 * @param[out]		stats	Port statistics.
 *
 */
int agnic_pfio_get_statistics(struct agnic_pfio *pfio, struct agnic_pfio_statistics *stats);

int agnic_pfio_set_mtu(struct agnic_pfio *pfio, u16 mtu);
int agnic_pfio_get_mtu(struct agnic_pfio *pfio, u16 *mtu);
int agnic_pfio_set_mru(struct agnic_pfio *pfio, u16 len);
int agnic_pfio_get_mru(struct agnic_pfio *pfio, u16 *len);

int agnic_pfio_set_link_state(struct agnic_pfio *pfio, int en);
int agnic_pfio_get_link_state(struct agnic_pfio *pfio, int *en);

int agnic_pfio_set_loopback(struct agnic_pfio *pfio, int en);
int agnic_pfio_get_loopback(struct agnic_pfio *pfio, int *en);

/** @} */ /* end of grp_agnic_io */

#endif /* __MV_AGNIC_PFIO_H__ */
