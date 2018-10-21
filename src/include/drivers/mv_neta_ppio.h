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

#ifndef __MV_NETA_PPIO_H__
#define __MV_NETA_PPIO_H__

#include "mv_std.h"
#include "mv_net.h"


/** @addtogroup grp_neta_io Packet Processor: I/O
 *
 *  Packet Processor I/O API documentation
 *
 *  @{
 */


#define NETA_PPIO_MAX_NUM_TCS	8 /**< Max. number of TCs per ppio. */
#define NETA_PPIO_MAX_NUM_OUTQS	8 /**< Max. number of outqs per ppio. */
#define NETA_PPIO_RX_BUF_ALIGN	64 /**< RX buffer must be aligned to cache line size */
#define NETA_ETH_FCS_LEN	4 /**< NETA Ethernet FCS length */

typedef u32	neta_dma_addr_t;
typedef u32	neta_cookie_t;

struct neta_buff_inf {
	neta_dma_addr_t addr;
	neta_cookie_t   cookie;
};

struct neta_ppio {
	u32	port_id;		/* port Id */
	void	*internal_param;	/* parameters for internal use */
};

/**
 * ppio tc parameters
 *
 */
struct neta_ppio_tc_params {
	u16		pkt_offset;    /* Must be multiple of 32 bytes.*/
	u32		size;          /* Q size - number of descriptors */
};

enum neta_ppio_outqs_sched_mode {
	NETA_PPIO_SCHED_M_NONE = 0,
};

/**
 * ppio inq parameters
 *
 */
struct neta_ppio_inqs_params {
	u16				num_tcs; /**< Number of Traffic-Classes */
	/** Parameters for each TC */
	struct neta_ppio_tc_params	tcs_params[NETA_PPIO_MAX_NUM_TCS];

	u16			mtu;
};

/**
 * ppio outq parameters
 *
 */
struct neta_ppio_outq_params {
	u32	size;	/**< q_size in number of descriptors */
	u8	weight; /**< The weight is relative among the PP-IO out-Qs */
};

/**
 * ppio outq related parameters
 *
 */
struct neta_ppio_outqs_params {
	u16				 num_outqs; /**< Number of outqs */
	struct neta_ppio_outq_params	 outqs_params[NETA_PPIO_MAX_NUM_OUTQS]; /**< Parameters for each outq */
};

/**
 * ppio parameters
 *
 */
struct neta_ppio_params {
	/** Used for DTS acc to find appropriate "physical" PP-IO obj;
	 * E.g. "eth-0" means port[0]
	 */
	const char			*match;
	struct neta_ppio_inqs_params	inqs_params; /**<  ppio inq parameters structure */
	struct neta_ppio_outqs_params	outqs_params; /**<  ppio outq parameters structure */
};

enum neta_outq_l3_type {
	NETA_OUTQ_L3_TYPE_IPV4 = 0,
	NETA_OUTQ_L3_TYPE_IPV6,
	NETA_OUTQ_L3_TYPE_OTHER
};

enum neta_outq_l4_type {
	NETA_OUTQ_L4_TYPE_TCP = 0,
	NETA_OUTQ_L4_TYPE_UDP,
	NETA_OUTQ_L4_TYPE_OTHER
};

enum neta_inq_l3_type {
	NETA_INQ_L3_TYPE_NA       = 0,	/* Unknown L3 protocol */
	NETA_INQ_L3_TYPE_IPV4_BAD = 1,	/* L3 type is IPv4. IP header is not OK */
	NETA_INQ_L3_TYPE_IPV6     = 2,	/* L3 type is IPv6 */
	NETA_INQ_L3_TYPE_IPV4_OK  = 3,	/* L3 type is IPv4. IP header is OK */
};

enum neta_inq_l4_type {
	NETA_INQ_L4_TYPE_NA = 0,
	NETA_INQ_L4_TYPE_TCP = 1,
	NETA_INQ_L4_TYPE_UDP = 2,
	NETA_INQ_L4_TYPE_OTHER = 3
};

enum neta_inq_desc_status {
	NETA_DESC_ERR_OK = 0,
	NETA_DESC_ERR_MAC_CRC,		/* L2 MAC error (for example CRC Error) */
	NETA_DESC_ERR_MAC_OVERRUN,	/* L2 Overrun Error*/
	NETA_DESC_ERR_MAC_RESERVED,	/* L2 Reserved */
	NETA_DESC_ERR_MAC_RESOURCE,	/* L2 Resource Error (No buffers for multi-buffer frame) */
	NETA_DESC_ERR_IPV4_HDR,		/* L3 IPv4 Header error */
	NETA_DESC_ERR_L4_CHECKSUM	/* L4 checksum error */
};

/**
 * ppio statistics
 *
 */
struct neta_ppio_statistics {
	/* Rx port statistics */
	u64	rx_bytes;		/**< RX Bytes Counter */
	u64	rx_packets;		/**< RX Packets Counter */
	u64	rx_bytes_err;		/**< Bad RX Bytes Counter */
	u64	rx_packets_err;		/**< Bad RX Packets Counter */
	u64	rx_broadcast_packets;	/**< RX Broadcast Packets Counter */
	u64	rx_multicast_packets;	/**< RX Multicast Packets Counter */
	u64	rx_undersize;		/**< RX Undersize Packets Counter */
	u64	rx_fragments;		/**< RX Fragments Packets Counter */
	u64	rx_oversize;		/**< RX Oversize Packets Counter */
	u64	rx_errors;		/**< RX MAC Errors Counter */
	u64	rx_crc_error;		/**< RX CRC Errors Counter */
	u64	rx_discard;		/**< RX Discard Counter */
	u64	rx_overrun;		/**< RX Overrun Packets Counter */
	/* Tx port statistics */
	u64	tx_bytes;		/**< TX Bytes Counter */
	u64	tx_packets;		/**< TX Packets Counter */
	u64	tx_broadcast_packets;	/**< TX Broadcast Packets Counter */
	u64	tx_multicast_packets;	/**< TX Multicast Packets Counter */
	u64	tx_errors;		/**< TX MAC Errors Counter */
};

/**
 * Initialize a ppio
 *
 * @param[in]	params	A pointer to structure that contains all relevant parameters.
 * @param[out]	ppio	A pointer to opaque ppio handle of type 'struct neta_ppio *'.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int neta_ppio_init(struct neta_ppio_params *params, struct neta_ppio **ppio);

/**
 * Destroy a ppio
 *
 * @param[in]	ppio	A ppio handle.
 *
 */
void neta_ppio_deinit(struct neta_ppio *ppio);


/* Run-time API */

#define NETA_PPIO_DESC_NUM_WORDS	8
#define NETA_PPIO_DESC_NUM_FRAGS	16 /* TODO: check if there’s HW limitation */


struct neta_ppio_desc {
	u32			cmds[NETA_PPIO_DESC_NUM_WORDS];
};

struct neta_ppio_sg_pkts {
	u16			 num;	/**< Number of scatter-gather packets */
	u8			*frags;	/**< Array with size of 'num' representing
					*   the number of fragments per packet
					*/
};

/******************** TxQ-desc *****************/
/* word 0 */
#define NETA_TXD_L3_OFF_MASK		(0x0000007f)
#define NETA_TXD_IP_HLEN_MASK		(0x00001F00)
#define NETA_TXD_L4_TYPE_MASK		BIT(16)
#define NETA_TXD_L3_TYPE_MASK		BIT(17)
#define NETA_TXD_IP_CSUM_MASK		BIT(18)
#define NETA_TXD_Z_PAD_MASK		BIT(19)
#define NETA_TXD_L_DESC_MASK		BIT(20)
#define NETA_TXD_F_DESC_MASK		BIT(21)
#define NETA_TXD_BUFF_REL_MASK		BIT(22)
#define NETA_TXD_PKT_OFF_MASK		(0x3F800000)
#define NETA_TXD_L4_CSUM_MASK		(0xC0000000)
#define NETA_TXD_L4_FULL_CSUM_MASK	(0x40000000)
#define NETA_TXD_L4_NO_CSUM_MASK	(0x80000000)
/* word 1 */
#define NETA_TXD_L4_ICHK_MASK		(0x0000FFFF)
#define NETA_TXD_PKT_SIZE_MASK		(0xFFFF0000)

#define NETA_TXD_FLZ_DESC_MASK		(NETA_TXD_Z_PAD_MASK  | \
					 NETA_TXD_L_DESC_MASK | \
					 NETA_TXD_F_DESC_MASK)

#define NETA_TXD_SET_FIRST_LAST(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~(NETA_TXD_L_DESC_MASK | NETA_TXD_F_DESC_MASK)) | data)

#define NETA_TXD_SET_L3_OFF(desc, data)		\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~NETA_TXD_L3_OFF_MASK) | ((data << 0) & NETA_TXD_L3_OFF_MASK))
#define NETA_TXD_SET_IP_HLEN(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~NETA_TXD_IP_HLEN_MASK) | ((data << 8) & NETA_TXD_IP_HLEN_MASK))
#define NETA_TXD_SET_L4_TYPE(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~NETA_TXD_L4_TYPE_MASK) | ((data << 16) & NETA_TXD_L4_TYPE_MASK))
#define NETA_TXD_SET_L3_TYPE(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~NETA_TXD_L3_TYPE_MASK) | ((data << 17) & NETA_TXD_L3_TYPE_MASK))
#define NETA_TXD_SET_PKT_OFFSET(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~NETA_TXD_PKT_OFF_MASK) | ((data << 23) & NETA_TXD_PKT_OFF_MASK))
#define NETA_TXD_SET_L4_FRAG_CSUM(desc)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~NETA_TXD_L4_CSUM_MASK))
#define NETA_TXD_SET_L4_FULL_CSUM(desc)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~NETA_TXD_L4_CSUM_MASK) | NETA_TXD_L4_FULL_CSUM_MASK)
#define NETA_TXD_SET_L4_NO_CSUM(desc)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~NETA_TXD_L4_CSUM_MASK) | NETA_TXD_L4_NO_CSUM_MASK)

#define NETA_TXD_SET_PKT_SIZE(desc, data)	\
	((desc)->cmds[1] = ((desc)->cmds[1] & ~NETA_TXD_PKT_SIZE_MASK) | ((data << 16) & NETA_TXD_PKT_SIZE_MASK))

/******************** RxQ-desc *****************/
#define NETA_RXD_ERR_CRC		0x0
#define NETA_RXD_ERROR_SUM_OFF		16
#define NETA_RXD_ERROR_CODE_OFF		17
#define NETA_RXD_ERR_CODE_MASK		(BIT(17) | BIT(18))
#define NETA_RXD_VLAN_OFF		19
#define NETA_RXD_BPDU_OFF		20
#define NETA_RXD_L4_PRS_MASK		(BIT(21) | BIT(22))
#define NETA_RXD_L3_PRS_MASK		(BIT(24) | BIT(25))
#define NETA_RXD_FIRST_LAST_DESC_MASK	(BIT(26) | BIT(27))
#define NETA_RXD_IP_HEAD_OK_OFF		25
#define NETA_RXD_L4_CHK_OK_OFF		30
#define NETA_RXD_IPV4_FRG_OFF		31

#define NETA_RXD_GET_ES(desc)		(((desc)->cmds[0] >> NETA_RXD_ERROR_SUM_OFF) & 1)
#define NETA_RXD_GET_EC(desc)		(((desc)->cmds[0] & NETA_RXD_ERR_CODE_MASK) >> NETA_RXD_ERROR_CODE_OFF)
#define NETA_RXD_GET_VLAN_INFO(desc)	(((desc)->cmds[0] >> NETA_RXD_VLAN_OFF) & 1)
#define NETA_RXD_GET_L4_CHK_OK(desc)	(((desc)->cmds[0] >> NETA_RXD_L4_CHK_OK_OFF) & 1)
#define NETA_RXD_GET_L3_IP_FRAG(desc)	(((desc)->cmds[0] >> NETA_RXD_IPV4_FRG_OFF) & 1)
#define NETA_RXD_GET_IP_HDR_OK(desc)	(((desc)->cmds[0] >> NETA_RXD_IP_HEAD_OK_OFF) & 1)
#define NETA_RXD_GET_L4_PRS_INFO(desc)	(((desc)->cmds[0] & NETA_RXD_L4_PRS_MASK) >> 21)
#define NETA_RXD_GET_L3_PRS_INFO(desc)	(((desc)->cmds[0] & NETA_RXD_L3_PRS_MASK) >> 24)

/**
 * Reset an outq packet descriptor to default value.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 *
 */
static inline void neta_ppio_outq_desc_reset(struct neta_ppio_desc *desc)
{
	memset(desc, 0, sizeof(struct neta_ppio_desc));
	(desc)->cmds[0] |= (NETA_TXD_FLZ_DESC_MASK | NETA_TXD_L4_NO_CSUM_MASK);
}

/**
 * Set the physical address in an outq packet descriptor.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	addr	Physical DMA address containing the packet to be sent.
 *
 */
static inline void neta_ppio_outq_desc_set_phys_addr(struct neta_ppio_desc *desc, dma_addr_t addr)
{
	desc->cmds[2] = (u32)addr;
}

/**
 * Set the user specified cookie in an outq packet descriptor.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	cookie	User specified cookie.
 *
 */
static inline void neta_ppio_outq_desc_set_cookie(struct neta_ppio_desc *desc, u64 cookie)
{
	desc->cmds[6] = (u32)cookie;
}

/**
 * Set TXQ descriptors fields relevant for CSUM calculation
 *
 * @param[out]	desc		A pointer to a packet descriptor structure.
 * @param[in]	l3_type		The l3 type of the packet.
 * @param[in]	l4_type		The l4 type of the packet.
 * @param[in]	l3_offset	The l3 offset of the packet.
 * @param[in]	l4_offset	The l4 offset of the packet.
 * @param[in]	gen_l3_chk	Set to '1' to generate IPV4 checksum.
 * @param[in]	gen_l4_chk	Set to '1' to generate TCP/UDP checksum.
 *
 */
static inline void neta_ppio_outq_desc_set_proto_info(struct neta_ppio_desc *desc, enum neta_outq_l3_type l3_proto,
						     enum neta_outq_l4_type l4_proto, u8 l3_offs, u8 l4_offs,
						     int gen_l3_chk, int gen_l4_chk)
{
	/* Fields: L3_offset, IP_hdrlen, L3_type, G_IPv4_chk,
	 * G_L4_chk, L4_type; required only for checksum
	 * calculation
	 */
	NETA_TXD_SET_L3_OFF(desc, l3_offs);
	NETA_TXD_SET_IP_HLEN(desc, (l4_offs - l3_offs) / sizeof(u32));
	NETA_TXD_SET_L3_TYPE(desc, l3_proto);
	NETA_TXD_SET_L4_TYPE(desc, l4_proto);

	if (gen_l3_chk)
		desc->cmds[0] |= NETA_TXD_IP_CSUM_MASK;
	else
		desc->cmds[0] &= ~NETA_TXD_IP_CSUM_MASK;

	if (gen_l4_chk)
		NETA_TXD_SET_L4_FULL_CSUM(desc);
	else
		NETA_TXD_SET_L4_NO_CSUM(desc);
}

/**
 * TODO - Add a Marvell DSA Tag to the packet.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 *
 */
void neta_ppio_outq_desc_set_dsa_tag(struct neta_ppio_desc *desc);

/**
 * Set the offset of the packet (in bytes) from the beginning of the buffer
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	offset	The packet oofset inside buffer.
 *
 */
static inline void neta_ppio_outq_desc_set_pkt_offset(struct neta_ppio_desc *desc, u8  offset)
{
	NETA_TXD_SET_PKT_OFFSET(desc, offset);
}

/**
 * Set the packet length in an outq packet descriptor..
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	len	The packet length, not including CRC.
 *
 */
static inline void neta_ppio_outq_desc_set_pkt_len(struct neta_ppio_desc *desc, u16 len)
{
	NETA_TXD_SET_PKT_SIZE(desc, len);
}

/******** RXQ  ********/

/**
 * Get the physical DMA address from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	physical dma address
 */
static inline dma_addr_t neta_ppio_inq_desc_get_phys_addr(struct neta_ppio_desc *desc)
{
	return (u32)(desc->cmds[2]);
}

/**
 * Get the user defined cookie from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	cookie
 */
static inline u64 neta_ppio_inq_desc_get_cookie(struct neta_ppio_desc *desc)
{
	return (u32)(desc->cmds[4]);
}

/**
 * Get the packet length from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	packet length without CRC and Marvell Header
 */
static inline u16 neta_ppio_inq_desc_get_pkt_len(struct neta_ppio_desc *desc)
{
	u16 len = ((desc->cmds[1] & 0xffff0000) >> 16);

	len -= (MV_MH_SIZE + NETA_ETH_FCS_LEN);
	return len;
}

/**
 * Get the L3 type from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[out]	type	A pointer to l3 type.
 *
 */
static inline void neta_ppio_inq_desc_get_l3_type(struct neta_ppio_desc *desc, enum neta_inq_l3_type *type)
{
	*type  = NETA_RXD_GET_L3_PRS_INFO(desc);
}

/**
 * Get the L3 information from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[out]	type	A pointer to l3 type.
 *
 */
static inline void neta_ppio_inq_desc_get_l3_info(struct neta_ppio_desc *desc, enum neta_inq_l3_type *type, u8 *offset)
{
	*type  = NETA_RXD_GET_L3_PRS_INFO(desc);

	*offset = 14; /* DA + SA + ETYPE */
	if (NETA_RXD_GET_VLAN_INFO(desc))
		*offset += 4; /* VLAN tag */
}

/**
 * Get the L4 type from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[out]	type	A pointer to l4 type.
 *
 */
static inline void neta_ppio_inq_desc_get_l4_type(struct neta_ppio_desc *desc, enum neta_inq_l4_type *type)
{
	*type = NETA_RXD_GET_L4_PRS_INFO(desc) + 1;
}

/**
 * Get the Layer information from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[out]	type	A pointer to l4 type.
 *
 */
static inline void neta_ppio_inq_desc_get_l4_info(struct neta_ppio_desc *desc, enum neta_inq_l4_type *type, u8 *offset)
{
	enum neta_inq_l3_type l3_type;
	u8 l3_offset;

	*type = NETA_RXD_GET_L4_PRS_INFO(desc) + 1;

	neta_ppio_inq_desc_get_l3_info(desc, &l3_type, &l3_offset);

	if (l3_type == NETA_INQ_L3_TYPE_IPV4_OK)
		*offset = l3_offset + 20;
	else
		*offset = 0;
}

/**
 * TODO - Check if there is IPV4 fragmentation in an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	0 - not fragmented, 1 - fragmented.
 */
int neta_ppio_inq_desc_get_ip_isfrag(struct neta_ppio_desc *desc);

/**
 * Check if packet in inq packet descriptor has a L2 MAC (CRC) error condition.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	see enum neta_inq_desc_status.
 */

static inline enum neta_inq_desc_status neta_ppio_inq_desc_get_l2_pkt_error(struct neta_ppio_desc *desc)
{
	if (unlikely(NETA_RXD_GET_ES(desc)))
		return (1 + NETA_RXD_GET_EC(desc));
	return NETA_DESC_ERR_OK;
}

/**
 * Check if packet in inq packet descriptor has or L3 IPv4 error condition.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	see enum neta_inq_desc_status.
 */
static inline enum neta_inq_desc_status neta_ppio_inq_desc_get_l3_pkt_error(struct neta_ppio_desc *desc)
{
	if (!NETA_RXD_GET_IP_HDR_OK(desc))
		return NETA_DESC_ERR_IPV4_HDR;
	return NETA_DESC_ERR_OK;
}

/**
 * Check if packet in inq packet descriptor has IP/L4 (checksum) error condition.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	see enum neta_inq_desc_status.
 */
static inline enum neta_inq_desc_status neta_ppio_inq_desc_get_l4_pkt_error(struct neta_ppio_desc *desc)
{
	enum neta_inq_l4_type l4_info = NETA_RXD_GET_L4_PRS_INFO(desc) + 1;

	if (unlikely((l4_info == NETA_INQ_L4_TYPE_TCP || l4_info == NETA_INQ_L4_TYPE_UDP) &&
		     !NETA_RXD_GET_L3_IP_FRAG(desc) && !NETA_RXD_GET_L4_CHK_OK(desc)))
		return NETA_DESC_ERR_L4_CHECKSUM;
	return NETA_DESC_ERR_OK;
}

/**
 * Check if packet in inq packet descriptor has some error condition.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	see enum neta_inq_desc_status.
 */
static inline enum neta_inq_desc_status neta_ppio_inq_desc_get_pkt_error(struct neta_ppio_desc *desc)
{
	enum neta_inq_desc_status status;

	status = neta_ppio_inq_desc_get_l2_pkt_error(desc);
	if (unlikely(status))
		return status;

	status = neta_ppio_inq_desc_get_l3_pkt_error(desc);
	if (unlikely(status))
		return status;

	return neta_ppio_inq_desc_get_l4_pkt_error(desc);
}

/**
 * Send a batch of frames (single dscriptor) on an OutQ of PP-IO.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		qid	out-Q id on which to send the frames.
 * @param[in]		descs	A pointer to an array of descriptors representing the
 *				frames to be sent.
 * @param[in,out]	num	input: number of frames to be sent; output: number of frames sent.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_send(struct neta_ppio	*ppio,
		  u8			 qid,
		  struct neta_ppio_desc	*descs,
		  u16			*num);

/**
 * Send a batch of S/G frames (single or multiple dscriptors) on an OutQ of PP-IO.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		qid	out-Q id on which to send the frames.
 * @param[in]		descs	A pointer to an array of descriptors representing the
 *				frames fragments to be sent.
 * @param[in,out]	num	input: number of descriptors to be sent;
 *				output: number of descriptors sent.
 * @param[in,out]	pkts	input: number of S/G packets and their fragments to be sent
 *				output: number of full packets sent in pkts->num.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_send_sg(struct neta_ppio		*ppio,
		      u8			 qid,
		     struct neta_ppio_desc	*descs,
		     u16			*num,
		     struct neta_ppio_sg_pkts	*pkts);

/**
 * Get number of packets sent on a queue, since last call of this API.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		hif	A hif handle.
 * @param[in]		qid	out-Q id on which to send the frames.
 * @param[out]		num	Number of frames that were sent.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_get_num_outq_done(struct neta_ppio	*ppio,
				u8			qid,
				u16			*num);

/**
 * Receive packets on a ppio.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		qid	in-Q id on which to receive the frames.
 * @param[in]		descs	A pointer to an array of descriptors represents the
 *				received frames.
 * @param[in,out]	num	input: Max number of frames to receive;
 *				output: number of frames received.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_recv(struct neta_ppio		*ppio,
		   u8				qid,
		   struct neta_ppio_desc	*descs,
		   u16				*num);

/**
 * Fill RX descriptors ring with buffer pointers
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		qid	in-Q id to fill.
 * @param[in]		bufs	A pointer to an array of buffers to put to descriptors.
 * @param[in,out]	num_of_buffs	input: number of buffers in array;
 *					output: number of buffers were put
 *					to descriptors.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_inq_put_buffs(struct neta_ppio		*ppio,
			    u8				qid,
			    struct neta_buff_inf	*bufs,
			    u16				*num_of_buffs);

/**
 * Get all free buffers found in InQ.
 * Tis routine shal be used only to cleanup InQ.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		qid	in-Q id to get the buffer from.
 * @param[out]		bufs	A pointer to an array of buffers to free.
 * @param[in,out]	num_of_buffs	input: number of buffers in array
 *					output: number of buffers to free
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_inq_get_all_buffs(struct neta_ppio	*ppio,
				u8			qid,
				struct neta_buff_inf	*bufs,
				u16			*num_of_buffs);


/* Run-time Control API */

/**
 * Enable a ppio
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_enable(struct neta_ppio *ppio);

/**
 * Disable a ppio
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_disable(struct neta_ppio *ppio);

/**
 * Set ppio Ethernet MAC address
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		addr	Ethernet MAC address to configure .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_set_mac_addr(struct neta_ppio *ppio, const eth_addr_t addr);

/**
 * Get ppio Ethernet MAC address
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		addr	Configured ppio Ethernet MAC address .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_get_mac_addr(struct neta_ppio *ppio, eth_addr_t addr);


/* For debug only */
int neta_ppio_set_mtu(struct neta_ppio *ppio, u16 mtu);
int neta_ppio_get_mtu(struct neta_ppio *ppio, u16 *mtu);
int neta_ppio_set_mru(struct neta_ppio *ppio, u16 len);
int neta_ppio_get_mru(struct neta_ppio *ppio, u16 *len);

/**
 * Set ppio to promiscuous mode
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		enr	1 - enable, 0 - disable.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_set_promisc(struct neta_ppio *ppio, int en);

/**
 * Get ppio promiscuous mode
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		enr	1 - enabled, 0 - disabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_get_promisc(struct neta_ppio *ppio, int *en);

/**
 * Set ppio to listen to all multicast mode
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		enr	1 - enable, 0 - disable.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_set_mc_promisc(struct neta_ppio *ppio, int en);

/**
 * Get ppio all multicast mode
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		enr	1 - enabled, 0 - disabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_get_mc_promisc(struct neta_ppio *ppio, int *en);

/**
 * Add ppio ethernet multicast MAC address
 *
 * Can support up to:
 *	- 256 MC address filtering of IP multicast packets and
 *	  256 other MAC multicast addresses
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		addr	Ethernet MAC address to add .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_add_mac_addr(struct neta_ppio *ppio, const eth_addr_t addr);

/**
 * Remove ppio Ethernet multicast MAC address
 *
 * Allows to remove the mac address added by neta_ppio_add_mac_addr()
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		addr	Ethernet MAC address to remove .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_remove_mac_addr(struct neta_ppio *ppio, const eth_addr_t addr);

/**
 * Flush ppio all ethernet MAC addresses
 *
 * NOTE: Does not flush the mac_address set by neta_ppio_set_mac_addr().
.*
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		uc	1 - flush unicast list.
 * @param[in]		mc	1 - flush multicast list .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int neta_ppio_flush_mac_addrs(struct neta_ppio *ppio, int uc, int mc);

/**
 * Get ppio statistics
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		stats	Port statistics.
 *
 */
int neta_ppio_get_statistics(struct neta_ppio *ppio, struct neta_ppio_statistics *stats);

int neta_ppio_set_link_state(struct neta_ppio *ppio, int en);
int neta_ppio_get_link_state(struct neta_ppio *ppio, int *en);

int neta_ppio_set_loopback(struct neta_ppio *ppio, int en);

/** @} */ /* end of grp_neta_io */

#endif /* __MV_NETA_PPIO_H__ */
