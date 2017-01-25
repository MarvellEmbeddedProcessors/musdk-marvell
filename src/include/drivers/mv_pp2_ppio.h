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

#ifndef __MV_PP2_PPIO_H__
#define __MV_PP2_PPIO_H__

#include "mv_std.h"

#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"

/** @addtogroup grp_pp2_io Packet Processor: I/O
 *
 *  Packet Processor I/O API documentation
 *
 *  @{
 */

struct pp2_ppio;

#define ETH_ADDR_NUM_OCTETS	6 /**< Number of octets (8-bit bytes) in an ethernet address */

#define PP2_PPIO_MAX_NUM_TCS	8 /**< Max. number of TCs per ppio. */
#define PP2_PPIO_MAX_NUM_OUTQS	8 /**< Max. number of outqs per ppio. */
#define PP2_PPIO_TC_MAX_POOLS	2 /**< Max. number of bpools per TC. */
#define PP2_PPIO_MAX_NUM_HASH	4

typedef u8 eth_addr_t[ETH_ADDR_NUM_OCTETS];

enum pp2_ppio_type {
	PP2_PPIO_T_LOG = 0,	/*  Logical-port is only a set of Out-Qs and In-TCs (i.e. no link, l2-filters) */
	PP2_PPIO_T_NIC		/*  NIC is a logical-port with link and l2-filters */
};

enum pp2_ppio_hash_type {
	PP2_PPIO_HASH_T_NONE = 0,	/* Invalid hash type */
	PP2_PPIO_HASH_T_2_TUPLE,	/* IP-src, IP-dst */
	PP2_PPIO_HASH_T_5_TUPLE,	/* IP-src, IP-dst, IP-Prot, L4-src, L4-dst */
	PP2_PPIO_HASH_T_OUT_OF_RANGE
};

enum pp2_ppio_outqs_sched_mode {
	PP2_PPIO_SCHED_M_NONE = 0,
};

/**
 * ppio inq parameters
 *
 */
struct pp2_ppio_inq_params {
	u32	size; /**< q_size in number of descriptors */
};

/**
 * ppio tc parameters
 *
 */
struct pp2_ppio_tc_params {
	int				 use_hash; /**< Use hashing mechanism */
	u16				 pkt_offset; /**< pkt offset, must be multiple of 32 bytes */
	u16				 num_in_qs; /**< number of inqs */
	struct pp2_ppio_inq_params	*inqs_params; /**< pointer to the tc's inq parameters */
	struct pp2_bpool		*pools[PP2_PPIO_TC_MAX_POOLS]; /**< bpools used by the tc */
/* TODO: future:
 *	u8				 qos;
 */
};

/**
 * ppio inq related parameters
 *
 */
struct pp2_ppio_inqs_params {
	u16				 num_tcs; /**< Number of tcs */
	struct pp2_ppio_tc_params	 tcs_params[PP2_PPIO_MAX_NUM_TCS]; /**< Parameters for each tc */
	/** hash engine may be selected only according to "parser-results";
	 * therefore, we put hash selection on a per port basis.
	 */
	enum pp2_ppio_hash_type		 hash_type[PP2_PPIO_MAX_NUM_HASH];
};

/**
 * ppio outq parameters
 *
 */
struct pp2_ppio_outq_params {
	u32	size;	/**< q_size in number of descriptors */
	u8	weight; /**< The weight is relative among the PP-IO out-Qs */

/* TODO: add rate-limit (burst, throughput) */
};

/**
 * ppio outq related parameters
 *
 */
struct pp2_ppio_outqs_params {
	u16				 num_outqs; /**< Number of outqs */
	struct pp2_ppio_outq_params	 outqs_params[PP2_PPIO_MAX_NUM_OUTQS]; /**< Parameters for each outq */

/* TODO: scheduling mode and parameters (WRR/Strict)
 *	enum pp2_ppio_outqs_sched_mode	sched_mode;
 */
};

/**
 * ppio parameters
 *
 */
struct pp2_ppio_params {
	/** Used for DTS acc to find appropriate "physical" PP-IO obj;
	 * E.g. "eth-0:0" means PPv2[0],port[0]
	 */
	const char			*match;

	enum pp2_ppio_type		 type; /**<  ppio type. TODO: currently only support "NIC" */
	struct pp2_ppio_inqs_params	 inqs_params; /**<  ppio inq parameters structure */
	struct pp2_ppio_outqs_params	 outqs_params; /**<  ppio outq parameters structure */
/* TODO: do we need extra pools per port?
 *	struct pp2_bpool		*pools[PP2_PPIO_TC_MAX_POOLS];
 */
};

/**
 * Initialize a ppio
 *
 * @param[in]	params	A pointer to structure that contains all relevant parameters.
 * @param[out]	ppio	A pointer to opaque ppio handle of type 'struct pp2_ppio *'.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int pp2_ppio_init(struct pp2_ppio_params *params, struct pp2_ppio **ppio);

/**
 * TODO - Destroy a ppio
 *
 * @param[in]	ppio	A ppio handle.
 *
 */
void pp2_ppio_deinit(struct pp2_ppio *ppio);

/****************************************************************************
 *	Run-time API
 ****************************************************************************/

#define PP2_PPIO_DESC_NUM_WORDS	8
#define PP2_PPIO_DESC_NUM_FRAGS	16 /* TODO: check if there’s HW limitation */

struct pp2_ppio_desc {
	u32			 cmds[PP2_PPIO_DESC_NUM_WORDS];
};

struct pp2_ppio_sg_desc {
	u8			 num_frags;
	struct pp2_ppio_desc	 descs[PP2_PPIO_DESC_NUM_FRAGS];
};

enum pp2_outq_l3_type {
	PP2_OUTQ_L3_TYPE_IPV4 = 0,
	PP2_OUTQ_L3_TYPE_IPV6,
	PP2_OUTQ_L3_TYPE_OTHER
};

enum pp2_outq_l4_type {
	PP2_OUTQ_L4_TYPE_TCP = 0,
	PP2_OUTQ_L4_TYPE_UDP,
	PP2_OUTQ_L4_TYPE_OTHER
};

enum pp2_inq_l3_type {
	PP2_INQ_L3_TYPE_NA = 0,
	PP2_INQ_L3_TYPE_IPV4_NO_OPTS,
	PP2_INQ_L3_TYPE_IPV4_OK,
	PP2_INQ_L3_TYPE_IPV4_TTL_ZERO,
	PP2_INQ_L3_TYPE_IPV6_NO_EXT,
	PP2_INQ_L3_TYPE_IPV6_EXT
};

enum pp2_inq_l4_type {
	PP2_INQ_L4_TYPE_OTHER = 0,
	PP2_INQ_L4_TYPE_TCP,
	PP2_INQ_L4_TYPE_UDP
};

enum pp2_inq_desc_status {
	PP2_DESC_ERR_MAC_OK = 0,
	PP2_DESC_ERR_MAC_CRC,
	PP2_DESC_ERR_MAC_RESOURCE = 0,
	PP2_DESC_ERR_MAC_OVERRUN,
	PP2_DESC_ERR_IPV4_HDR,
	PP2_DESC_ERR_L4_CHECKSUM
};

/******** TXQ  ********/

/* TODO: Add PTP, PME, L4Icheck */

/* NOTE: Following functions must be called
 *	pp2_ppio_outq_desc_reset ()
 *	pp2_ppio_outq_desc_set_phys_addr()
 *	pp2_ppio_outq_desc_set_proto_info()
 *	pp2_ppio_outq_desc_set_pkt_len()
 */

/******************** TxQ-desc *****************/
/* cmd 0 */
#define TXD_FIRST                  (0x2)
#define TXD_LAST                   (0x1)
#define TXD_FIRST_LAST             (0x3)
#define TXD_IP_CHK_DISABLE         (0x1)
#define TXD_IP_CHK_ENABLE          (0x0)
#define TXD_L4_CHK_ENABLE          (0x0)
#define TXD_L4_CHK_FRG_ENABLE      (0x1)
#define TXD_L4_CHK_DISABLE         (0x2)

#define TXD_L_MASK                 (0x10000000)
#define TXD_F_MASK                 (0x20000000)
#define TXD_FL_MASK                (TXD_F_MASK | TXD_L_MASK)
#define TXD_FORMAT_MASK            (0x40000000)
#define TXD_L3_TYPE_MASK           (0x0C000000)
#define TXD_L4_TYPE_MASK           (0x03000000)
#define TXD_PKT_OFF_EXT_MASK       (0x00100000)
#define TXD_POOL_ID_MASK           (0x000F0000)
#define TXD_GEN_L4_CHK_MASK        (0x00006000)
#define TXD_GEN_IP_CHK_MASK        (0x00008000)
#define TXD_IP_HEAD_LEN_MASK       (0x00001F00)

#define TXD_BUFMODE_MASK           (0x00000080)
#define TXD_L3_OFFSET_MASK         (0x0000007F)

/* cmd 1 */
#define TXD_PKT_OFF_MASK           (0x000000FF)
#define TXD_DEST_QID_MASK          (0x0000FF00)
#define TXD_BYTE_COUNT_MASK        (0xFFFF0000)
/* cmd 4 */
#define TXD_BUF_PHYS_LO_MASK       (0xFFFFFFFF)
/* cmd 5 */
#define TXD_BUF_PHYS_HI_MASK       (0x000000FF)
#define TXD_PTP_DESC_MASK          (0xFFFFFF00)
/* cmd 6 */
#define TXD_BUF_VIRT_LO_MASK       (0xFFFFFFFF)
/* cmd 7 */
#define TXD_BUF_VIRT_HI_MASK       (0x000000FF)

/******************** RxQ-desc *****************/
/* cmd 0 */
#define RXD_L3_OFF_MASK            (0x0000007F)
#define RXD_IPHDR_LEN_MASK         (0x00001F00)
#define RXD_EC_MASK                (0x00006000)
#define RXD_ES_MASK                (0x00008000)
#define RXD_POOL_ID_MASK           (0x000F0000)
#define RXD_HWF_SYNC_MASK          (0x00200000)
#define RXD_L4_CHK_OK_MASK         (0x00400000)
#define RXD_L4_IP_FRAG_MASK        (0x00800000)
#define RXD_IP_HDR_ERR_MASK        (0x01000000)
#define RXD_L4_PRS_INFO_MASK       (0x0E000000)
#define RXD_L3_PRS_INFO_MASK       (0x70000000)
#define RXD_BUF_HDR_MASK           (0x80000000)
/* cmd 1 */
#define RXD_BYTE_COUNT_MASK        (0xFFFF0000)
/* cmd 4 */
#define RXD_BUF_PHYS_LO_MASK       (0xFFFFFFFF)
/* cmd 5 */
#define RXD_BUF_PHYS_HI_MASK       (0x000000FF)
#define RXD_KEY_HASH_MASK          (0xFFFFFF00)
/* cmd 6 */
#define RXD_BUF_VIRT_LO_MASK       (0xFFFFFFFF)
/* cmd 7 */
#define RXD_BUF_VIRT_HI_MASK       (0x000000FF)

#define DM_RXD_GET_L3_OFF(desc)         (((desc)->cmds[0] & RXD_L3_OFF_MASK) >> 0)
#define DM_RXD_GET_IPHDR_LEN(desc)      (((desc)->cmds[0] & RXD_IPHDR_LEN_MASK) >> 8)
#define DM_RXD_GET_EC(desc)             (((desc)->cmds[0] & RXD_EC_MASK) >> 13)
#define DM_RXD_GET_ES(desc)             (((desc)->cmds[0] & RXD_ES_MASK) >> 15)
#define DM_RXD_GET_POOL_ID(desc)        (((desc)->cmds[0] & RXD_POOL_ID_MASK) >> 16)
#define DM_RXD_GET_HWF_SYNC(desc)       (((desc)->cmds[0] & RXD_HWF_SYNC_MASK) >> 21)
#define DM_RXD_GET_L4_CHK_OK(desc)      (((desc)->cmds[0] & RXD_L4_CHK_OK_MASK) >> 22)
#define DM_RXD_GET_L4_IP_FRAG(desc)     (((desc)->cmds[0] & RXD_L4_IP_FRAG_MASK) >> 23)
#define DM_RXD_GET_IP_HDR_ERR(desc)     (((desc)->cmds[0] & RXD_IP_HDR_ERR_MASK) >> 24)
#define DM_RXD_GET_L4_PRS_INFO(desc)    (((desc)->cmds[0] & RXD_L4_PRS_INFO_MASK) >> 25)
#define DM_RXD_GET_L3_PRS_INFO(desc)    (((desc)->cmds[0] & RXD_L3_PRS_INFO_MASK) >> 28)
#define DM_RXD_GET_BUF_HDR(desc)        (((desc)->cmds[0] & RXD_BUF_HDR_MASK) >> 31)

#define DM_TXD_SET_GEN_L4_CHK(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_GEN_L4_CHK_MASK) | (data << 13 & TXD_GEN_L4_CHK_MASK))
#define DM_TXD_SET_GEN_IP_CHK(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_GEN_IP_CHK_MASK) | (data << 15 & TXD_GEN_IP_CHK_MASK))
#define DM_TXD_SET_FIRST_LAST(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_FL_MASK) | (data << 28 & TXD_FL_MASK))
#define DM_TXD_SET_L3_OFF(desc, data)	\
		((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_L3_OFFSET_MASK) | (data << 0 & TXD_L3_OFFSET_MASK))
#define DM_TXD_SET_IP_HEAD_LEN(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_IP_HEAD_LEN_MASK) | (data << 8 & TXD_IP_HEAD_LEN_MASK))
#define DM_TXD_SET_L3_TYPE(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_L3_TYPE_MASK) | (data << 26 & TXD_L3_TYPE_MASK))
#define DM_TXD_SET_L4_TYPE(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_L4_TYPE_MASK) | (data << 24 & TXD_L4_TYPE_MASK))

/**
 * Reset an outq packet descriptor to default value.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 *
 */
static inline void pp2_ppio_outq_desc_reset(struct pp2_ppio_desc *desc)
{
	desc->cmds[0] = desc->cmds[1] = desc->cmds[2] = desc->cmds[3] =
	desc->cmds[5] = desc->cmds[7] = 0;

	/* Do not generate L4 nor IPv4 header checksum by default */
	DM_TXD_SET_GEN_IP_CHK(desc, TXD_IP_CHK_DISABLE);
	DM_TXD_SET_GEN_L4_CHK(desc, TXD_L4_CHK_DISABLE);
	DM_TXD_SET_FIRST_LAST(desc, TXD_FIRST_LAST);
}

/**
 * Set the physical address in an outq packet descriptor.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	addr	Physical DMA address containing the packet to be sent.
 *
 */
static inline void pp2_ppio_outq_desc_set_phys_addr(struct pp2_ppio_desc *desc, dma_addr_t addr)
{
	desc->cmds[4] = (u32)addr;
	desc->cmds[5] = (desc->cmds[5] & ~TXD_BUF_PHYS_HI_MASK) | ((u64)addr >> 32 & TXD_BUF_PHYS_HI_MASK);
}

/**
 * Set the user specified cookie in an outq packet descriptor.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	cookie	User specified cookie.
 *
 */
static inline void pp2_ppio_outq_desc_set_cookie(struct pp2_ppio_desc *desc, u64 cookie)
{
	desc->cmds[6] = (u32)cookie;
	desc->cmds[7] = (desc->cmds[7] & ~TXD_BUF_PHYS_HI_MASK) | (cookie >> 32 & TXD_BUF_PHYS_HI_MASK);
}

/**
 * Set the ppv2 pool to return the buffer to, after sending the packet.
 * Calling this API will cause PPV2 HW to return the buffer to the PPV2 Buffer Manager.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	pool	A bpool handle.
 *
 */
void pp2_ppio_outq_desc_set_pool(struct pp2_ppio_desc *desc, struct pp2_bpool *pool);

/**
 * Set the protocol info in an outq packet descriptor.
 * This API must be called, if the PPV2 needs to generate l3 or l4 checksum.
 * Otherwise, it may, or may not, be called.
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
static inline void pp2_ppio_outq_desc_set_proto_info(struct pp2_ppio_desc *desc, enum pp2_outq_l3_type l3_type,
						     enum pp2_outq_l4_type l4_type, u8  l3_offset, u8 l4_offset,
						     int gen_l3_chk, int gen_l4_chk)
{
	DM_TXD_SET_L3_TYPE(desc, l3_type);
	DM_TXD_SET_L4_TYPE(desc, l4_type);
	DM_TXD_SET_L3_OFF(desc, l3_offset);
	DM_TXD_SET_IP_HEAD_LEN(desc, (l4_offset - l3_offset));
	DM_TXD_SET_GEN_IP_CHK(desc, ((gen_l3_chk) ? TXD_IP_CHK_ENABLE : TXD_IP_CHK_DISABLE));
	DM_TXD_SET_GEN_L4_CHK(desc, ((gen_l4_chk) ? TXD_L4_CHK_ENABLE : TXD_L4_CHK_DISABLE));
}

/**
 * TODO - Add a Marvell DSA Tag to the packet.
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 *
 */
void pp2_ppio_outq_desc_set_dsa_tag(struct pp2_ppio_desc *desc);

static inline void pp2_ppio_outq_desc_set_pkt_offset(struct pp2_ppio_desc *desc, u8  offset)
{
	desc->cmds[1] = (u32)offset;
}

/**
 * Set the packet length in an outq packet descriptor..
 *
 * @param[out]	desc	A pointer to a packet descriptor structure.
 * @param[in]	len	The packet length, not including CRC.
 *
 */
static inline void pp2_ppio_outq_desc_set_pkt_len(struct pp2_ppio_desc *desc, u16 len)
{
	desc->cmds[1] = (desc->cmds[1] & ~TXD_BYTE_COUNT_MASK) | (len << 16 & TXD_BYTE_COUNT_MASK);
}

/******** RXQ  ********/

/* TODO: Timestamp, L4IChk */

/**
 * Get the physical DMA address from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	physical dma address
 */
static inline dma_addr_t pp2_ppio_inq_desc_get_phys_addr(struct pp2_ppio_desc *desc)
{
	return ((u64)((desc->cmds[5] & RXD_BUF_PHYS_HI_MASK) >> 0) << 32) |
		((u64)((desc->cmds[4] & RXD_BUF_PHYS_LO_MASK) >> 0));
}

/**
 * Get the user defined cookie from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	cookie
 */
static inline u64 pp2_ppio_inq_desc_get_cookie(struct pp2_ppio_desc *desc)
{
	return ((u64)((desc->cmds[7] & RXD_BUF_VIRT_HI_MASK) >> 0) << 32) |
		((u64)((desc->cmds[6] & RXD_BUF_VIRT_LO_MASK) >> 0));
}

struct pp2_ppio *pp2_ppio_inq_desc_get_pp_io(struct pp2_ppio_desc *desc); /*Note: under _DEBUG_*/

/**
 * Get the packet length from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	packet length
 */
static inline u16 pp2_ppio_inq_desc_get_pkt_len(struct pp2_ppio_desc *desc)
{
	return ((desc->cmds[1] & RXD_BYTE_COUNT_MASK) >> 16);
}

/**
 * Get the Layer information from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[out]	type	A pointer to l3 type.
 * @param[out]	offset	A pointer to l3 offset.
 *
 */
static inline void pp2_ppio_inq_desc_get_l3_info(struct pp2_ppio_desc *desc, enum pp2_inq_l3_type *type, u8 *offset)
{
	*type   = (desc->cmds[0] & RXD_L3_PRS_INFO_MASK) >> 28;
	*offset = (desc->cmds[0] & RXD_L3_OFF_MASK) >> 0;
}

/**
 * Get the Layer information from an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 * @param[out]	type	A pointer to l4 type.
 * @param[out]	offset	A pointer to l4 offset.
 *
 */
static inline void pp2_ppio_inq_desc_get_l4_info(struct pp2_ppio_desc *desc, enum pp2_inq_l4_type *type, u8 *offset)
{
	*type   = (desc->cmds[0] & RXD_L4_PRS_INFO_MASK) >> 25;
	*offset = ((desc->cmds[0] & RXD_L3_OFF_MASK) >> 0) + ((desc->cmds[0] & RXD_IPHDR_LEN_MASK) >> 8);
}

/**
 * TODO - Check if there is IPV4 fragmentation in an inq packet descriptor.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	0 - not fragmented, 1 - fragmented.
 */
int pp2_ppio_inq_desc_get_ip_isfrag(struct pp2_ppio_desc *desc);

struct pp2_bpool *pp2_ppio_inq_desc_get_bpool(struct pp2_ppio_desc *desc);

/**
 * Check if packet in inq packet descriptor has a MAC (CRC) or IP/L4 (checksum) error condition.
 *
 * @param[in]	desc	A pointer to a packet descriptor structure.
 *
 * @retval	see enum pp2_inq_desc_status.
 */
static inline enum pp2_inq_desc_status pp2_ppio_inq_desc_get_pkt_error(struct pp2_ppio_desc *desc)
{
	if (unlikely(DM_RXD_GET_ES(desc)))
		return (1 + DM_RXD_GET_EC(desc));
	if (unlikely(DM_RXD_GET_IP_HDR_ERR(desc)))
		return PP2_DESC_ERR_IPV4_HDR;
	if (likely(DM_RXD_GET_L4_CHK_OK(desc)))
		return PP2_DESC_ERR_MAC_OK;

	return PP2_DESC_ERR_L4_CHECKSUM;
}

/**
 * Send a batch of frames (single dscriptor) on an OutQ of PP-IO.
 *
 * The routine assumes that the BM-Pool is either freed by HW (by appropriate desc
 * setter) or by the MUSDK client SW.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		hif	A hif handle.
 * @param[in]		qid	out-Q id on which to send the frames.
 * @param[in]		descs	A pointer to an array of descriptors representing the
 *				frames to be sent.
 * @param[in,out]	num	input: number of frames to be sent; output: number of frames sent.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_send(struct pp2_ppio	*ppio,
		  struct pp2_hif	*hif,
		  u8			 qid,
		  struct pp2_ppio_desc	*descs,
		  u16			*num);

/**
 * TODO - Send a batch of S/G frames (single or multiple dscriptors) on an OutQ of PP-IO.
 *
 * The routine assumes that the BM-Pool is either freed by HW (by appropriate desc
 * setter) or by the MUSDK client SW.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		hif	A hif handle.
 * @param[in]		qid	out-Q id on which to send the frames.
 * @param[in]		descs	A pointer to an array of S/G-descriptors representing the
 *				frames to be sent.
 * @param[in,out]	num	input: number of frames to be sent; output: number of frames sent.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_send_sg(struct pp2_ppio		*ppio,
		     struct pp2_hif		*hif,
		     u8			 qid,
		     struct pp2_ppio_sg_desc	*descs,
		     u16			*num);

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
int pp2_ppio_get_num_outq_done(struct pp2_ppio	*ppio,
			       struct pp2_hif	*hif,
			       u8		 qid,
			       u16		*num);

/**
 * Receive packets on a ppio.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		tc	traffic class on which to receive frames
 * @param[in]		qid	out-Q id on which to receive the frames.
 * @param[in]		descs	A pointer to an array of descriptors represents the
 *				received frames.
 * @param[in,out]	num	input: Max number of frames to receive;
 *				output: number of frames received.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_recv(struct pp2_ppio	*ppio,
		  u8			 tc,
		  u8			 qid,
		  struct pp2_ppio_desc	*descs,
		  u16			*num);

/****************************************************************************
 *	Run-time Control API
 ****************************************************************************/

/**
 * Enable a ppio
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_enable(struct pp2_ppio *ppio);

/**
 * Disable a ppio
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_disable(struct pp2_ppio *ppio);

/**
 * Set ppio Ethernet MAC address
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		addr	Ethernet MAC address to configure .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_set_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr);

/**
 * Get ppio Ethernet MAC address
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		addr	Configured ppio Ethernet MAC address .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_get_mac_addr(struct pp2_ppio *ppio, eth_addr_t addr);
int pp2_ppio_set_mtu(struct pp2_ppio *ppio, u16 mtu); /* For debug only, check mtu during  pkt_send() */
int pp2_ppio_get_mtu(struct pp2_ppio *ppio, u16 *mtu);
int pp2_ppio_set_mru(struct pp2_ppio *ppio, u16 len);
int pp2_ppio_get_mru(struct pp2_ppio *ppio, u16 *len);

/**
 * Set ppio to promiscuous mode
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		enr	1 - enable, 0 - disable.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_set_uc_promisc(struct pp2_ppio *ppio, int en);

/**
 * Get ppio promiscuous mode
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		enr	1 - enabled, 0 - disabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_get_uc_promisc(struct pp2_ppio *ppio, int *en);

/**
 * Set ppio to listen to all multicast mode
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		enr	1 - enable, 0 - disable.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_set_mc_promisc(struct pp2_ppio *ppio, int en);

/**
 * Get ppio all multicast mode
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[out]		enr	1 - enabled, 0 - disabled.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_get_mc_promisc(struct pp2_ppio *ppio, int *en);

/**
 * Add ppio Ethernet MAC address
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		addr	Ethernet MAC address to add .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_add_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr);

/**
 * Remove ppio Ethernet MAC address
 *
 * Allows to remove the mac_address set  by pp2_ppio_set_mac_addr().
  *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		addr	Ethernet MAC address to remove .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_remove_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr);

/**
 * Flush ppio Ethernet MAC address
 *
 * Does not flush the mac_address set  by pp2_ppio_set_mac_addr().
.*
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		uc	1 - flush multicast list.
 * @param[in]		mc	1 - flush unicast list .
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_flush_mac_addrs(struct pp2_ppio *ppio, int uc, int mc);

int pp2_ppio_get_phys_in_q(struct pp2_ppio *ppio, u8 tc, u8 qid, u8 *pq);
int pp2_ppio_get_outq_state(struct pp2_ppio *ppio, u8 qid);

/**
 * Add ppio filtering vlan id
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		vlan	vlan id to add.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_add_vlan(struct pp2_ppio *ppio, u16 vlan);

/**
 * Remove ppio filtering vlan id
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		vlan	vlan id to remove.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_remove_vlan(struct pp2_ppio *ppio, u16 vlan);

/**
 * Flush ppio filtering vlan id's
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_flush_vlan(struct pp2_ppio *ppio);

/*TODO: link state ???*/
/*TODO: counters/statistics for port/Q*/

/** @} */ /* end of grp_pp2_io */

#endif /* __MV_PP2_PPIO_H__ */
