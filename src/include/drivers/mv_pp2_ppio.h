/******************************************************************************
	Copyright (C) 2016 Marvell International Ltd.

  If you received this File from Marvell, you may opt to use, redistribute
  and/or modify this File under the following licensing terms.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

  	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.

  	* Redistributions in binary form must reproduce the above copyright
  	  notice, this list of conditions and the following disclaimer in the
  	  documentation and/or other materials provided with the distribution.

  	* Neither the name of Marvell nor the names of its contributors may be
  	  used to endorse or promote products derived from this software
	  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef __MV_PP2_PPIO_H__
#define __MV_PP2_PPIO_H__

#include "mv_std.h"

#include "mv_pp2_hif.h"
#include "mv_pp2_bpool.h"


/**************************************************************************//**
	Initialization API
*//***************************************************************************/

struct pp2_ppio;

#define ETH_ADDR_NUM_OCTETS	6 /**< Number of octets (8-bit bytes) in an ethernet address */

#define PP2_PPIO_MAX_NUM_TCS	8
#define PP2_PPIO_MAX_NUM_OUTQS	8
#define PP2_PPIO_TC_MAX_POOLS	2
#define PP2_PPIO_MAX_NUM_HASH	4


typedef u8 eth_addr_t[ETH_ADDR_NUM_OCTETS];


enum pp2_ppio_type {
	PP2_PPIO_T_LOG = 0,	/**< Logical-port is only a set of Out-Qs and In-TCs (i.e. no link, l2-filters) */
	PP2_PPIO_T_NIC		/**< NIC is a logical-port with link and l2-filters */
};

enum pp2_ppio_hash_type {
	PP2_PPIO_HASH_T_NONE = 0,	/**< Invalid hash type */
	PP2_PPIO_HASH_T_2_TUPLE,	/**< IP-src, IP-dst */
	PP2_PPIO_HASH_T_5_TUPLE,	/**< IP-src, IP-dst, IP-Prot, L4-src, L4-dst */
	PP2_PPIO_HASH_T_OUT_OF_RANGE
};

enum pp2_ppio_outqs_sched_mode {
	PP2_PPIO_SCHED_M_NONE = 0,
};

struct pp2_ppio_inq_params {
	u32	size; /**< Q size – number of descriptors */
};

struct pp2_ppio_tc_params {
	int				 use_hash;
	u16				 pkt_offset;    /* Must be multiple of 32 bytes.*/
	u16				 num_in_qs;
	struct pp2_ppio_inq_params	*inqs_params;
	struct pp2_bpool		*pools[PP2_PPIO_TC_MAX_POOLS];
/* TODO: future:
	u8				 qos;
*/
};

struct pp2_ppio_inqs_params {
	u16				 num_tcs;
	struct pp2_ppio_tc_params	 tcs_params[PP2_PPIO_MAX_NUM_TCS];
	/** hash engine may be selected only according to "parser-results";
	 * therefore, we put hash selection on a per port basis. */
	enum pp2_ppio_hash_type		 hash_type[PP2_PPIO_MAX_NUM_HASH];
};

struct pp2_ppio_outq_params {
	u32	size;	/**< Q size – number of descriptors */
	u8	weight; /**< The weight is relative among the PP-IO out-Qs */

/* TODO: add rate-limit (burst, throughput) */
};

struct pp2_ppio_outqs_params {
	u16				 num_outqs;
	struct pp2_ppio_outq_params	 outqs_params[PP2_PPIO_MAX_NUM_OUTQS];

/* TODO: scheduling mode and parameters (WRR/Strict)
	enum pp2_ppio_outqs_sched_mode	sched_mode;
*/
};

struct pp2_ppio_params {
	/** Used for DTS acc to find appropriate "physical" PP-IO obj;
	 * E.g. "eth-0:0" means PPv2[0],port[0] */
	const char			*match;

	enum pp2_ppio_type		 type; /* TODO: support only "NIC" type for short-term! */
	struct pp2_ppio_inqs_params	 inqs_params;
	struct pp2_ppio_outqs_params	 outqs_params;
/* TODO: do we need extra pools per port?
	struct pp2_bpool		*pools[PP2_PPIO_TC_MAX_POOLS];
*/
};


int pp2_ppio_init(struct pp2_ppio_params *params, struct pp2_ppio **ppio);

void pp2_ppio_deinit(struct pp2_ppio *ppio);


/**************************************************************************//**
	Run-time API
*//***************************************************************************/

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
	PP2_OUTQ_L4_TYPE_UDP
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
	PP2_DESC_ERR_MAC_RESOURCE = 0,
	PP2_DESC_ERR_MAC_OVERRUN,
	PP2_DESC_ERR_MAC_CRC,
	PP2_DESC_ERR_IPV4_HDR,
	PP2_DESC_ERR_L4_CHECKSUM
};

/******** TXQ  ********/

/* TODO: Add PTP, PME, L4Icheck */

/*NOTE: Following functions must be called
	pp2_ppio_outq_desc_reset ()
	pp2_ppio_outq_desc_set_phys_addr()
	pp2_ppio_outq_desc_set_proto_info()
	pp2_ppio_outq_desc_set_pkt_len()
*/

/******************** TxQ-desc *****************/
/* cmd 0 */
#define TXD_L_MASK                 (0x10000000)
#define TXD_F_MASK                 (0x20000000)
#define TXD_FL_MASK                (TXD_F_MASK | TXD_L_MASK)
#define TXD_FORMAT_MASK            (0x40000000)
#define TXD_PKT_OFF_EXT_MASK       (0x00100000)
#define TXD_POOL_ID_MASK           (0x000F0000)
#define TXD_GEN_L4_CHK_MASK        (0x00006000)
#define TXD_GEN_IP_CHK_MASK        (0x00008000)
#define TXD_BUFMODE_MASK           (0x00000080)
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

static inline void pp2_ppio_outq_desc_reset (struct pp2_ppio_desc *desc)
{
#define DM_TXD_SET_GEN_L4_CHK(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_GEN_L4_CHK_MASK) | (data << 13 & TXD_GEN_L4_CHK_MASK))
#define DM_TXD_SET_GEN_IP_CHK(desc, data)	\
	((desc)->cmds[0] = ((desc)->cmds[0] & ~TXD_GEN_IP_CHK_MASK) | (data << 15 & TXD_GEN_IP_CHK_MASK))

	desc->cmds[0] = desc->cmds[1] = desc->cmds[2] = desc->cmds[3] =
	desc->cmds[5] = desc->cmds[7] = 0;

	/* Do not generate L4 nor IPv4 header checksum by default */
	DM_TXD_SET_GEN_IP_CHK(desc, 0x01);
	DM_TXD_SET_GEN_L4_CHK(desc, 0x02);
}

static inline void pp2_ppio_outq_desc_set_phys_addr(struct pp2_ppio_desc *desc, dma_addr_t addr)
{
	desc->cmds[4] = (u32)addr;
	desc->cmds[5] = (desc->cmds[5] & ~TXD_BUF_PHYS_HI_MASK) | ((u64)addr >> 32 & TXD_BUF_PHYS_HI_MASK);
}

static inline void pp2_ppio_outq_desc_set_cookie(struct pp2_ppio_desc *desc, u64 cookie)
{
	desc->cmds[6] = (u32)cookie;
	desc->cmds[7] = (desc->cmds[7] & ~TXD_BUF_PHYS_HI_MASK) | (cookie >> 32 & TXD_BUF_PHYS_HI_MASK);
}

static inline void pp2_ppio_outq_desc_set_pool(struct pp2_ppio_desc *desc, struct pp2_bpool *pool)
{
	/* TODO: we write here the pool hardcoded!!! */
	desc->cmds[0] = (desc->cmds[0] & ~(TXD_POOL_ID_MASK | TXD_BUFMODE_MASK)) |
		(0 << 16 & TXD_POOL_ID_MASK) |
		(1 << 7 & TXD_BUFMODE_MASK);
}

void pp2_ppio_outq_desc_set_proto_info(struct pp2_ppio_desc *desc,
				       enum pp2_outq_l3_type l3_type,
				       enum pp2_outq_l4_type l4_type,
				       u8  l3_offset,
				       u8 l4_offset,
				       int gen_l3_chk,
				       int gen_l4_chk
				      );
void pp2_ppio_outq_desc_set_dsa_tag(struct pp2_ppio_desc *desc);

static inline void pp2_ppio_outq_desc_set_pkt_offset(struct pp2_ppio_desc *desc, u8  offset)
{
	desc->cmds[1] = (u32)offset;
}

static inline void pp2_ppio_outq_desc_set_pkt_len(struct pp2_ppio_desc *desc , u16 len)
{
	desc->cmds[1] = (desc->cmds[1] & ~TXD_BYTE_COUNT_MASK) | (len << 16 & TXD_BYTE_COUNT_MASK);
}

/******** RXQ  ********/

/* TODO: Timestamp, L4IChk */

static inline dma_addr_t pp2_ppio_inq_desc_get_phys_addr(struct pp2_ppio_desc *desc)
{
	return ((u64)((desc->cmds[5] & RXD_BUF_PHYS_HI_MASK) >> 0) << 32) |
		((u64)((desc->cmds[4] & RXD_BUF_PHYS_LO_MASK) >> 0));
}

static inline u64 pp2_ppio_inq_desc_get_cookie(struct pp2_ppio_desc *desc)
{
	return ((u64)((desc->cmds[7] & RXD_BUF_VIRT_HI_MASK) >> 0) << 32) |
		((u64)((desc->cmds[6] & RXD_BUF_VIRT_LO_MASK) >> 0));
}

struct pp2_ppio * pp2_ppio_inq_desc_get_pp_io(struct pp2_ppio_desc *desc); /*Note: under _DEBUG_*/

static inline u16 pp2_ppio_inq_desc_get_pkt_len(struct pp2_ppio_desc *desc)
{
	return ((desc->cmds[1] & RXD_BYTE_COUNT_MASK) >> 16);
}

void pp2_ppio_inq_desc_get_l3_info(struct pp2_ppio_desc *desc, enum pp2_inq_l3_type *type, u8 *offset);
void pp2_ppio_inq_desc_get_l4_info(struct pp2_ppio_desc *desc, enum pp2_inq_l4_type *type, u8 *offset);
int pp2_ppio_inq_desc_get_ip_isfrag(struct pp2_ppio_desc *desc);

struct pp2_bpool * pp2_ppio_inq_desc_get_bpool(struct pp2_ppio_desc *desc);

enum pp2_inq_desc_status pp2_ppio_inq_desc_get_pkt_error(struct pp2_ppio_desc *desc);

/**
 * Send a batch of frames (single dscriptor) on an OutQ of PP-IO.
 *
 * The routine assumes that the BM-Pool is either free by HW (by appropriate desc
 * setter) or by the MUSDK client SW.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		hif	TODO
 * @param[in]		qid	out-Q id on which to send the frames.
 * @param[in]		descs	A pointer to an array of descriptors represents the
 * 				frames to be sent.
 * @param[in/out]	num	Number of frames to be sent; output: number of frames sent.
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
 * Send a batch of S/G frames (single or multiple dscriptors) on an OutQ of PP-IO.
 *
 * The routine assumes that the BM-Pool is either free by HW (by appropriate desc
 * setter) or by the MUSDK client SW.
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		hif	TODO
 * @param[in]		qid	out-Q id on which to send the frames.
 * @param[in]		descs	A pointer to an array of S/G-descriptors represents the
 * 				frames to be sent.
 * @param[in/out]	num	Number of frames to be sent; output: number of frames sent.
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
 * TODO
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		hif	TODO
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
 * TODO
 *
 * @param[in]		ppio	A pointer to a PP-IO object.
 * @param[in]		tc	traffic class on which to recieve frames
 * @param[in]		qid	out-Q id on which to recieve the frames.
 * @param[in]		descs	A pointer to an array of descriptors represents the
 * 				recieved frames.
 * @param[in/out]	num	Number of frames to recieve (as max);
 * 				output: number of frames recieved.
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_ppio_recv(struct pp2_ppio	*ppio,
		  u8			 tc,
		  u8			 qid,
		  struct pp2_ppio_desc	*descs,
		  u16			*num);


/**************************************************************************//**
	Run-time Control API
*//***************************************************************************/

int pp2_ppio_enable(struct pp2_ppio *ppio);
int pp2_ppio_disable(struct pp2_ppio *ppio);

int pp2_ppio_set_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr);
int pp2_ppio_get_mac_addr(struct pp2_ppio *ppio, eth_addr_t addr);
int pp2_ppio_set_mtu(struct pp2_ppio *ppio, u16 mtu); /* For debug only, check mtu during  pkt_send() */
int pp2_ppio_get_mtu(struct pp2_ppio *ppio, u16 *mtu);
int pp2_ppio_set_mru(struct pp2_ppio *ppio, u16 len);
int pp2_ppio_get_mru(struct pp2_ppio *ppio, u16 *len);
int pp2_ppio_set_uc_promisc(struct pp2_ppio *ppio, int en);
int pp2_ppio_get_uc_promisc(struct pp2_ppio *ppio, int *en);
int pp2_ppio_set_mc_promisc(struct pp2_ppio *ppio, int en);
int pp2_ppio_get_mc_promisc(struct pp2_ppio *ppio, int *en);
int pp2_ppio_add_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr);
int pp2_ppio_remove_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr); /* Allows to remove the mac_address set  by pp2_ppio_set_mac_addr() */
int pp2_ppio_flush_mac_addrs(struct pp2_ppio *ppio, int uc, int mc); /* Does not flush the mac_address set  by pp2_ppio_set_mac_addr() */

int pp2_ppio_get_phys_in_q(struct pp2_ppio *ppio, u8 tc, u8 qid, u8 *pq);
int pp2_ppio_get_outq_state(struct pp2_ppio *ppio, u8 qid);

/*TODO: VLAN filters ????*/
/*TODO: link state ???*/
/*TODO: counters/statistics for port/Q*/


#endif /* __MV_PP2_PPIO_H__ */
