/*******************************************************************************
	Copyright (C) 2016 Marvell International Ltd.

	If you received this File from Marvell, you may opt to use, redistribute
	and/or modify this File under the following licensing terms.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

		* Redistributions of source code must retain the above copyright notice,
		  this list of conditions and the following disclaimer.

		* Redistributions in binary form must reproduce the above copyright
		  notice, this list of conditions and the following disclaimer in the
		  documentation and/or other materials provided with the distribution.

		* Neither the name of Marvell nor the names of its contributors may be
		  used to endorse or promote products derived from this software without
		  specific prior written permission.

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
*******************************************************************************/

#ifndef __MV_PP2_PPIO_H__
#define __MV_PP2_PPIO_H__

#include "std.h"

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
};

enum pp2_ppio_outqs_sched_mode {
	PP2_PPIO_SCHED_M_NONE = 0,
};

struct pp2_ppio_inq_params {
	u32	size; /**< Q size – number of dewcriptors */
};

struct pp2_ppio_tc_params {
	int				 use_hash;
	u16				 pkt_offset;    /* Must be multiple of 32 bytes.*/
	u16				 num_in_qs;
	struct pp2_ppio_inq_params	*inqs_params;
	struct pp2_bpool		*pools[PP2_PPIO_TC_MAX_POOLS];
/* TODO: future:
	int				 qos;
*/
};

struct pp2_ppio_inqs_params {
	u16				 num_tcs;
	struct pp2_ppio_tc_params	 tcs_params[PP2_PPIO_MAX_NUM_TCS];
	/** hash engine may be seleceted only according to “parser-results”;
	 * therefore, we put hash selection on a per port basis. */
	enum pp2_ppio_hash_type		 hash_type[PP2_PPIO_MAX_NUM_HASH];
};

struct pp2_ppio_outq_params {
	u32	size;	/**< Q size – number of dewcriptors */
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
	/* Used for DTS acc to find appropriate “physical” PP-IO obj;
	 * E.g. “eth-0:0” means PPv2[0],port[0] */
	char				*match;

	enum pp2_ppio_type		 type; /* TODO: support only “NIC” type for short-term! */
	struct pp2_ppio_inqs_params	 inqs_params;
	struct pp2_ppio_outqs_params	 outqs_params;
/* TODO: do we need extra pools per port? 
	struct pp2_bpool		*pools[PP2_PPIO_TC_MAX_POOLS];
*/
};


int pp2_ppio_init(struct pp2_ppio_params *params, struct pp2_ppio **ppio);


/**************************************************************************//**
	Run-time API
*//***************************************************************************/

#define PP2_PPIO_DESC_NUM_WORDS	8
#define PP2_PPIO_DESC_NUM_FRAGS	16 /* TODO: check if there’s HW limitation */


struct pp2_ppio_desc {
	u32	cmds[PP2_PPIO_DESC_NUM_WORDS];
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

void pp2_ppio_outq_desc_reset (struct pp2_ppio_desc *desc);
void pp2_ppio_outq_desc_set_phys_addr(struct pp2_ppio_desc *desc, dma_addr_t addr);
void pp2_ppio_outq_desc_set_cookie(struct pp2_ppio_desc *desc, u64 cookie);

void pp2_ppio_outq_desc_set_pool(struct pp2_ppio_desc *desc, struct pp2_bpool *pool);

void pp2_ppio_outq_desc_set_proto_info(struct pp2_ppio_desc *desc,
				       enum pp2_outq_l3_type l3_type,
				       enum pp2_outq_l4_type l4_type,
				       u8  l3_offset,
				       u8 l4_offset,
				       int gen_l3_chk,
				       int gen_l4_chk
				      );
void pp2_ppio_outq_desc_set_dsa_tag(struct pp2_ppio_desc *desc);

void pp2_ppio_outq_desc_set_pkt_offset(struct pp2_ppio_desc *desc, u8  offset);
void pp2_ppio_outq_desc_set_pkt_len(struct pp2_ppio_desc *desc , u16 len);

/******** RXQ  ********/

/* TODO: Timestamp, L4IChk */

dma_addr_t pp2_ppio_inq_desc_get_phys_addr(struct pp2_ppio_desc *desc);
u64 pp2_ppio_inq_desc_get_cookie(struct pp2_ppio_desc *desc);

struct pp2_ppio * pp2_ppio_inq_desc_get_pp_io(struct pp2_ppio_desc *desc); /*Note: under _DEBUG_*/

u16 pp2_ppio_inq_desc_get_pkt_len(struct pp2_ppio_desc *desc);
void pp2_ppio_inq_desc_get_l3_info(struct pp2_ppio_desc *desc, enum pp2_inq_l3_type *type, u8 *offset);
void pp2_ppio_inq_desc_get_l4_info(struct pp2_ppio_desc *desc, enum pp2_inq_l4_type *type, u8 *offset);
int pp2_ppio_inq_desc_get_ip_isfrag(struct pp2_ppio_desc *desc);

struct pp2_bpool * pp2_ppio_inq_desc_get_bpool(struct pp2_ppio_desc *desc);

enum pp2_inq_desc_status pp2_ppio_inq_desc_get_pkt_error(struct pp2_ppio_desc *desc);

/* pp2_ppio_send
It is assumed that the BM-Pool is either free by HW (by appropriate desc setter) or by the MUSDK client SW.
*/
int pp2_ppio_send(struct pp2_ppio	*ppio,
		  struct pp2_hif	*hif,
		  int			 qid,
		  struct pp2_ppio_desc	*desc);
int pp2_ppio_send_sg(struct pp2_ppio		*ppio,
		     struct pp2_hif		*hif,
		     int			 qid,
		     int			 num_frags,
		     struct pp2_ppio_desc	**desc);
int pp2_ppio_get_num_outq_done(struct pp2_ppio	*ppio,
			       struct pp2_hif	*hif,
			       int		 qid,
			       int		*num);
int pp2_ppio_recv(struct pp2_ppio *ppio, int tc, int qid, struct pp2_ppio_desc *desc);


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

int pp2_ppio_get_phys_in_q(struct pp2_ppio *ppio, int tc, int qid, int *pq);
int pp2_ppio_get_outq_state(struct pp2_ppio *ppio, int qid);

/*TODO: VLAN filters ????*/
/*TODO: link state ???*/
/*TODO: counters/statistics for port/Q*/


#endif /* __MV_PP2_PPIO_H__ */
