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

#ifndef __MV_PP2_H__
#define __MV_PP2_H__

#include "mv_std.h"
#include "mv_net.h"

#define PP2_NUM_PKT_PROC		4 /**< Maximum number of packet processors */
#define PP2_NUM_ETH_PPIO		3 /**< Maximum number of ppio instances in each packet processor */
#define PP2_MAX_PROTO_SUPPORTED		8 /**< Maximum number of net protocols supported in pp2 parser */
#define PP2_MAX_FIELDS_SUPPORTED	2 /**< Maximum number of net protocol special fields supported in pp2 parser*/

/** @addtogroup grp_pp2_init Packet Processor: Initialization
 *
 *  Packet Processor Initialization API documentation
 *
 *  @{
 */

struct pp2_proto_field {
	enum mv_net_proto		proto;
	union mv_net_proto_fields	field;
};


/**
 *ppio parser parameters definitions
 * this API is used in order to modify the pp2 parser parameters according to customer needs
 */
struct pp2_parse_params {
	/** The following API section allows configuration of a network protocol.in pp2 parser
	* The following network protocols are supported:
	* - MV_NET_PROTO_ETH,
	* - MV_NET_PROTO_VLAN,
	* - MV_NET_PROTO_PPPOE,
	* - MV_NET_PROTO_IP,
	* - MV_NET_PROTO_IP4,
	* - MV_NET_PROTO_IP6,
	* - MV_NET_PROTO_TCP,
	* - MV_NET_PROTO_UDP,
	*/

	/**< Indicates the existing supported protocols will be overrided in parser */
	/** override_protos = '0' - no changes (keep default)
	 *   override_protos  = '1' - override
	 */
	int			override_protos;
	/** Indicates how many protocols should be supported by the parser
	 * num_protos = '0' - no protocols supported
	 */
	u8			num_protos;
	enum mv_net_proto	protos[PP2_MAX_PROTO_SUPPORTED];

	/** The following API section allows configuration of special fields (1 or 2 bits) in a specified protocol.
	 * The following protocol special fields are supported:
	 * - IPv4 fragmentation
	 * - DSA tag mode
	 */

	/** Indicates the existing supported network protocol special fields will be overrided in parser
	 * value '0' - no changes (keep default)
	 * value '1' - override
	 */
	int			override_fields;
	/** Indicates how many network protocol special fields should be supported by the parser
	 * value '0' - no special fields supported
	 */
	u8			num_fields;
	struct pp2_proto_field	fields[PP2_MAX_FIELDS_SUPPORTED];
};

/**
 * pp2 init parameters
 *
 */
struct pp2_init_params {
	/** Bitmap of reserved HIF objects (0-8), that may not be used by MUSDK. bit0=hif0, etc. */
	u16			hif_reserved_map;
	/** Bitmap of reserved bm_pools (0-15). The pools are reserved in all packet_processors. */
	u16			bm_pool_reserved_map;
	/** Bitmap of RSS Tables (0-7). The tables are reserved in all packet_processors. */
	u8			rss_tbl_reserved_map;
	/** Bitmap of reserved policers (0-30). The policers are reserved in all packet_processors. */
	u32			policers_reserved_map;

	/* TODO FUTURE struct pp2_parse_params	prs_params; */
};

/**
 * Initialize the global PPv2x
 *
 * @param[in]	params	A pointer to structure that contains all relevant parameters.
 *
 * @retval	A pointer to a DMA memory on success
 * @retval	<0 on failure
 */
int pp2_init(struct pp2_init_params *params);

/**
 * TODO - Destroy the global PPv2x
 *
 */
void pp2_deinit(void);

/**
 * Get the pp_id and port_id parameters corresponding to the provided interface name
 * These parameters are needed for initializing the corresponding ppio
 *
 * @param[in]	ifname	A pointer to the interface name.
 * @param[out]	pp_id	A pointer to the corresponding packet prosessor id.
 * @param[out]	ppio_id	A pointer to the corresponding port id.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int pp2_netdev_get_ppio_info(char *ifname, u8 *pp_id, u8 *ppio_id);

/**
 * Get port availability for musdk
 *
 * @param[in]	pp_id	A pointer to the corresponding packet prosessor id.
 * @param[in]	ppio_id	A pointer to the corresponding port id.
 *
 * @retval	'1' - available
 * @retval	'0' - not available
 *
 */
int pp2_ppio_available(int pp_id, int ppio_id);


/**
 * Get number of active packet_processors.
 *
 *
 * @retval	>0 on success
 * @retval	=0 on failure
 */
u8 pp2_get_num_inst(void);

/** @} */ /* end of grp_pp2_init */

#endif /* __MV_PP2_H__ */
