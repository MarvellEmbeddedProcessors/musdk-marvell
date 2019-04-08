/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef __MV_PP2_H__
#define __MV_PP2_H__

#include "mv_std.h"
#include "mv_net.h"

#define PP2_NUM_PKT_PROC		4 /**< Maximum number of packet processors */
#define PP2_NUM_ETH_PPIO		3 /**< Maximum number of ppio instances in each packet processor */
#define PP2_MAX_PROTO_SUPPORTED		8 /**< Maximum number of net protocols supported in pp2 parser */
#define PP2_MAX_FIELDS_SUPPORTED	2 /**< Maximum number of net protocol special fields supported in pp2 parser*/
#define PP2_MAX_UDFS_SUPPORTED		3 /**< Maximum number of udf (user-define-field) options */

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
 * pp2 parser udf parameters
 * contains parser udf data
 */
struct pp2_parse_udf_params {
	enum mv_net_proto		match_proto;
	union mv_net_proto_fields	match_field;
	u8				*match_key;
	u8				*match_mask;
	 /**
	  * the offset of the udf field relative to the location of
	  * the 'match_field' (incl.)
	  */
	u8				offset;
};

/**
 * pp2 parser udfs structure
 * this API is used in order to add udf parsing to the pp2 parser
 */
struct pp2_parse_udfs {
	u8				num_udfs;
	struct pp2_parse_udf_params	udfs[PP2_MAX_UDFS_SUPPORTED];
};

/**
 * pp2 init parameters
 *
 */

/* reserved_maps autodetect bitflags */
#define	PP2_RSRVD_MAP_HIF_AUTO		0x1
#define	PP2_RSRVD_MAP_BM_POOL_AUTO	0x2
#define	PP2_RSRVD_MAP_RSS_AUTO		0x4
#define	PP2_RSRVD_MAP_POLICER_AUTO	0x8

struct pp2_init_params {
	/** Bitmap of reserved HIF objects (0-8), that may not be used by MUSDK. bit0=hif0, etc. */
	u16			hif_reserved_map;
	/** Bitmap of reserved bm_pools (0-15). The pools are reserved in all packet_processors. */
	u16			bm_pool_reserved_map;
	/** Bitmap of RSS Tables (0-7). The tables are reserved in all packet_processors. */
	u8			rss_tbl_reserved_map;
	/** Bitmap of reserved policers (0-30). The policers are reserved in all packet_processors. */
	u32			policers_reserved_map;
	/** Bitmap of reserved early-drops (0-14). The reservation is applied to all packet_processors. */
	u32			early_drop_reserved_map;
	/** flag indicating to skip hw initializations (useful for guest mode) */
	int			skip_hw_init;
	/** Bitmap of the reserved_maps that should be autodetected. */
	u32			res_maps_auto_detect_map;
	/** user defined parser fields */
	struct pp2_parse_udfs	prs_udfs;
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
 * Get the interface name corresponding to the provided pp_id and port_id parameters
 *
 * @param[in]	pp_id	packet prosessor id.
 * @param[in]	ppio_id	port id.
 * @param[out]	ifname	A pointer to the interface name.
 *
 * @retval	0 on success
 * @retval	<0 on failure
 */
int pp2_netdev_get_ifname(u8 pp_id, u8 ppio_id, char *ifname);

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

/**
 * Get bitmap of hifs used by kernel.
 *
 *
 * @retval	>0 on success
 * @retval	=0 on failure
 */
u16 pp2_get_used_hif_map(void);

/**
 * Get bitmap of used bm_pools.
 *
 *
 * @retval	>0 on success
 * @retval	=0 on failure
 */
u16 pp2_get_used_bm_pool_map(void);

/**
 * return if pp2 sysfs is available
 *
 * @retval	'1' - available
 * @retval	'0' - not available
 */
int pp2_is_sysfs_avail(void);
/** @} */ /* end of grp_pp2_init */

#endif /* __MV_PP2_H__ */
