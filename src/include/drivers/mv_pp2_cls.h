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

#ifndef __MV_PP2_CLS_H__
#define __MV_PP2_CLS_H__

#include "mv_std.h"
#include "mv_pp2.h"

/** @addtogroup grp_pp2_cls Packet Processor: Classifier
 *
 *  Packet Processor Classifier API documentation
 *
 *  @{
 */

struct pp2_cls_tbl;
struct pp2_cls_plcr;
struct pp2_cls_early_drop;

#define PP2_CLS_TBL_MAX_NUM_FIELDS	5
#define PP2_CLS_PLCR_NUM		31
#define PP2_CLS_EARLY_DROP_NUM		15

enum pp2_cls_tbl_type {
	PP2_CLS_TBL_EXACT_MATCH = 0,
	PP2_CLS_TBL_MASKABLE
};

/**
 * TODO
 */
enum pp2_cls_qos_tbl_type {
	PP2_CLS_QOS_TBL_NONE = 0,	/**< No QoS support */
	/** QoS according to VLAN-priority (outer tag) if exists; otherwise, use default */
	PP2_CLS_QOS_TBL_VLAN_PRI,
	/** QoS according to IP-priority (i.e. DSCP) if exists;
	 * otherwise, use default
	 */
	PP2_CLS_QOS_TBL_IP_PRI,
	/** QoS according to VLAN-priority (outer tag) if exists; otherwise, use IP-priority (i.e. DSCP) if exists;
	 * otherwise, use default
	 */
	PP2_CLS_QOS_TBL_VLAN_IP_PRI,
	/** QoS according to IP-priority (i.e. DSCP) if exists; otherwise, use VLAN-priority (outer tag) if exists;
	 * otherwise, use default
	 */
	PP2_CLS_QOS_TBL_IP_VLAN_PRI,
	PP2_CLS_QOS_TBL_OUT_OF_RANGE	/**< Invalid QoS type */
};

/**
 * TODO: Future
 * enum pp2_cls_tbl_statistics_mode {
 *	PP2_CLS_TBL_STATS_M_NONE = 0,
 *	PP2_CLS_TBL_STATS_M_FRM,
 *	PP2_CLS_TBL_STATS_M_BYTES
 * };
 */
enum pp2_cls_tbl_action_type {
	PP2_CLS_TBL_ACT_DROP = 0,
	PP2_CLS_TBL_ACT_DONE,
	/* TODO: PP2_CLS_TBL_ACT_LU */
};

enum pp2_cls_plcr_token_unit {
	PP2_CLS_PLCR_BYTES_TOKEN_UNIT = 0,/* token unit is based on number of bytes	*/
	PP2_CLS_PLCR_PACKETS_TOKEN_UNIT	/* token unit is based on number of packets	*/
};

enum pp2_cls_plcr_color_mode {
	PP2_CLS_PLCR_COLOR_BLIND_MODE = 0,/* color blind mode	*/
	PP2_CLS_PLCR_COLOR_AWARE_MODE	/* color aware mode	*/
};

struct pp2_cls_plcr_params {
	/** Used for DTS access to find appropriate Policer obj;
	 * E.g. "policer-0:0" means PPv2[0],policer[0]
	 */
	const char		*match;

	enum pp2_cls_plcr_token_unit token_unit;	/* token in unit of bytes or packets	*/
	enum pp2_cls_plcr_color_mode color_mode;	/* color mode, blind or aware of former color	*/
	u32	cir;		/** commit information rate in unit of Kbps (data rate) or pps.
				 *  minimum value - 104Kbps or 125pps. value of '0' means maximum value.
				 *  In Byte mode, the final value is a multiple of 8.
				 */
	u32	cbs;		/** commit burst size in unit of KB or number of packets;
				 *  minimum value - 64KB or 1Kpps. value of '0' means maximum value.
				 */
	u32	ebs;		/** excess burst size in unit of KB or number of packets
				 *  minimum value - 64KB or 1Kpps. value of '0' means maximum value.
				 */
};

struct pp2_cls_early_drop_params {
	/** Used for DTS access to find appropriate early-drop obj;
	 * E.g. "ed-0:0" means PPv2[0],early-drop[0]
	 */
	const char	*match;

	u16		threshold; /** TODO */
};

struct pp2_cls_cos_desc {
	struct pp2_ppio	*ppio;
	u8		 tc;
};

struct pp2_cls_tbl_action {
	enum pp2_cls_tbl_action_type		 type;
	/** Valid only in case of 'MASKABLE' table.
	 * This value will be reflected on the inQ descriptor in case of hit.
	 * Allowed values: 0-4095. Value of '0' means 'not in use'.
	 */
	u16					 flow_id;
	/* TODO: struct pp2_cls_tbl		*next_tbl; */
	/** 'NULL' value means no-cos change; i.e. keep original cos */
	struct pp2_cls_cos_desc			*cos;
	/** 'NULL' value means no-plcr change; i.e. keep original plcr */
	struct pp2_cls_plcr			*plcr;
};

struct pp2_cls_tbl_key {
	u8				key_size;
	u8				num_fields;
	struct pp2_proto_field		proto_field[PP2_CLS_TBL_MAX_NUM_FIELDS];
};

struct pp2_cls_tbl_params {
	enum pp2_cls_tbl_type			 type;
	u16					 max_num_rules;
	struct pp2_cls_tbl_key			 key;
	/* TODO: enum pp2_cls_tbl_statistics_mode	 stats_mode; */
	/* TODO: enum pp2_cls_tbl_aging_mode	 aging_mode; */
	/* TODO: enum pp2_cls_tbl_priority_mode	 prio_mode; */
	struct pp2_cls_tbl_action		 default_act;
};

/**
 * TODO
 */
struct pp2_cls_qos_tbl_params {
	enum pp2_cls_qos_tbl_type		 type;
	struct pp2_cls_cos_desc			 pcp_cos_map[MV_VLAN_PRIO_NUM];
	struct pp2_cls_cos_desc			 dscp_cos_map[MV_DSCP_NUM];
};

/**
 * Create a classifier table object
 *
 * @param[in]	params	A pointer to the classifier table parameters
 * @param[out]	tbl	A pointer to an allocated classifier table
 *
 * @retval		0 on success
 * @retval		error-code otherwise
 */
int pp2_cls_tbl_init(struct pp2_cls_tbl_params *params, struct pp2_cls_tbl **tbl);

/**
 * Deinit a classifier table object
 *
 * @param[in]	tbl		A pointer to a classifier table object
 */
void pp2_cls_tbl_deinit(struct pp2_cls_tbl *tbl);

/**
 * Create a QoS table object
 *
 * @param[in]	params	A pointer to the classifier table parameters
 * @param[out]	tbl	A pointer to an allocated classifier table
 *
 * @retval		0 on success
 * @retval		error-code otherwise
 */
int pp2_cls_qos_tbl_init(struct pp2_cls_qos_tbl_params *params, struct pp2_cls_tbl **tbl);

/**
 * Deinit a classifier table object
 *
 * @param[in]	tbl		A pointer to a classifier table object
 */
void pp2_cls_qos_tbl_deinit(struct pp2_cls_tbl *tbl);

struct pp2_cls_rule_key_field {
	u8	size;
	u8	*key;
	u8	*mask;
};

/*TODO : Add union to support future API's */

struct pp2_cls_tbl_rule {
	u8	num_fields;
	struct pp2_cls_rule_key_field fields[PP2_CLS_TBL_MAX_NUM_FIELDS];
};

/**
 * Add a classifier rule
 *
 * @param[in]	tbl		A pointer to a classifier table object
 * @param[in]	rule		A pointer to a classifier rule
 * @param[in]	action		A pointer to a classifier action
 *
 * @retval	0 on success
 * @retval	positive value - reserved
 * @retval	error-code otherwise (negative value)
 */
int pp2_cls_tbl_add_rule(struct pp2_cls_tbl		*tbl,
			 struct pp2_cls_tbl_rule	*rule,
			 struct pp2_cls_tbl_action	*action);

/**
 * Modify the action of an existing classifier rule
 *
 * The rule must be already in place
 *
 * @param[in]	tbl		A pointer to a classifier table object
 * @param[in]	rule		A pointer to a classifier rule
 * @param[in]	action		A pointer to a classifier action
 *
 * @retval	0 on success
 * @retval	error-code otherwise
 */
int pp2_cls_tbl_modify_rule(struct pp2_cls_tbl		*tbl,
			    struct pp2_cls_tbl_rule	*rule,
			    struct pp2_cls_tbl_action	*action);

/**
 * Remove a classifier rule
 *
 * @param[in]	tbl		A pointer to a classifier table object
 * @param[in]	rule		A pointer to a classifier rule
 *
 * @retval		0 on success
 * @retval		error-code otherwise
 */
int pp2_cls_tbl_remove_rule(struct pp2_cls_tbl		*tbl,
			    struct pp2_cls_tbl_rule	*rule);

/**
 * Create a classifier policer object
 *
 * @param[in]	params	A pointer to the classifier policer parameters
 * @param[out]	tbl	A pointer to an allocated classifier policer
 *
 * @retval		0 on success
 * @retval		error-code otherwise
 */
int pp2_cls_plcr_init(struct pp2_cls_plcr_params *params, struct pp2_cls_plcr **plcr);

/**
 * Deinit a classifier policer object
 *
 * @param[in]	plcr	A pointer to a classifier policer object
 */
void pp2_cls_plcr_deinit(struct pp2_cls_plcr *plcr);

/**
 * Create a classifier early-drop object
 *
 * @param[in]	params	A pointer to the classifier early-drop parameters
 * @param[out]	ed	A pointer to an allocated classifier early-drop
 *
 * @retval		0 on success
 * @retval		error-code otherwise
 */
int pp2_cls_early_drop_init(struct pp2_cls_early_drop_params *params, struct pp2_cls_early_drop **ed);

/**
 * Deinit a classifier early_drop object
 *
 * @param[in]	ed	A pointer to a classifier early-drop object
 */
void pp2_cls_early_drop_deinit(struct pp2_cls_early_drop *ed);

/** @} */ /* end of grp_pp2_cls */

#endif /* __MV_PP2_CLS_H__ */
