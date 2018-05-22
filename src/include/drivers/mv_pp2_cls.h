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

#define PP2_CLS_TBL_MAX_NUM_FIELDS	5
#define PP2_CLS_PLCR_NUM		31

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
 *
 * @retval		0 on success
 */
void pp2_cls_plcr_deinit(struct pp2_cls_plcr *plcr);

/** @} */ /* end of grp_pp2_cls */

#endif /* __MV_PP2_CLS_H__ */
