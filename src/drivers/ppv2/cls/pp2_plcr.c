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


/***********************/
/* c file declarations */
/***********************/
#include "std_internal.h"

#include "../pp2_types.h"
#include "../pp2.h"
#include "pp2_hw_cls.h"
#include "pp2_hw_cls_dbg.h"
#include "pp2_plcr.h"

/******************************************************************************
 * Type Definition
 ******************************************************************************/
/* minimum rate/burst of '0'means preceding maximum value */

struct pp2_cls_plcr_token_type_t g_pp2_pkt_token_type[] = {
	/*          token type			rate res      min rate	max rate   burst res  min burst max burst */
	{MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B,	   125,		125,	127875,		1,	1024,	65535},
	{MVPP2_PLCR_TOKEN_RATE_TYPE_10KBPS_8B,	   1250,	0,	1278750,	8,	0,	524280},
	{MVPP2_PLCR_TOKEN_RATE_TYPE_100KBPS_64B,   12500,	0,	12787500,	64,	0,	4194240},
	{MVPP2_PLCR_TOKEN_RATE_TYPE_1MBPS_512B,    125000,	0,	127875000,	512,	0,	33553920},
	{MVPP2_PLCR_TOKEN_RATE_TYPE_10MBPS_4KB,    1250000,	0,	1278750000,	4096,	0,	268431360},
};

/* The table assumes that the API is in Kbps and KB*/
struct pp2_cls_plcr_token_type_t g_pp2_byte_token_type[] = {
	/*          token type			rate res      min rate	max rate   burst res  min burst	max burst */
	{MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B,	   1,		13,	1023,		1,	64,	64},
	{MVPP2_PLCR_TOKEN_RATE_TYPE_10KBPS_8B,	   10,		0,	10230,		8,	0,	512},
	{MVPP2_PLCR_TOKEN_RATE_TYPE_100KBPS_64B,   100,		0,	102300,		64,	0,	4096},
	{MVPP2_PLCR_TOKEN_RATE_TYPE_1MBPS_512B,	   1000,	0,	1023000,	512,	0,	32768},
	{MVPP2_PLCR_TOKEN_RATE_TYPE_10MBPS_4KB,	   10000,	0,	10230000,	4096,	0,	262144},
};


/******************************************************************************
 * Function Definition
 ******************************************************************************/

static int pp2_cls_plcr_calc_token_type(const struct pp2_cls_plcr_params *policer_entry,
					enum pp2_cls_plcr_token_update_type_t *token_type,
					u64 *token_value)
{
	struct pp2_cls_plcr_token_type_t *token_arr;
	struct pp2_cls_plcr_token_type_t *token_entry;
	u64 cir, token_update_period[] = {10000, 1000, 100, 10, 1};

	/*
	 *	The formula to calculate obtained bandwidth is as follows:
	 *	Parameters:
	 *	BaseClockFrequency = 333MHz. ==> 'BaseClockCycles' ~= 3ns
	 *	'TokenUpdateBasePeriod' = 800*BaseClockCycles (register 0x1304, is 800 for this 'BaseClockFrequency')
	 *	'TokenUpdatePeriod'   = UpdateFactor*'TokenUpdateBasePeriod'
	 *	'TokenUpdateAddition' = 3*'TokenUpdateValue' tokens.
	 *	'UpdateFactor': defined by 'TokenUpdateType' (register 0x131c, bits 12-14)
	 *	CIR = (TokenUpdateAddition) / (TokenUpdatePeriod)
	 *		= 3*TokenUpdateValue / (UpdateFactor*TokenUpdateBasePeriod)
	 *		= 3*TokenUpdateValue / (UpdateFactor*800*3*E-09)
	 *		=   TokenUpdateValue / (UpdateFactor*800*E-09)
	 *	TokenUpdateValue = CIR * (UpdateFactor*800*E-09)
	 *			 = CIRKbps * (UpdateFactor*800*E-06)
	 *	UpdateFactor Example: For TokenUpdateType=2, UpdateFactor=100
	 */

	if (policer_entry->token_unit == PP2_CLS_PLCR_PACKETS_TOKEN_UNIT)
		token_arr = g_pp2_pkt_token_type;
	else
		token_arr = g_pp2_byte_token_type;

	cir = policer_entry->cir;
	if (cir == MVPP2_PLCR_CIR_NO_LIMIT) {
		*token_type = MVPP2_PLCR_TOKEN_RATE_TYPE_10MBPS_4KB;
		*token_value = MVPP2_PLCR_MAX_TOKEN_VALUE;
		return 0;
	}

	*token_type = MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B;
	do {
		token_entry = &token_arr[*token_type];
		if (cir % token_entry->rate_resl)
			cir = roundup(cir, token_entry->rate_resl);
		*token_value = cir * MVPP2_TOKEN_PERIOD_800_CORE_CLOCK * token_update_period[*token_type];
		*token_value /= 1000000;
		if (policer_entry->token_unit == PP2_CLS_PLCR_PACKETS_TOKEN_UNIT)
			*token_value /= 1000;
	} while ((*token_value > MVPP2_PLCR_MAX_TOKEN_VALUE) &&
		 (++(*token_type) <= MVPP2_PLCR_TOKEN_RATE_TYPE_10MBPS_4KB));

	if (*token_type > MVPP2_PLCR_TOKEN_RATE_TYPE_10MBPS_4KB) {
		pr_err("invalid state\n");
		return -EINVAL;
	}

	return 0;
}

/*******************************************************************************
* pp2_cls_plcr_hw_entry_add()
*
* DESCRIPTION: This API adds a policer entry to hardware.
*
* INPUTS:
*	policer_id    - policer ID.
*	policer_entry - policer entry configuration.
*
* OUTPUTS:
*	None.
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
*******************************************************************************/
static int pp2_cls_plcr_hw_entry_add(struct pp2_inst				*inst,
				     u8						policer_id,
				     struct pp2_cls_plcr_params			*policer_entry,
				     enum pp2_cls_plcr_token_update_type_t	token_type,
				     u64					token_value)
{
	struct pp2_cls_plcr_token_type_t *token_arr;
	struct pp2_cls_plcr_token_type_t *token_entry;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	int rc = 0;

	if (mv_pp2x_ptr_validate(policer_entry))
		return -EFAULT;

	if (mv_pp2x_range_validate(policer_id, 0, MVPP2_PLCR_MAX - 1)) {
		pr_err("invalid policer ID %d, out of range[%d, %d]\n",	policer_id, 0, MVPP2_PLCR_MAX - 1);
		return -EINVAL;
	}

	/* convert CIR to token_type and token_value */
	if (policer_entry->token_unit == PP2_CLS_PLCR_PACKETS_TOKEN_UNIT)
		token_arr = g_pp2_pkt_token_type;
	else
		token_arr = g_pp2_byte_token_type;

	token_entry = &token_arr[token_type];

	/* set token unit and type */
	rc = mv_pp2x_plcr_hw_token_config(cpu_slot, policer_id, policer_entry->token_unit, token_type);
	if (rc) {
		pr_err("failed to set token unit and type to HW\n");
		return rc;
	}

	/* set token value */
	rc = mv_pp2x_plcr_hw_token_value(cpu_slot, policer_id, token_value);
	if (rc) {
		pr_err("failed to set token value to HW\n");
		return rc;
	}

	/* set color mode */
	rc = mv_pp2x_plcr_hw_color_mode_set(cpu_slot, policer_id, policer_entry->color_mode);
	if (rc) {
		pr_err("failed to set color mode to HW\n");
		return rc;
	}

	/* set CBS and EBS */
	rc = mv_pp2x_plcr_hw_bucket_size_set(cpu_slot,
					     policer_id,
					     policer_entry->cbs / token_entry->burst_size_resl,
					     policer_entry->ebs / token_entry->burst_size_resl);
	if (rc) {
		pr_err("ailed to set CBS and EBS to HW\n");
		return rc;
	}

	/* enable this policer */
	rc = mv_pp2x_plcr_hw_enable(cpu_slot, policer_id, MVPP2_PLCR_ENTRY_VALID_STATE);
	if (rc) {
		pr_err("failed to enable policer to HW\\n");
		return rc;
	}

	return rc;
}

/*******************************************************************************
* pp2_cls_plcr_hw_entry_del()
*
* DESCRIPTION: This API deletes a policer entry from HW.
*
* INPUTS:
*	policer_id  - policer ID.
*
* OUTPUTS:
*	None.
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
*******************************************************************************/
static int pp2_cls_plcr_hw_entry_del(struct pp2_inst *inst, u8 policer_id)
{
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	int rc = 0;

	if (mv_pp2x_range_validate(policer_id, 0, MVPP2_PLCR_MAX - 1)) {
		pr_err("invalid policer ID %d, out of range[%d, %d]\n",	policer_id, 0, MVPP2_PLCR_MAX - 1);
		return -EINVAL;
	}

	/* disable this policer */
	rc = mv_pp2x_plcr_hw_enable(cpu_slot, policer_id, MVPP2_PLCR_ENTRY_INVALID_STATE);
	if (rc) {
		pr_err("failed to disable policer to HW\\n");
		return rc;
	}

	return rc;
}

/*******************************************************************************
* pp2_cls_plcr_entry_convert()
*
* DESCRIPTION: This API convert policer entry, especially CIR.
*
* INPUTS:
*	input_entry  - inputted policer entry configuration.
*
* OUTPUTS:
*	output_entry - inputted policer entry configuration.
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
*******************************************************************************/
static int pp2_cls_plcr_entry_convert(struct pp2_cls_plcr_params		*input_entry,
				      struct pp2_cls_plcr_params		*output_entry,
				      enum pp2_cls_plcr_token_update_type_t	token_type)
{
	struct pp2_cls_plcr_token_type_t *token_arr;
	struct pp2_cls_plcr_token_type_t *token_entry;
	int rc = 0;

	if (mv_pp2x_ptr_validate(input_entry))
		return -EFAULT;

	if (mv_pp2x_ptr_validate(output_entry))
		return -EFAULT;

	/* copy the entry */
	memcpy(output_entry, input_entry, sizeof(struct pp2_cls_plcr_params));

	if (output_entry->token_unit == PP2_CLS_PLCR_PACKETS_TOKEN_UNIT)
		token_arr = g_pp2_pkt_token_type;
	else
		token_arr = g_pp2_byte_token_type;

	/* convert the CIR if it is not multiple time of the resolution */
	token_entry = &token_arr[token_type];

	if (output_entry->cir == MVPP2_PLCR_CIR_NO_LIMIT)
		output_entry->cir = token_entry->max_rate;
	if (output_entry->cir % token_entry->rate_resl) {
		pr_warn("CIR(%d) is not multiple times of resolution(%d), will be adjusted to (%d)\n",
			    output_entry->cir,
			    token_entry->rate_resl,
			    roundup(output_entry->cir, token_entry->rate_resl));
		output_entry->cir = roundup(output_entry->cir, token_entry->rate_resl);
	}

	/* convert the CBS if it is not multiple time of the resolution */
	if (output_entry->cbs == MVPP2_PLCR_BURST_SIZE_NO_LIMIT)
		output_entry->cbs = token_entry->max_burst_size;
	if (output_entry->cbs % token_entry->burst_size_resl) {
		pr_warn("CBS(%d) is not multiple times of resolution(%d), will be adjusted to (%d)\n",
			    output_entry->cbs,
			    token_entry->burst_size_resl,
			    roundup(output_entry->cbs, token_entry->burst_size_resl));
		output_entry->cbs = roundup(output_entry->cbs, token_entry->burst_size_resl);
	}

	/* convert the EBS if it is not multiple time of the resolution */
	if (output_entry->ebs == MVPP2_PLCR_BURST_SIZE_NO_LIMIT)
		output_entry->ebs = token_entry->max_burst_size;
	if (output_entry->ebs % token_entry->burst_size_resl) {
		pr_warn("EBS(%d) is not multiple times of resolution(%d), will be adjusted to (%d)\n",
			    output_entry->ebs,
			    token_entry->burst_size_resl,
			    roundup(output_entry->ebs, token_entry->burst_size_resl));
		output_entry->ebs = roundup(output_entry->ebs, token_entry->burst_size_resl);
	}

	return rc;
}

/*******************************************************************************
* pp2_cls_plcr_entry_check()
*
* DESCRIPTION: This API verifies a policer entry.
*
* INPUTS:
*	policer_entry - policer entry configuration.
*
* OUTPUTS:
*	None.
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
*******************************************************************************/
static int pp2_cls_plcr_entry_check(struct pp2_cls_plcr_params *policer_entry,
				    enum pp2_cls_plcr_token_update_type_t *token_type,
				    u64 *token_value)
{
	struct pp2_cls_plcr_token_type_t *token_arr;
	struct pp2_cls_plcr_token_type_t *token_entry;

	if (mv_pp2x_ptr_validate(policer_entry))
		return -EFAULT;

	if ((policer_entry->token_unit != PP2_CLS_PLCR_BYTES_TOKEN_UNIT) &&
	    (policer_entry->token_unit != PP2_CLS_PLCR_PACKETS_TOKEN_UNIT)) {
		pr_err("invalid token unit selection\n");
		return -EINVAL;
	}

	if ((policer_entry->color_mode != PP2_CLS_PLCR_COLOR_BLIND_MODE) &&
	    (policer_entry->color_mode != PP2_CLS_PLCR_COLOR_AWARE_MODE)) {
		pr_err("invalid color mode selection\n");
		return -EINVAL;
	}

	/* check CIR, CBS/EBS range */
	if (policer_entry->token_unit == PP2_CLS_PLCR_PACKETS_TOKEN_UNIT) {
		if ((policer_entry->cir != MVPP2_PLCR_CIR_NO_LIMIT) &&
		    ((policer_entry->cir < g_pp2_pkt_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B].min_rate) ||
		     (policer_entry->cir > g_pp2_pkt_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_10MBPS_4KB].max_rate))) {
			pr_err("invalid CIR value %d, out of range[%d, %d]\n",
				policer_entry->cir,
				g_pp2_pkt_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B].min_rate,
				g_pp2_pkt_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_10MBPS_4KB].max_rate);
			return -EINVAL;
		}

		if ((policer_entry->cbs != MVPP2_PLCR_BURST_SIZE_NO_LIMIT) &&
		    (policer_entry->cbs < g_pp2_pkt_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B].min_burst_size)) {
			pr_err("invalid cbs value %d, must be at least %d\n", policer_entry->cbs,
				g_pp2_pkt_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B].min_burst_size);
			return -EINVAL;
		}

		if ((policer_entry->ebs != MVPP2_PLCR_BURST_SIZE_NO_LIMIT) &&
		    (policer_entry->ebs < g_pp2_pkt_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B].min_burst_size)) {
			pr_err("invalid ebs value %d, must be at least %d\n", policer_entry->ebs,
				g_pp2_pkt_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B].min_burst_size);
			return -EINVAL;
		}
	} else {
		policer_entry->cir = roundup(policer_entry->cir, 8) / 8; /* need to adjust the value to Bytes */
		if ((policer_entry->cir != MVPP2_PLCR_CIR_NO_LIMIT) &&
		    ((policer_entry->cir < g_pp2_byte_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B].min_rate) ||
		     (policer_entry->cir > g_pp2_byte_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_10MBPS_4KB].max_rate))) {
			pr_err("invalid CIR value %d, out of range[%d, %d]\n",
				policer_entry->cir * 8,
				g_pp2_byte_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B].min_rate * 8,
				g_pp2_byte_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_10MBPS_4KB].max_rate * 8);
			return -EINVAL;
		}

		if ((policer_entry->cbs != MVPP2_PLCR_BURST_SIZE_NO_LIMIT) &&
		    (policer_entry->cbs < g_pp2_byte_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B].min_burst_size)) {
			pr_err("invalid cbs value %d, must be at least %d\n", policer_entry->cbs,
				g_pp2_byte_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B].min_burst_size);
			return -EINVAL;
		}

		if ((policer_entry->ebs != MVPP2_PLCR_BURST_SIZE_NO_LIMIT) &&
		    (policer_entry->ebs < g_pp2_byte_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B].min_burst_size)) {
			pr_err("invalid ebs value %d, must be at least %d\n", policer_entry->ebs,
				g_pp2_byte_token_type[MVPP2_PLCR_TOKEN_RATE_TYPE_1KBPS_1B].min_burst_size);
			return -EINVAL;
		}
	}

	pp2_cls_plcr_calc_token_type(policer_entry, token_type, token_value);
	/* Check that CBS and EBS values compatible with CIR */
	if (policer_entry->token_unit == PP2_CLS_PLCR_PACKETS_TOKEN_UNIT)
		token_arr = g_pp2_pkt_token_type;
	else
		token_arr = g_pp2_byte_token_type;
	token_entry = &token_arr[*token_type];

	if ((policer_entry->cbs > token_entry->max_burst_size) ||
	    (policer_entry->ebs > token_entry->max_burst_size)) {
		pr_err("cbs(%d) and/or ebs(%d) must be at most %d\n", policer_entry->cbs, policer_entry->ebs,
			token_entry->max_burst_size);
		return -EINVAL;
	}

	return 0;
}

/*******************************************************************************
* pp2_cls_plcr_entry_add()
*
* DESCRIPTION: This API adds a policer entry.
*
* INPUTS:
*	policer_entry - policer entry configuration.
*
* OUTPUTS:
*	policer_id    - policer ID.
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
*******************************************************************************/
int pp2_cls_plcr_entry_add(struct pp2_inst			*inst,
			struct pp2_cls_plcr_params	*policer_entry,
			u8				policer_id)
{
	struct pp2_cls_db_plcr_entry_t l_plcr_entry;
	enum pp2_cls_plcr_token_update_type_t token_type;
	u64 token_value;
	int rc = 0;

	if (mv_pp2x_ptr_validate(policer_entry))
		return -EFAULT;

	/* validate the input policer entry */
	rc = pp2_cls_plcr_entry_check(policer_entry, &token_type, &token_value);
	if (rc) {
		pr_err("failed to check policer entry\n");
		return rc;
	}

	/* Convert CIR and other parameters */
	MVPP2_MEMSET_ZERO(l_plcr_entry);
	rc = pp2_cls_plcr_entry_convert(policer_entry, &l_plcr_entry.plcr_entry, token_type);
	if (rc) {
		pr_err("failed to convert policer entry\n");
		return rc;
	}

	/* Add policer entry to HW */
	rc = pp2_cls_plcr_hw_entry_add(inst, policer_id, &l_plcr_entry.plcr_entry, token_type, token_value);
	if (rc) {
		pr_err("failed to add policer entry to HW\n");
		return rc;
	}

	/* Add policer entry to DB */
	l_plcr_entry.valid = MVPP2_PLCR_ENTRY_VALID_STATE;
	rc = pp2_cls_db_plcr_entry_set(inst, policer_id, &l_plcr_entry);
	if (rc) {
		pr_err("failed to save policer entry to DB\n");
		return rc;
	}

	return rc;
}

/*******************************************************************************
* pp2_cls_plcr_entry_del()
*
* DESCRIPTION: This API deletes a policer entry.
*
* INPUTS:
*	policer_id    - policer ID.
*
* OUTPUTS:
*	None.
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
*******************************************************************************/
int pp2_cls_plcr_entry_del(struct pp2_inst *inst, u8 policer_id)
{
	struct pp2_cls_db_plcr_entry_t l_plcr_entry;
	int rc = 0;

	if (mv_pp2x_range_validate(policer_id, MVPP2_PLCR_MIN_ENTRY_ID, MVPP2_PLCR_MAX - 1)) {
		pr_err("invalid policer ID %d, out of range[%d, %d]\n",	policer_id,
			MVPP2_PLCR_MIN_ENTRY_ID, MVPP2_PLCR_MAX - 1);
		return -EINVAL;
	}

	/* check police status */
	rc = pp2_cls_db_plcr_entry_get(inst, policer_id, &l_plcr_entry);
	if (rc) {
		pr_err("failed to get policer entry from DB\n");
		return rc;
	}
	if (l_plcr_entry.valid == MVPP2_PLCR_ENTRY_INVALID_STATE) {
		pr_err("policer_id(%d) is invalid, don't need to delete it\n", policer_id);
		return -EINVAL;
	}
	if ((l_plcr_entry.rules_ref_cnt > 0) || (l_plcr_entry.ppios_ref_cnt > 0)) {
		pr_err("the policer is still used, rules_ref_cnt(%d) or ppios_ref_cnt(%d), could not be deleted\n",
			l_plcr_entry.rules_ref_cnt, l_plcr_entry.ppios_ref_cnt);
		return -EINVAL;
	}

	/* Delete policer entry from HW */
	rc = pp2_cls_plcr_hw_entry_del(inst, policer_id);
	if (rc) {
		pr_err("failed to delete policer entry from HW\n");
		return rc;
	}

	/* Delete policer entry from DB */
	MVPP2_MEMSET_ZERO(l_plcr_entry);
	l_plcr_entry.valid = MVPP2_PLCR_ENTRY_INVALID_STATE;
	rc = pp2_cls_db_plcr_entry_set(inst, policer_id, &l_plcr_entry);
	if (rc) {
		pr_err("failed to delete policer entry from DB\n");
		return rc;
	}

	return rc;
}

/*******************************************************************************
* pp2_cls_plcr_ref_cnt_update()
*
* DESCRIPTION: This API update the reference counter for a policer entry.
*
* INPUTS:
*	policer_id  - policer ID.
*	cnt_action  - reference counter action
*
* OUTPUTS:
*	None.
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
*******************************************************************************/
int pp2_cls_plcr_ref_cnt_update(struct pp2_inst *inst,
				u8 policer_id,
				enum pp2_cls_plcr_ref_cnt_action_t cnt_action,
				int update_ppio)
{
	int rc = 0;

	if (mv_pp2x_range_validate(policer_id, MVPP2_PLCR_MIN_ENTRY_ID, MVPP2_PLCR_MAX - 1)) {
		pr_err("invalid policer ID %d, out of range[%d, %d]\n",	policer_id,
			MVPP2_PLCR_MIN_ENTRY_ID, MVPP2_PLCR_MAX - 1);
		return -EINVAL;
	}

	if (mv_pp2x_range_validate(cnt_action, 0, MVPP2_PLCR_REF_CNT_CLEAR)) {
		pr_err("invalid reference counter action %d, out of range[%d, %d]\n", cnt_action,
			0, MVPP2_PLCR_REF_CNT_CLEAR);
		return -EINVAL;
	}

	/* update the policer reference counter in DB */
	rc = pp2_cls_db_plcr_ref_cnt_update(inst, policer_id, cnt_action, update_ppio);
	if (rc) {
		pr_err("failed to update policer reference counter\n");
		return rc;
	}

	return rc;
}

/*******************************************************************************
* pp2_cls_plcr_ref_cnt_get()
*
* DESCRIPTION: This API get the refernce count of both rules and ppios.
*
* INPUTS:
*	policer_id    - policer ID.
*
* OUTPUTS:
*	None.
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
*******************************************************************************/
int pp2_cls_plcr_ref_cnt_get(struct pp2_inst *inst, u8 policer_id, u32 *rules_ref, u32 *ppios_ref)
{
	struct pp2_cls_db_plcr_entry_t l_plcr_entry;
	int rc = 0;

	if (mv_pp2x_ptr_validate(inst))
		return -EFAULT;

	if (mv_pp2x_range_validate(policer_id, 0, MVPP2_PLCR_MAX - 1)) {
		pr_err("invalid policer ID %d, out of range[%d, %d]\n",	policer_id,
			0, MVPP2_PLCR_MAX - 1);
		return -EINVAL;
	}

	/* check police status */
	rc = pp2_cls_db_plcr_entry_get(inst, policer_id, &l_plcr_entry);
	if (rc) {
		pr_err("failed to get policer entry from DB\n");
		return rc;
	}

	if (rules_ref)
		*rules_ref = l_plcr_entry.rules_ref_cnt;
	if (ppios_ref)
		*ppios_ref = l_plcr_entry.ppios_ref_cnt;

	return rc;
}

/*******************************************************************************
* pp2_cls_plcr_entry_state_get()
*
* DESCRIPTION: This API get the status of a policer entry.
*
* INPUTS:
*	policer_id    - policer ID.
*
* OUTPUTS:
*	None.
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
*******************************************************************************/
int pp2_cls_plcr_entry_state_get(struct pp2_inst *inst, u8 policer_id, enum pp2_cls_plcr_entry_state_t *state)
{
	struct pp2_cls_db_plcr_entry_t l_plcr_entry;
	int rc = 0;

	if (mv_pp2x_ptr_validate(state))
		return -EFAULT;

	if (mv_pp2x_range_validate(policer_id, 0, MVPP2_PLCR_MAX - 1)) {
		pr_err("invalid policer ID %d, out of range[%d, %d]\n",	policer_id,
			0, MVPP2_PLCR_MAX - 1);
		return -EINVAL;
	}

	/* check police status */
	rc = pp2_cls_db_plcr_entry_get(inst, policer_id, &l_plcr_entry);
	if (rc) {
		pr_err("failed to get policer entry from DB\n");
		return rc;
	}
	*state = l_plcr_entry.valid;

	return rc;
}

/*******************************************************************************
* pp2_cls_plcr_entry_clear()
*
* DESCRIPTION: This API deletes all policer entries.
*
* INPUTS:
*	None.
*
* OUTPUTS:
*	None.
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
*******************************************************************************/
int pp2_cls_plcr_entry_clear(struct pp2_inst *inst)
{
	unsigned int idx = 0;
	struct pp2_cls_db_plcr_entry_t plcr_entry;
	int rc = 0;

	for (idx = MVPP2_PLCR_MIN_ENTRY_ID; idx < MVPP2_PLCR_MAX; idx++) {
		/* get policer from DB */
		rc = pp2_cls_db_plcr_entry_get(inst, idx, &plcr_entry);
		if (rc) {
			pr_err("failed to get policer entry from DB\n");
			return rc;
		}

		/* delete the policer if it is valid */
		if (plcr_entry.valid == MVPP2_PLCR_ENTRY_VALID_STATE) {
			rc = pp2_cls_plcr_entry_del(inst, idx);
			if (rc) {
				pr_err("failed to delete policer entry\n");
				return rc;
			}
		}
	}

	return rc;
}

/*******************************************************************************
* pp2_cls_plcr_gen_cfg_set()
*
* DESCRIPTION: This API sets policer general parameters.
*
* INPUTS:
*	gen_cfg - policer general configuration.
*
* OUTPUTS:
*	None.
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
*******************************************************************************/
int pp2_cls_plcr_gen_cfg_set(struct pp2_inst *inst, struct pp2_cls_plcr_gen_cfg_t *gen_cfg)
{
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	int rc;

	/* parameter verification */
	if (mv_pp2x_ptr_validate(gen_cfg))
		return -EFAULT;

	if ((gen_cfg->rate_state != MVPP2_PLCR_BASE_RATE_ENABLE) &&
	    (gen_cfg->rate_state != MVPP2_PLCR_BASE_RATE_DISABLE)) {
		pr_err("invalid base rate state (%d)\n", gen_cfg->rate_state);
		return -EINVAL;
	}

	if ((gen_cfg->base_period != MVPP2_TOKEN_PERIOD_400_CORE_CLOCK) &&
	    (gen_cfg->base_period != MVPP2_TOKEN_PERIOD_480_CORE_CLOCK) &&
	    (gen_cfg->base_period != MVPP2_TOKEN_PERIOD_600_CORE_CLOCK) &&
	    (gen_cfg->base_period != MVPP2_TOKEN_PERIOD_800_CORE_CLOCK)) {
		pr_err("invalid token base period(%d)\n", gen_cfg->base_period);
		return -EINVAL;
	}

	if ((gen_cfg->mode != MVPP2_PLCR_MODE_SERIAL_BANK_0_1) &&
	    (gen_cfg->mode != MVPP2_PLCR_MODE_SERIAL_BANK_1_0) &&
	    (gen_cfg->mode != MVPP2_PLCR_MODE_PARALLEL) &&
	    (gen_cfg->mode != MVPP2_PLCR_MODE_ONLY_BANK_0)) {
		pr_err("invalid mode(%d)\n", gen_cfg->mode);
		return -EINVAL;
	}

	/* set token base period */
	rc = mv_pp2x_plcr_hw_base_period_set(cpu_slot, gen_cfg->base_period);
	if (rc) {
		pr_err("failed to set token base period to HW\n");
		return rc;
	}

	/* set base rate generation state */
	rc = mv_pp2x_plcr_hw_base_rate_gen_enable(cpu_slot, gen_cfg->rate_state);
	if (rc) {
		pr_err("failed to set token base rate generation to HW\n");
		return rc;
	}

	/* set min packet length */
	rc = mv_pp2x_plcr_hw_min_pkt_len(cpu_slot, gen_cfg->min_pkt_len);
	if (rc) {
		pr_err("failed to set min packet length to HW\n");
		return rc;
	}

	/* set operation mode */
	rc = mv_pp2x_plcr_hw_mode(cpu_slot, gen_cfg->mode);
	if (rc) {
		pr_err("failed to set mode to HW\n");
		return rc;
	}

	/* save the general configuration to DB */
	rc = pp2_cls_db_plcr_gen_cfg_set(inst, gen_cfg);
	if (rc) {
		pr_err("failed to set policer general configuration to DBr\n");
		return rc;
	}

	return 0;
}

/*******************************************************************************
* pp2_cls_plcr_early_drop_set()
*
* DESCRIPTION: This API sets policer early drop parameters.
*
* INPUTS:
*	early_drop - early drop parameters.
*
* OUTPUTS:
*	None.
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
*******************************************************************************/
int pp2_cls_plcr_early_drop_set(struct pp2_inst *inst, struct pp2_cls_plcr_early_drop_t *early_drop)
{
	unsigned int gmac;
	unsigned int idx;
	int txq;
	uintptr_t cpu_slot = pp2_default_cpu_slot(inst);
	int rc = 0;

	if (mv_pp2x_ptr_validate(early_drop))
		return -EFAULT;

	if ((early_drop->state != MVPP2_PLCR_EARLY_DROP_ENABLE) &&
	    (early_drop->state != MVPP2_PLCR_EARLY_DROP_DISABLE)) {
		pr_err("invalid early drop state (%d)\n", early_drop->state);
		return -EINVAL;
	}

	if (early_drop->state == MVPP2_PLCR_EARLY_DROP_ENABLE) {
		/* set CPU thresholds */
		for (idx = 0; idx < MVPP2_PLCR_EDROP_THRESH_NUM; idx++) {
			rc = mv_pp2x_plcr_hw_cpu_thresh_set(cpu_slot, idx, early_drop->cpu_q_thesh[idx]);
			if (rc) {
				pr_err("failed to set CPU queue threshold to HW\n");
				return rc;
			}
		}

		/* set HWF thresholds */
		for (idx = 0; idx < MVPP2_PLCR_EDROP_THRESH_NUM; idx++) {
			rc = mv_pp2x_plcr_hw_hwf_thresh_set(cpu_slot, idx, early_drop->hwf_q_thesh[idx]);
			if (rc) {
				pr_err("failed to set HWF queue threshold to HW\n");
				return rc;
			}
		}

		/* set CPU RX queue index */
		for (idx = 0; idx < MVPP2_RXQ_TOTAL_NUM; idx++) {
			if (early_drop->rxq_idx[idx] != MVPP2_PLCR_INVALID_Q_THESH_IDX) {
				rc = mv_pp2x_plcr_hw_rxq_thresh_set(cpu_slot, idx, early_drop->rxq_idx[idx]);
				if (rc) {
					pr_err("failed to set RX queue threshold index to HW\n");
					return rc;
				}
			}
		}

		/* set HWF TX queue index */
		for (gmac = 0; gmac < MVPP2_MAX_PORTS; gmac++) {
			for (idx = 0; idx < MVPP2_MAX_TXQ; idx++) {
				if (early_drop->txq_idx[gmac][idx] != MVPP2_PLCR_INVALID_Q_THESH_IDX) {
					txq = (gmac * MVPP2_MAX_TXQ) | idx;
					rc = mv_pp2x_plcr_hw_txq_thresh_set(cpu_slot, txq,
									    early_drop->txq_idx[gmac][idx]);
					if (rc) {
						pr_err("fail to set TX queue threshold index to HW\n");
						return rc;
					}
				}
			}
		}
	}

	/* set early drop state */
	rc = mv_pp2x_plcr_hw_early_drop_set(cpu_slot, early_drop->state);
	if (rc) {
		pr_err("failed to set early drop to HW\n");
		return rc;
	}

	/* save the early drop configuration to DB */
	rc = pp2_cls_db_plcr_early_drop_set(inst, early_drop);
	if (rc) {
		pr_err("failed to set policer early drop configuration to DB\n");
		return rc;
	}

	return rc;
}

/*******************************************************************************
* pp2_cls_plcr_reset
*
* DESCRIPTION: The routine reset and re-start policer sub-module
*
* INPUTS:
*	None
*
* OUTPUTS:
*	None
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
* COMMENTS:
*	This routine is called by pp2_mib_reset
*******************************************************************************/
int pp2_cls_plcr_reset(struct pp2_inst *inst)
{
	struct pp2_cls_plcr_gen_cfg_t gen_cfg;
	struct pp2_cls_plcr_early_drop_t early_drop;
	int rc = 0;

	/* init policer DB */
	rc = pp2_cls_db_plcr_init(inst);
	if (rc) {
		pr_err("fail to init policer DB\n");
		return rc;
	}

	/* set policer default general configuration */
	MVPP2_MEMSET_ZERO(gen_cfg);
	gen_cfg.rate_state  = MVPP2_PLCR_BASE_RATE_ENABLE;
	gen_cfg.mode        = MVPP2_PLCR_MODE_ONLY_BANK_0;
	gen_cfg.base_period = MVPP2_TOKEN_PERIOD_800_CORE_CLOCK;
	gen_cfg.min_pkt_len = MVPP2_PLCR_MIN_PKT_LEN;
	rc = pp2_cls_plcr_gen_cfg_set(inst, &gen_cfg);
	if (rc) {
		pr_err("fail to set policer default general configuration\n");
		return rc;
	}

	/* set policer default early drop configuration */
	MVPP2_MEMSET_ZERO(early_drop);
	early_drop.state = MVPP2_PLCR_EARLY_DROP_DISABLE;
	rc = pp2_cls_plcr_early_drop_set(inst, &early_drop);
	if (rc) {
		pr_err("fail to set policer default early drop configuration\n");
		return rc;
	}

	return rc;
}

/*******************************************************************************
* pp2_cls_plcr_start
*
* DESCRIPTION: The routine starts policer sub-module
*
* INPUTS:
*	None
*
* OUTPUTS:
*	None
*
* RETURNS:
*	On success, the function returns 0. On error different types are returned
*	according to the case - see pp2_error_code_t.
* COMMENTS:
*	This routine is called by pp2_start
*******************************************************************************/
int pp2_cls_plcr_start(struct pp2_inst *inst)
{
	int i;

	if (pp2_cls_plcr_reset(inst) != 0) {
		pr_err("MVPP2 policer start failed\n");
		return -EINVAL;
	}

	/* Make sure policers are disabled */
	for (i = 0; i < PP2_CLS_PLCR_NUM; i++) {
		if (!(pp2_ptr->init.policers_reserved_map & (1 << i)))
			pp2_cls_plcr_hw_entry_del(inst, (i+1));
	}

	return 0;
}

void pp2_cls_plcr_finish(struct pp2_inst *inst)
{
	if (pp2_cls_plcr_entry_clear(inst) != 0)
		pr_debug("MVPP2 policer finish failed\n");

}


