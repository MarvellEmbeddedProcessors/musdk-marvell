/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#ifndef _GUEST_MNG_CMD_H_
#define _GUEST_MNG_CMD_H_

#include "drivers/mv_pp2_cls.h"

#define GUEST_PP2_CLS_KEY_SIZE_MAX	40	/* Max possible size of HEK key */

/*
 * msg_from_guest_codes - Define the list of messages codes that can be send by guest.
 */
enum msg_from_guest_codes {
	MSG_F_GUEST_NONE = 0,
	MSG_F_GUEST_TABLE_INIT,
	MSG_F_GUEST_TABLE_DEINIT,
	MSG_F_GUEST_ADD_RULE,
	MSG_F_GUEST_MODIFY_RULE,
	MSG_F_GUEST_REMOVE_RULE,
	MSG_F_GUEST_KA,
	MSG_F_GUEST_GPIO_ENABLE,
	MSG_F_GUEST_GPIO_DISABLE,
	MSG_F_GUEST_LAST,
};

struct guest_pp2_cls_tbl_action {
	struct pp2_cls_tbl_action action;
	struct pp2_cls_cos_desc cos;
};

struct guest_pp2_cls_tbl_params {
	struct pp2_cls_tbl_params params;
	struct guest_pp2_cls_tbl_action def_action;
};

struct guest_pp2_cls_rule_key_field {
	u8	size;
	int	key_valid;
	u8	key[GUEST_PP2_CLS_KEY_SIZE_MAX];
	int	mask_valid;
	u8	mask[GUEST_PP2_CLS_KEY_SIZE_MAX];
};

struct guest_pp2_cls_tbl_rule {
	u8	num_fields;
	struct guest_pp2_cls_rule_key_field fields[PP2_CLS_TBL_MAX_NUM_FIELDS];
};

struct guest_pp2_cls_rule_add {
	u32 tbl_id;
	struct guest_pp2_cls_tbl_rule rule;
	struct guest_pp2_cls_tbl_action action;
};

struct guest_pp2_cls_rule_remove {
	u32 tbl_id;
	struct guest_pp2_cls_tbl_rule rule;
};

struct guest_pp2_cls_cmd_resp {
	union {
		struct {
			u32	tbl_id;
		} tbl_init;
	};
};

struct guest_cmd_resp {
#define RESP_STATUS_OK		(0)
#define RESP_STATUS_FAIL	(1)
	u8 status;
	union {
		struct guest_pp2_cls_cmd_resp pp2_cls_resp;
	};
};

#endif /* _GUEST_MNG_CMD_H_ */
