/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __PP2_MAIN_H__
#define __PP2_MAIN_H__

#include "drivers/mv_pp2_cls.h"
#include "pp2_utils.h"
#include "lib/list.h"

struct pp2_cls_table_node {
	u32				idx;
	struct	pp2_cls_tbl		*tbl;
	struct	pp2_cls_tbl_params	tbl_params;
	struct	pp2_cls_qos_tbl_params	qos_tbl_params;
	char				ppio_name[MVAPPS_PPIO_NAME_MAX];
	struct list			list_node;
};

int register_cli_cls_api_cmds(struct port_desc *arg);
int register_cli_cls_api_qos_cmds(struct port_desc *arg);
int register_cli_cls_api_plcr_cmds(struct port_desc *arg);
int register_cli_cls_api_edrop_cmds(struct port_desc *arg);
int register_cli_filter_cmds(struct pp2_ppio *ppio);

void unregister_cli_cls_api_cmds(void);
void unregister_cli_cls_api_qos_cmds(void);
void unregister_cli_cls_api_plcr_cmds(void);
void unregister_cli_cls_api_edrop_cmds(void);

int pp2_cls_cli_ppio_tag_mode(struct pp2_ppio_params *port_params, int argc, char *argv[]);
int pp2_cls_logical_port_params(struct pp2_ppio_params *port_params, int argc, char *argv[]);
int pp2_egress_scheduler_params(struct pp2_ppio_params *port_params, int argc, char *argv[]);

int pp2_cls_table_remove(u32 tbl_idx, struct list *cls_tbl_head);
int pp2_cls_table_next_index_get(struct list *cls_tbl_head);
struct list *pp2_cls_table_next_node_get(struct list *cls_tbl_head, u32 index);
int cli_cls_policer_get(u32 idx, struct pp2_cls_plcr **plcr);
void cli_cls_prepare_policers_db(u32 policers_reserved_map);
int cli_cls_policer_params(struct port_desc *port_desc);
void cli_cls_prepare_edrops_db(u32 policers_reserved_map);

#endif /*__PP2_MAIN_H__*/
