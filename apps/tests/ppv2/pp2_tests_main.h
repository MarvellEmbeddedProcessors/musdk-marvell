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
