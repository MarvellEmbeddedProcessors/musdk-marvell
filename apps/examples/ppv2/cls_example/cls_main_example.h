/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __CLS_MAIN_EXAMPLE_H__
#define __CLS_MAIN_EXAMPLE_H__

#include "drivers/mv_pp2_cls.h"
#include "pp2_utils.h"
#include "lib/list.h"

int pp2_cls_logical_port_params_example(struct pp2_ppio_params *port_params,
					enum pp2_ppio_type *port_type);

int pp2_cls_add_5_tuple_table(struct port_desc *ports_desc);
int pp2_cls_example_rule_key(struct port_desc *ports_desc);

int pp2_cls_qos_table_add_example(struct pp2_ppio *ppio);

int register_cli_filter_cmds(struct pp2_ppio *ppio);

int pp2_cls_policer_params_example(struct port_desc *port);

int pp2_cls_txsched_params_example(struct port_desc *port);

#endif /*__CLS_MAIN_EXAMPLE_H__*/
