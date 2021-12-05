/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************/
/* h file declarations */
/***********************/
#ifndef _PP2_MNG_H_
#define _PP2_MNG_H_

#include "drivers/mv_pp2_cls.h"
#include "drivers/ppv2/pp2.h"
/******************************************************************************/
/*                                 MACROS                                     */
/******************************************************************************/
#define CLS_MNG_KEY_SIZE_MAX			37	/* Max possible size of HEK key */
#define CLS_MNG_RULES_SIZE_MAX			20	/* Max possible rules in CLS table */


/******************************************************************************/
/*                              ENUMERATIONS                                  */
/******************************************************************************/
/******************************************************************************/
/*                               STRUCTURES                                   */
/******************************************************************************/

/******************************************************************************/
/*                                PROTOTYPE                                   */
/******************************************************************************/
void pp2_cls_mng_init(struct pp2_inst *inst);
void pp2_cls_mng_deinit(struct pp2_inst *inst);
int pp2_cls_mng_tbl_init(struct pp2_cls_tbl_params *params, struct pp2_cls_tbl **tbl, int lkp_type);
int pp2_cls_mng_table_deinit(struct pp2_cls_tbl *tbl);
int pp2_cls_mng_rule_add(struct pp2_cls_tbl *tbl, struct pp2_cls_tbl_rule *rule,
			 struct pp2_cls_tbl_action *action, int lkp_type);
int pp2_cls_mng_rule_modify(struct pp2_cls_tbl *tbl, struct pp2_cls_tbl_rule *rule, struct pp2_cls_tbl_action *action);
int pp2_cls_mng_rule_remove(struct pp2_cls_tbl *tbl, struct pp2_cls_tbl_rule *rule);
int pp2_cls_mng_set_logical_port_params(struct pp2_ppio *ppio, struct pp2_ppio_params *params);
int pp2_cls_dscp_flow_modify(struct pp2_port *port, int set);
int pp2_cls_mng_qos_tbl_init(struct pp2_cls_qos_tbl_params *qos_params, struct pp2_cls_tbl **tbl);
int pp2_cls_mng_qos_tbl_deinit(struct pp2_cls_tbl *tbl);
int pp2_cls_mng_lkp_type_to_prio(int lkp_type);
void pp2_cls_mng_rss_port_init(struct pp2_port *port, u16 rss_map);
void pp2_cls_mng_config_default_cos_queue(struct pp2_port *port);
int pp2_cls_mng_eth_start_header_params_set(struct pp2_ppio *ppio,
					    enum pp2_ppio_eth_start_hdr eth_start_hdr);
int pp2_cls_mng_modify_default_flows(struct pp2_ppio *ppio, int clear);

#endif /* _PP2_MNG_H_ */
