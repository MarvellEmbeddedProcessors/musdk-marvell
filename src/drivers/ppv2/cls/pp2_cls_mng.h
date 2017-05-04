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
int pp2_cls_mng_lkp_type_to_prio(int lkp_type);
void pp2_cls_mng_rss_port_init(struct pp2_port *port, u16 rss_map);

#endif /* _PP2_MNG_H_ */
