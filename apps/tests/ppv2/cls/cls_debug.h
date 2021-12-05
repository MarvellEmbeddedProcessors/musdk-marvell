/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __CLS_DBG_H__
#define __CLS_DBG_H__

#include "mv_std.h"
/* #define CLS_DEBUG */

int register_cli_cls_cmds(struct pp2_ppio *ppio);
int register_cli_c3_cmds(struct pp2_ppio *ppio);
int register_cli_c2_cmds(struct pp2_ppio *ppio);
int register_cli_qos_cmds(struct pp2_ppio *ppio);
int register_cli_mng_cmds(struct pp2_ppio *ppio);
int register_cli_prs_cmds(struct pp2_ppio *ppio);
int register_cli_cntrs_cmds(struct pp2_ppio *ppio);
int register_cli_rss_cmds(struct pp2_ppio *ppio);

#endif /*__CLS_DBG_H__*/
