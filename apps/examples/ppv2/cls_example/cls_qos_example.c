/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdio.h>
#include "mvapp.h"
#include "cls_main_example.h"

/*
 * pp2_cls_qos_table_add_example()
 * is example for qos setting,
 * 1. set qos type to vlan-ip
 * 2. fill all pcp  table with q='2' except for index 1 -> q='1' and index 2 -> q='0'
 * 3. fill all dscp table with q='2' except for index 4 -> q='1' and index 6 -> q='3'
 */
int pp2_cls_qos_table_add_example(struct pp2_ppio *ppio)
{
	int type;
	int pcp_map[MV_VLAN_PRIO_NUM];
	int dscp_map[MV_DSCP_NUM];
	struct pp2_cls_tbl		*tbl;
	struct pp2_cls_qos_tbl_params	*qos_tbl_params;
	int i;

	/* set qos to vlan-ip  */
	type = PP2_CLS_QOS_TBL_VLAN_IP_PRI;

	/* fill pcp_table with default values*/
	for (i = 0; i < MV_VLAN_PRIO_NUM; i++)
		pcp_map[i] = 2;
	/* select specific pcp value */
	pcp_map[1] = 1;
	pcp_map[2] = 0;
	/* fill dscp_table with default values*/
	for (i = 0; i < MV_DSCP_NUM; i++)
		dscp_map[i] = 2;
	/* select specific dscp value */
	dscp_map[4] = 1;
	dscp_map[6] = 3;

	qos_tbl_params = malloc(sizeof(*qos_tbl_params));
	if (!qos_tbl_params) {
		pr_err("%s no mem for qos_tbl_params\n", __func__);
		return -ENOMEM;
	}
	memset(qos_tbl_params, 0, sizeof(*qos_tbl_params));

	qos_tbl_params->type = type;

	/* fill pcp_table wit default values */
	for (i = 0; i < MV_VLAN_PRIO_NUM; i++) {
		qos_tbl_params->pcp_cos_map[i].tc = pcp_map[i];
		qos_tbl_params->pcp_cos_map[i].ppio = ppio;
		pr_debug("pcp[%d] %d\n", i, qos_tbl_params->pcp_cos_map[i].tc);
	}

	/* fill dscp_table wit default values */
	for (i = 0; i < MV_DSCP_NUM; i++) {
		qos_tbl_params->dscp_cos_map[i].tc = dscp_map[i];
		qos_tbl_params->dscp_cos_map[i].ppio = ppio;
		pr_debug("dscp[%d] %d\n", i, qos_tbl_params->dscp_cos_map[i].tc);
	}

	if (!pp2_cls_qos_tbl_init(qos_tbl_params, &tbl))
		printf("OK\n");
	else
		printf("FAIL\n");

	return 0;
}
