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
