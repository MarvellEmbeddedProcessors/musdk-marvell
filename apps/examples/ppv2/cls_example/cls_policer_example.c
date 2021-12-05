/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdio.h>
#include "cls_main_example.h"

/*
 * pp2_cls_policer_params_example()
 * example for policer setting.
 * 1. create policer . one for each pp2 instance
 * 2. attach the instance's policer to the port
 */
int pp2_cls_policer_params_example(struct port_desc *port)
{
	struct pp2_cls_plcr_params policer_params = {0};
	static struct pp2_cls_plcr *policers[] = { NULL, NULL};

	if (policers[0] == NULL && policers[1] == NULL) {
		policer_params.token_unit = PP2_CLS_PLCR_BYTES_TOKEN_UNIT;
		policer_params.cir = 200;
		policer_params.cbs = 0;
		policer_params.ebs = 0;
		policer_params.match = "policer-0:0";
		pp2_cls_plcr_init(&policer_params, &policers[0]);
		policer_params.match = "policer-1:0";
		pp2_cls_plcr_init(&policer_params, &policers[1]);
	}

	port->port_params.inqs_params.plcr = policers[port->pp_id];

	return 0;
}
