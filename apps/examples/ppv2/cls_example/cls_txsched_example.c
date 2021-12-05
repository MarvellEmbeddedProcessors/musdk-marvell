/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdio.h>
#include "cls_main_example.h"

/*
 * pp2_cls_txsched_params_example()
 * example for txsched setting.
 */
int pp2_cls_txsched_params_example(struct port_desc *port)
{
	port->port_params.rate_limit.rate_limit_enable = 1;
	port->port_params.rate_limit.rate_limit_params.cbs = 1000;  /* burst size, in kB */
	port->port_params.rate_limit.rate_limit_params.cir = 10000; /* rate limit, in kbps */

	port->port_params.outqs_params.outqs_params[0].sched_mode = PP2_PPIO_SCHED_M_WRR; /* Weighted Round Robin */
	port->port_params.outqs_params.outqs_params[0].weight = 1;

	port->port_params.outqs_params.outqs_params[1].sched_mode = PP2_PPIO_SCHED_M_WRR;
	port->port_params.outqs_params.outqs_params[1].weight = 10;

	port->port_params.outqs_params.outqs_params[2].sched_mode = PP2_PPIO_SCHED_M_SP; /* Strict Priority */
	port->port_params.outqs_params.outqs_params[2].rate_limit.rate_limit_enable = 1;
	port->port_params.outqs_params.outqs_params[2].rate_limit.rate_limit_params.cbs = 1000;
	port->port_params.outqs_params.outqs_params[2].rate_limit.rate_limit_params.cir = 1000;

	return 0;
}
