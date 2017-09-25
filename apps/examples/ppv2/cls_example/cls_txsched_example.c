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
#include "cls_main_example.h"

/*
 * pp2_cls_txsched_params_example()
 * example for txsched setting.
 */
int pp2_cls_txsched_params_example(struct port_desc *port)
{
	port->port_params.rate_limit_enable = 1;
	port->port_params.rate_limit_params.cbs = 1000;  /* burst size, in kB */
	port->port_params.rate_limit_params.cir = 10000; /* rate limit, in kbps */

	port->port_params.outqs_params.outqs_params[0].sched_mode = PP2_PPIO_SCHED_M_WRR; /* Weighted Round Robin */
	port->port_params.outqs_params.outqs_params[0].weight = 1;

	port->port_params.outqs_params.outqs_params[1].sched_mode = PP2_PPIO_SCHED_M_WRR;
	port->port_params.outqs_params.outqs_params[1].weight = 10;

	port->port_params.outqs_params.outqs_params[2].sched_mode = PP2_PPIO_SCHED_M_SP; /* Strict Priority */
	port->port_params.outqs_params.outqs_params[2].rate_limit_enable = 1;
	port->port_params.outqs_params.outqs_params[2].rate_limit_params.cbs = 1000;
	port->port_params.outqs_params.outqs_params[2].rate_limit_params.cir = 1000;

	return 0;
}
