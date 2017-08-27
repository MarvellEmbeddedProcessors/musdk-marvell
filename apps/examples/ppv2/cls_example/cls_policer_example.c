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
