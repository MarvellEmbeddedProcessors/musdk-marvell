/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include <string.h>
#include <stdio.h>
#include <getopt.h>
#include "mvapp.h"
#include "mv_pp2_ppio.h"
#include "pp2_tests_main.h"

int pp2_egress_scheduler_params(struct pp2_ppio_params *port_params, int argc, char *argv[])
{
	char *ret_ptr;
	int i, option;
	int long_index = 0;
	u8 txq;
	struct option long_options[] = {
		{"port_rate_limit_enable", no_argument, 0, 'p'},
		{"port_rate_limit", required_argument, 0, 'r'},
		{"port_burst_size", required_argument, 0, 'b'},
		{"txq_rate_limit_enable", required_argument, 0, 'l'},
		{"txq_rate_limit", required_argument, 0, 't'},
		{"txq_burst_size", required_argument, 0, 'q'},
		{"txq_arb_mode", required_argument, 0, 'a'},
		{"txq_wrr_weight", required_argument, 0, 'w'},
		{0, 0, 0, 0}
	};

	if  (argc < 2 || argc > (6 + 8 * PP2_PPIO_MAX_NUM_OUTQS)) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		/* Get parameters */
		switch (option) {
		case 'p':
			port_params->rate_limit.rate_limit_enable = 1;
			break;
		case 'r':
			port_params->rate_limit.rate_limit_params.cir = strtoul(optarg, &ret_ptr, 10);
			if (ret_ptr == optarg) {
				pr_err("port_rate_limit must contain a decimal integer.\n");
				return -EINVAL;
			}
			break;
		case 'b':
			port_params->rate_limit.rate_limit_params.cbs = strtoul(optarg, &ret_ptr, 10);
			if (ret_ptr == optarg) {
				pr_err("port_burst_size must contain a decimal integer.\n");
				return -EINVAL;
			}
			break;
		case 'l':
			txq = strtoul(optarg, &ret_ptr, 10);
			if (ret_ptr == optarg) {
				pr_err("Invalid txq id in txq_rate_limit_enable.\n");
				return -EINVAL;
			}
			port_params->outqs_params.outqs_params[txq].rate_limit.rate_limit_enable = 1;
			break;
		case 't':
			txq = strtoul(optarg, &ret_ptr, 10);
			if (ret_ptr == optarg) {
				pr_err("Invalid txq id in txq_rate_limit.\n");
				return -EINVAL;
			}
			if (*ret_ptr++ != ',') {
				pr_err("The two integers must be separated by a comma in txq_rate_limit for txq %u.\n",
				       txq);
				return -EINVAL;
			}
			optarg = ret_ptr;
			port_params->outqs_params.outqs_params[txq].rate_limit.rate_limit_params.cir =
				strtoul(optarg, &ret_ptr, 10);
			if (ret_ptr == optarg) {
				pr_err("Invalid rate limit in txq_rate_limit.\n");
				return -EINVAL;
			}
			break;
		case 'q':
			txq = strtoul(optarg, &ret_ptr, 10);
			if (ret_ptr == optarg) {
				pr_err("Invalid txq id in txq_burst_size.\n");
				return -EINVAL;
			}
			if (*ret_ptr++ != ',') {
				pr_err("The two integers must be separated by a comma in txq_burst_size for txq %u.\n",
				       txq);
				return -EINVAL;
			}
			optarg = ret_ptr;
			port_params->outqs_params.outqs_params[txq].rate_limit.rate_limit_params.cbs =
				strtoul(optarg, &ret_ptr, 10);
			if (ret_ptr == optarg) {
				pr_err("Invalid burst size in txq_burst_size.\n");
				return -EINVAL;
			}
			break;
		case 'a':
			txq = strtoul(optarg, &ret_ptr, 10);
			if (ret_ptr == optarg) {
				pr_err("Invalid txq id in txq_arb_mode.\n");
				return -EINVAL;
			}
			if (*ret_ptr++ != ',') {
				pr_err("The two integers must be separated by a comma in txq_arb_mode for txq %u.\n",
				       txq);
				return -EINVAL;
			}
			optarg = ret_ptr;
			port_params->outqs_params.outqs_params[txq].sched_mode = strtoul(optarg, &ret_ptr, 10);
			if (ret_ptr == optarg) {
				pr_err("Invalid burst size in txq_arb_mode.\n");
				return -EINVAL;
			}
			break;
		case 'w':
			txq = strtoul(optarg, &ret_ptr, 10);
			if (ret_ptr == optarg) {
				pr_err("Invalid txq id in txq_wrr_weight.\n");
				return -EINVAL;
			}
			if (*ret_ptr++ != ',') {
				pr_err("The two integers must be separated by a comma in txq_wrr_weight for txq %u.\n",
				       txq);
				return -EINVAL;
			}
			optarg = ret_ptr;
			port_params->outqs_params.outqs_params[txq].weight = strtoul(optarg, &ret_ptr, 10);
			if (ret_ptr == optarg) {
				pr_err("Invalid weight in txq_wrr_weight.\n");
				return -EINVAL;
			}
			break;
		default:
			printf("(%d) parsing fail, wrong input\n", __LINE__);
			return -EINVAL;
		}
	}

	return 0;
}
