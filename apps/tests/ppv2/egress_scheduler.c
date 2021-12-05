/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
