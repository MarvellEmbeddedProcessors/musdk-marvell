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
#include <getopt.h>
#include "mvapp.h"
#include "cls_main.h"
#include "src/drivers/ppv2/cls/pp2_cls_utils.h"

static int pp2_cls_cli_mac_addr(void *arg, int argc, char *argv[])
{
	struct pp2_ppio *ppio = (struct pp2_ppio *)arg;
	int rc;
	int i, option, uc, mc;
	int long_index = 0;
	u8 mac[ETH_ALEN];
	struct option long_options[] = {
		{"set", required_argument, 0, 's'},
		{"get", no_argument, 0, 'g'},
		{"add", required_argument, 0, 'a'},
		{"remove", required_argument, 0, 'r'},
		{"flush", no_argument, 0, 'f'},
		{"uc", no_argument, 0, 'u'},
		{"mc", no_argument, 0, 'm'},
		{0, 0, 0, 0}
	};

	if  (argc < 2 || argc > 4) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		/* Get parameters */
		switch (option) {
		case 's':
			rc = mv_pp2x_parse_mac_address(optarg, mac);
			if (rc) {
				printf("parsing fail, wrong input for mac_address set\n");
				return -EINVAL;
			}
			printf("mac_addr set1 %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

			rc = pp2_ppio_set_mac_addr(ppio, mac);
			if (rc) {
				printf("Unable to set mac address\n");
				return -EINVAL;
			}
			break;
		case 'g':
			rc = pp2_ppio_get_mac_addr(ppio, mac);
			if (rc) {
				printf("Unable to get mac address\n");
				return -EINVAL;
			}

			printf("mac_addr get %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
			break;
		case 'a':
			rc = mv_pp2x_parse_mac_address(optarg, mac);
			if (rc) {
				printf("parsing fail, wrong input for mac_address add\n");
				return -EINVAL;
			}
			printf("mac_addr add %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

			rc = pp2_ppio_add_mac_addr(ppio, mac);
			if (rc) {
				printf("Unable to add mac address\n");
				return -EINVAL;
			}
			break;
		case 'r':
			rc = mv_pp2x_parse_mac_address(optarg, mac);
			if (rc) {
				printf("parsing fail, wrong input for mac_address remove\n");
				return -EINVAL;
			}
			printf("mac_addr remove %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

			rc = pp2_ppio_remove_mac_addr(ppio, mac);
			if (rc) {
				printf("Unable to remove mac address\n");
				return -EINVAL;
			}
			break;
		case 'f':
			uc = 0;
			mc = 0;
			option = getopt_long_only(argc, argv, "", long_options, &long_index);
			if (option == 'u') {
				uc = 1;
				option = getopt_long_only(argc, argv, "", long_options, &long_index);
				if (option == 'm')
					mc = 1;
			} else if (option == 'm') {
				mc = 1;
				option = getopt_long_only(argc, argv, "", long_options, &long_index);
				if (option == 'u')
					uc = 1;
			} else {
				printf("uc or mc flags not selected. Command ignored\n");
				return -EINVAL;
			}
			rc = pp2_ppio_flush_mac_addrs(ppio, uc, mc);
			if (rc) {
				printf("Unable to flush mac address %d, %d\n", uc, mc);
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}
	return 0;
}

static int pp2_cls_cli_promisc_mode(void *arg, int argc, char *argv[])
{
	struct pp2_ppio *ppio = (struct pp2_ppio *)arg;
	int rc;
	int i, option;
	int long_index = 0;
	int en = -1;
	int cmd = -1;
	struct option long_options[] = {
		{"on", no_argument, 0, 'n'},
		{"off", no_argument, 0, 'f'},
		{"get", no_argument, 0, 'g'},
		{0, 0, 0, 0}
	};

	if  (argc < 2 || argc > 3) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		/* Get parameters */
		switch (option) {
		case 'n':
			if (cmd < 0) {
				cmd = 1;
				en = 1;
			}
			break;
		case 'f':
			if (cmd < 0) {
				cmd = 1;
				en = 0;
			}
			break;
		case 'g':
			if (cmd < 0)
				cmd = 0;
			break;
		default:
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}

	if (cmd == 1) {
		rc = pp2_ppio_set_promisc(ppio, en);
		if (rc) {
			printf("Unable to enable promiscuous mode\n");
			return -rc;
		}
	} else if (cmd == 0) {
		rc = pp2_ppio_get_promisc(ppio, &en);
		if (rc) {
			printf("Unable to get promiscuous mode\n");
			return -rc;
		}
		if (en)
			printf("promiscuous mode: enabled\n");
		else
			printf("promiscuous mode: disabled\n");
	} else {
		pr_err("Option not supported\n");
		return -EFAULT;
	}
	return 0;
}

static int pp2_cls_cli_multicast_mode(void *arg, int argc, char *argv[])
{
	struct pp2_ppio *ppio = (struct pp2_ppio *)arg;
	int rc;
	int i, option;
	int long_index = 0;
	int en = -1;
	int cmd = -1;
	struct option long_options[] = {
		{"on", no_argument, 0, 'n'},
		{"off", no_argument, 0, 'f'},
		{"get", no_argument, 0, 'g'},
		{0, 0, 0, 0}
	};

	if  (argc < 2 || argc > 3) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		/* Get parameters */
		switch (option) {
		case 'n':
			if (cmd < 0) {
				cmd = 1;
				en = 1;
			}
			break;
		case 'f':
			if (cmd < 0) {
				cmd = 1;
				en = 0;
			}
			break;
		case 'g':
			if (cmd < 0)
				cmd = 0;
			break;
		default:
			printf("parsing fail, wrong input\n");
			return -EINVAL;
		}
	}

	if (cmd == 1) {
		rc = pp2_ppio_set_mc_promisc(ppio, en);
		if (rc) {
			printf("Unable to enable all multicast mode\n");
			return -rc;
		}
	} else if (cmd == 0) {
		rc = pp2_ppio_get_mc_promisc(ppio, &en);
		if (rc) {
			printf("Unable to get all multicast mode\n");
			return -rc;
		}
		if (en)
			printf("all multicast mode: enabled\n");
		else
			printf("all multicast mode: disabled\n");
	} else {
		pr_err("Option not supported\n");
		return -EFAULT;
	}
	return 0;
}

static int pp2_cls_cli_vlan(void *arg, int argc, char *argv[])
{
	struct pp2_ppio *ppio = (struct pp2_ppio *)arg;
	int rc;
	char *ret_ptr;
	int i, option;
	int long_index = 0;
	u16 vlan_id;
	int cmd = -1;
	struct option long_options[] = {
		{"set", required_argument, 0, 's'},
		{"remove", required_argument, 0, 'r'},
		{"flush", no_argument, 0, 'f'},
		{0, 0, 0, 0}
	};

	if  (argc < 2 || argc > 4) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		/* Get parameters */
		switch (option) {
		case 's':
			vlan_id = strtoul(optarg, &ret_ptr, 0);
			cmd = 0;
			break;
		case 'r':
			vlan_id = strtoul(optarg, &ret_ptr, 0);
			cmd = 1;
			break;
		case 'f':
			cmd = 2;
			break;
		}
	}

	if (cmd < 0)
		printf("parsing fail, wrong input\n");

	if (cmd == 0 && vlan_id >= 0 && vlan_id <= 4095) {
		rc = pp2_ppio_add_vlan(ppio, vlan_id);
		if (rc) {
			printf("Unable to add vlan id %d\n", vlan_id);
			return -EINVAL;
		}
	} else if (cmd == 1 && vlan_id >= 0 && vlan_id <= 4095) {
		rc = pp2_ppio_remove_vlan(ppio, vlan_id);
		if (rc) {
			printf("Unable to remove vlan id %d\n", vlan_id);
			return -EINVAL;
		}
	} else if (cmd == 2) {
		rc = pp2_ppio_flush_vlan(ppio);
		if (rc) {
			printf("Unable to flush vlans\n");
			return -EINVAL;
		}
	} else {
		printf("parsing fail, wrong input\n");
		return -EINVAL;
	}
	return 0;
}

int register_cli_filter_cmds(struct pp2_ppio *ppio)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "mac_addr";
	cmd_params.desc		= "set/get/add/remove/flush ppio MAC address";
	cmd_params.format	= "--set <xx:xx:xx:xx:xx:xx>\n"
				  "\t\t\t\t\t\t--get\n"
				  "\t\t\t\t\t\t--add <xx:xx:xx:xx:xx:xx>\n"
				  "\t\t\t\t\t\t--remove <xx:xx:xx:xx:xx:xx>\n"
				  "\t\t\t\t\t\t--flush --uc --mc\n";
	cmd_params.cmd_arg	= ppio;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_mac_addr;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "promisc";
	cmd_params.desc		= "set/get ppio promiscuous mode";
	cmd_params.format	= "--<on/off/get>\n";
	cmd_params.cmd_arg	= ppio;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_promisc_mode;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "multicast";
	cmd_params.desc		= "set/get ppio all multicast mode";
	cmd_params.format	= "--<on/off/get>\n";
	cmd_params.cmd_arg	= ppio;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_multicast_mode;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "vlan";
	cmd_params.desc		= "set/remove/flush ppio vlan filter";
	cmd_params.format	= "--set <vlan_id>\n"
				  "\t\t\t\t\t\t--remove <vlan_id>\n"
				  "\t\t\t\t\t\t--flush\n";
	cmd_params.cmd_arg	= ppio;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_vlan;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}
