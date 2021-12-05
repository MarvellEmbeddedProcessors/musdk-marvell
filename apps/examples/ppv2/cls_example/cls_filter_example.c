/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdio.h>
#include <getopt.h>
#include "mvapp.h"
#include "utils.h"
#include "cls_main_example.h"
#include "src/drivers/ppv2/cls/pp2_cls_utils.h"

/*
 * pp2_cls_cli_mac_addr_set()
 * set mac address example
 */
static int pp2_cls_cli_mac_addr_set(void *arg, int argc, char *argv[])
{
	struct pp2_ppio *ppio = (struct pp2_ppio *)arg;
	int rc;
	u8 mac[ETH_ALEN];

	if (argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	rc = mv_pp2x_parse_mac_address(argv[1], mac);
	if (rc) {
		printf("parsing fail, wrong input for mac_address set\n");
		return -EINVAL;
	}
	printf("mac_addr set %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	rc = pp2_ppio_set_mac_addr(ppio, mac);
	if (rc) {
		printf("Unable to set mac address\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * pp2_cls_cli_mac_addr_add()
 * add mac address example
 */
static int pp2_cls_cli_mac_addr_add(void *arg, int argc, char *argv[])
{
	struct pp2_ppio *ppio = (struct pp2_ppio *)arg;
	int rc;
	u8 mac[ETH_ALEN];

	if (argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	rc = mv_pp2x_parse_mac_address(argv[1], mac);
	if (rc) {
		printf("parsing fail, wrong input for mac_address set\n");
		return -EINVAL;
	}

	printf("mac_addr set %x:%x:%x:%x:%x:%x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	rc = pp2_ppio_add_mac_addr(ppio, mac);
	if (rc) {
		printf("Unable to set mac address\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * pp2_cls_cli_promisc_state_set()
 * enable or disable promisc mode
 */
static int pp2_cls_cli_promisc_state_set(void *arg, int argc, char *argv[])
{
	struct pp2_ppio *ppio = (struct pp2_ppio *)arg;
	int rc;
	int en;
	char *ret_ptr;

	if  (argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	en = strtoul(argv[1], &ret_ptr, 0);
	if (en != 0 && en != 1) {
		printf("wrong value for enable = %d\n", en);
		return -EINVAL;
	}
	rc = pp2_ppio_set_promisc(ppio, en);
	if (rc) {
		printf("Unable to enable unicast promiscuous mode\n");
		return -rc;
	}

	return 0;
}

/*
 * pp2_cls_cli_set_vlan()
 * add vlan id
 */
static int pp2_cls_cli_set_vlan(void *arg, int argc, char *argv[])
{
	struct pp2_ppio *ppio = (struct pp2_ppio *)arg;
	int rc;
	char *ret_ptr;
	u16 vlan_id;

	if  (argc != 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	vlan_id = strtoul(argv[1], &ret_ptr, 0);

	rc = pp2_ppio_add_vlan(ppio, vlan_id);
	if (rc) {
		printf("Unable to add vlan id %d\n", vlan_id);
		return -EINVAL;
	}

	return 0;
}

int register_cli_filter_cmds(struct pp2_ppio *ppio)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pp2_cls_cli_mac_addr_set";
	cmd_params.desc		= "set ppio MAC address";
	cmd_params.format	= "<xx:xx:xx:xx:xx:xx>\n";
	cmd_params.cmd_arg	= ppio;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_mac_addr_set;
	mvapp_register_cli_cmd(&cmd_params);

	cmd_params.name		= "pp2_cls_cli_mac_addr_add";
	cmd_params.desc		= "add ppio MAC address";
	cmd_params.format	= "<xx:xx:xx:xx:xx:xx>\n";
	cmd_params.cmd_arg	= ppio;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_mac_addr_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "promisc_state";
	cmd_params.desc		= "enable/disable promiscuous mode";
	cmd_params.format	= "0 - for disable\n"
				  "\t\t\t\t1 - for enable\n";
	cmd_params.cmd_arg	= ppio;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_promisc_state_set;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "vlan_set";
	cmd_params.desc		= "set ppio vlan filter";
	cmd_params.format	= "<vlan_id>\n";
	cmd_params.cmd_arg	= ppio;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_cls_cli_set_vlan;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}
