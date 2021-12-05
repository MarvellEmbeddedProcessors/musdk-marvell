/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file neta.c
 *
 * PPDK container structures and packet processor initialization
 */

#ifndef DEBUG
#define DEBUG
#endif

#include "std_internal.h"
#include "drivers/mv_neta.h"
#include "drivers/mv_neta_ppio.h"
#include "neta_ppio.h"

#define NETA_NETDEV_PATH	"/sys/class/net/"
#define MAX_BUF_STR_LEN		256

struct netdev_if_params {
	char	if_name[16];
	u8	ppio_id;
};

/* Main control structure for the PP */
struct neta {

	/* Port objects associated */
	u32 num_ports;
	struct netdev_if_params *ports[NETA_NUM_ETH_PPIO];

};

static struct neta	*neta_ptr;

int neta_is_initialized(void)
{
	return (neta_ptr) ? 1 : 0;
}

int neta_init(void)
{
	if (neta_ptr) {
		pr_warn("NETA US driver already initialized\n");
		return -EINVAL;
	}
	neta_ptr = kcalloc(1, sizeof(struct neta), GFP_KERNEL);
	if (unlikely(!neta_ptr)) {
		pr_err("%s: out of memory for NETA driver allocation\n", __func__);
		return -ENOMEM;
	}
	neta_ptr->num_ports = NETA_NUM_ETH_PPIO;

	pr_debug("NETA PP run\n");

	return 0;
}

static int get_last_num_from_string(char *buf, int *id)
{
	int len = strlen(buf);
	int i, found = 0;

	for (i = len - 1; (i >= 0); i--) {
		if (isdigit(buf[i]))
			found++;
		else if (found)
			break;
	}

	if (found) {
		*id = atoi(&buf[i+1]);
		return 0;
	}

	return -1;
}

/* check interface name is NETA interface and it was congured
 * in kernel space by ifconfig command
 */
static int neta_netdev_if_verify(const char *if_name, int *port_id)
{
	FILE *fp;
	char path[MAX_BUF_STR_LEN];
	char buf[MAX_BUF_STR_LEN];
	int found;

	/* check if it is mvneta interface */
	sprintf(path, "%s%s/device/uevent", NETA_NETDEV_PATH, if_name);
	fp = fopen(path, "r");
	if (!fp) {
		pr_err("error opening %s\n", path);
		return -EEXIST;
	}
	found = 0;
	*port_id = 0xff;
	while (fgets(buf, MAX_BUF_STR_LEN, fp)) {
		if (strncmp("DRIVER=mvneta", buf, strlen("DRIVER=mvneta")) == 0) {
			found = 1;
			do {
				/* get HW port number */
				if (strncmp("OF_ALIAS_0=eth", buf,
					    strlen("OF_ALIAS_0=eth")) == 0) {
					get_last_num_from_string(buf, port_id);
					found++;
					break;
				}
			} while (fgets(buf, MAX_BUF_STR_LEN, fp));
			break;
		}
	}
	fclose(fp);
	if (found < 2) {
		pr_err("interface %s is not NETA port %d\n", if_name, *port_id);
		return -EEXIST;
	}

	return 0;
}

int neta_port_register(const char *if_name, int *port_id)
{
	struct netdev_if_params *port;
	int err;

	err = neta_netdev_if_verify(if_name, port_id);
	if (err)
		return -1;

	if (*port_id >= NETA_NUM_ETH_PPIO) {
		pr_err("[%s] Invalid ppio number %d.\n", __func__, *port_id);
		return -ENXIO;
	}

	if (neta_ptr->ports[*port_id])
		return -1;

	port = kcalloc(1, sizeof(struct netdev_if_params), GFP_KERNEL);
	if (unlikely(!port)) {
		pr_err("%s out of memory neta_port alloc\n", __func__);
			return -1;
	}
	strcpy(port->if_name, if_name);
	port->ppio_id = *port_id;

	neta_ptr->ports[*port_id] = port;

	return 0;
}

void neta_port_unregister(int port_id)
{
	neta_ptr->ports[port_id] = 0;
}

void neta_deinit(void)
{
	int i;

	if (!neta_is_initialized())
		return;

	for (i = 0; i < neta_ptr->num_ports; i++)
		kfree(neta_ptr->ports[i]);

	/* Destroy the PPDK handle */
	kfree(neta_ptr);

	neta_ptr = NULL;
}

/* Find port_id parameters from ifname.
* Description: loop through all ports in packet processor and compare the interface name
* to the one configured for each port. If there is a match, the port_id are returned.
* This function should be called after neta_init() and before ppio_init().
 */
int neta_netdev_get_port_info(char *ifname, u8 *port_id)
{
	int ret, id;

	ret = neta_netdev_if_verify(ifname, &id);
	if (!ret)
		*port_id = (u8)id;
	return ret;
}
