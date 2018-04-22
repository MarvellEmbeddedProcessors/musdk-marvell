/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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

/**
 * @file neta.c
 *
 * PPDK container structures and packet processor initialization
 */

#define DEBUG
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

/* check interface name is NETA interface and it was congured
 * in kernel space by ifconfig command
 */
static int neta_netdev_if_verify(const char *if_name)
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
	while (fgets(buf, MAX_BUF_STR_LEN, fp)) {
		if (strncmp("DRIVER=mvneta", buf, 13) == 0) {
			found = 1;
			break;
		}
	}
	fclose(fp);
	if (!found) {
		pr_err("interface %s doesn't NETA port\n", if_name);
		return -EEXIST;
	}

	/* TBD: check this is MUSDK port */

	return 0;
}

int neta_port_register(const char *if_name, int port_id)
{
	struct netdev_if_params *port;
	int err;

	err = neta_netdev_if_verify(if_name);
	if (err)
		return -1;

	if (neta_ptr->ports[port_id])
		return -1;

	port = kcalloc(1, sizeof(struct netdev_if_params), GFP_KERNEL);
	if (unlikely(!port)) {
		pr_err("%s out of memory neta_port alloc\n", __func__);
			return -1;
	}
	strcpy(port->if_name, if_name);
	port->ppio_id = port_id;

	neta_ptr->ports[port_id] = port;

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
}

/* Find port_id parameters from ifname.
* Description: loop through all ports in packet processor and compare the interface name
* to the one configured for each port. If there is a match, the port_id are returned.
* This function should be called after neta_init() and before ppio_init().
 */
int neta_netdev_get_port_info(char *ifname, u8 *port_id)
{
	struct netdev_if_params *port;
	int i;

	if (!neta_is_initialized()) {
		pr_warn("NETA US driver not initialized\n");
		return -EINVAL;
	}

	for (i = 0; i < NETA_NUM_ETH_PPIO; i++) {
		if (!neta_ptr->ports[i])
			continue;

		port = neta_ptr->ports[i];

		if (strcmp(ifname, port->if_name) == 0) {
			*port_id = port->ppio_id;
			pr_info("%s: port %d\n", ifname, *port_id);
			return 0;
		}
	}
	return -EEXIST;
}
