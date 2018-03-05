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

#ifndef DEBUG
#define DEBUG
#endif

#include "std_internal.h"
#include "drivers/mv_neta.h"
#include "drivers/mv_neta_ppio.h"
#include "neta_ppio.h"


struct neta_uio {
	struct uio_info_t *uio_info;
	struct uio_mem_t *mem;
};

/* Main control structure for the PP */
struct neta {
	struct neta_init_params init;

	/* Port objects associated */
	u32 num_ports;
	struct neta_ppio *ports[NETA_NUM_ETH_PPIO];

	/* BM Pools associated */
	/*struct pp2_bm_pool *bm_pools[PP2_BPOOL_NUM_POOLS];*/
};

static struct neta	*neta_ptr;

int neta_is_initialized(void)
{
	return (neta_ptr) ? 1 : 0;
}

int neta_init(struct neta_init_params *params)
{
	int i;

	if (neta_ptr) {
		pr_warn("NETA US driver already initialized\n");
		return -EINVAL;
	}
	neta_ptr = kcalloc(1, sizeof(struct neta), GFP_KERNEL);
	if (unlikely(!neta_ptr)) {
		pr_err("%s: out of memory for NETA driver allocation\n", __func__);
		return -ENOMEM;
	}
	memcpy(&neta_ptr->init, params, sizeof(*params));

	/* TBD: do it only US ports */
	neta_ptr->num_ports = NETA_NUM_ETH_PPIO;
	for (i = 0; i < neta_ptr->num_ports; i++) {
		struct neta_ppio *port = kcalloc(1, sizeof(struct neta_ppio), GFP_KERNEL);

		if (unlikely(!port)) {
			pr_err("%s out of memory neta_port alloc\n", __func__);
			break;
		}
		neta_ptr->ports[i] = port;
	}

	pr_debug("NETA PP run\n");

	return 0;
}

void neta_deinit(void)
{
	int i;

	if (!neta_is_initialized())
		return;

	for (i = 0; i < neta_ptr->num_ports; i++) {
		struct neta_ppio *port = neta_ptr->ports[i];

		neta_ppio_deinit(port);
	}

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
	struct neta_port *port;
	int i;

	if (!neta_is_initialized()) {
		pr_warn("NETA US driver not initialized\n");
		return -EINVAL;
	}

	for (i = 0; i < NETA_NUM_ETH_PPIO; i++) {
		if (!neta_ptr->ports[i])
			continue;

		port = (struct neta_port *)neta_ptr->ports[i]->internal_param;

		if (strcmp(ifname, port->if_name) == 0) {
			*port_id = i;
			pr_info("%s: port %d\n", ifname, *port_id);
			return 0;
		}
	}
	return -EEXIST;
}
