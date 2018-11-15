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
}

/* Find port_id parameters from ifname.
* Description: loop through all ports in packet processor and compare the interface name
* to the one configured for each port. If there is a match, the port_id are returned.
* This function should be called after neta_init() and before ppio_init().
 */
int neta_netdev_get_port_info(char *ifname, u8 *port_id)
{
	int ret, id;

	if (!neta_is_initialized()) {
		pr_warn("NETA US driver not initialized\n");
		return -EINVAL;
	}

	ret = neta_netdev_if_verify(ifname, &id);
	if (!ret)
		*port_id = (u8)id;
	return ret;
}
