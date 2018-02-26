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

#include <string.h>
#include <stdio.h>
#include <getopt.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_dma.h"

#include "mvapp.h"
#include "mv_neta.h"
#include "neta_utils.h"

u64 neta_sys_dma_high_addr;

static u64 buf_alloc_cnt;
static u64 buf_free_cnt;
static u64 hw_rxq_buf_free_cnt;
static u64 hw_bm_buf_free_cnt;
static u64 hw_buf_free_cnt;
static u64 tx_shadow_q_buf_free_cnt[MVAPPS_NETA_MAX_NUM_CORES];

void app_show_port_stat(struct port_desc *port_desc, int reset)
{
	struct neta_ppio_statistics stats;

	printf("\n--------  Port %s MIB stats --------\n", port_desc->name);
	neta_ppio_get_statistics(port_desc->ppio, &stats);
	printf("\t Rx statistics:\n");
	printf("\t\tReceived bytes:          %" PRIu64 "\n", stats.rx_bytes);
	printf("\t\tReceived packets:        %" PRIu64 "\n", stats.rx_packets);
	printf("\t\tReceived error bytes:    %" PRIu64 "\n", stats.rx_bytes_err);
	printf("\t\tReceived error packets:  %" PRIu64 "\n", stats.rx_packets_err);
	printf("\t\tReceived broadcast:      %" PRIu64 "\n", stats.rx_broadcast_packets);
	printf("\t\tReceived multicast:      %" PRIu64 "\n", stats.rx_multicast_packets);
	printf("\t\tUndersize received:      %" PRIu64 "\n", stats.rx_undersize);
	printf("\t\tOversize received:       %" PRIu64 "\n", stats.rx_oversize);
	printf("\t\tFragments received:      %" PRIu64 "\n", stats.rx_fragments);
	printf("\t\tMAC errors received:     %" PRIu64 "\n", stats.rx_errors);
	printf("\t\tCRC errors received:     %" PRIu64 "\n", stats.rx_crc_error);
	printf("\t\tRX discard:              %" PRIu64 "\n", stats.rx_discard);
	printf("\t\tOverrun received:        %" PRIu64 "\n", stats.rx_overrun);
	printf("\n");
	printf("\t Tx statistics:\n");
	printf("\t\tSent bytes:              %" PRIu64 "\n", stats.tx_bytes);
	printf("\t\tSent packets:            %" PRIu64 "\n", stats.tx_packets);
	printf("\t\tSent broadcast:          %" PRIu64 "\n", stats.tx_broadcast_packets);
	printf("\t\tSent multicast:          %" PRIu64 "\n", stats.tx_multicast_packets);
	printf("\t\tSent errors:             %" PRIu64 "\n", stats.tx_errors);
}

static int port_stat_cmd_cb(void *arg, int argc, char *argv[])
{
	struct glb_common_args *g_cmn_args = (struct glb_common_args *)arg;
	struct port_desc *ports = (struct port_desc *)g_cmn_args->plat;
	struct neta_ppio *ppio = NULL;
	int i;

	if (argc < 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n",
		       __func__, argc);
		return -EINVAL;
	}

	for (i = 0; i < g_cmn_args->num_ports; i++) {
		if (!strcmp(ports[i].name, argv[1])) {
			ppio = ports[i].ppio;
			break;
		}
	}
	if (!ppio) {
		pr_err("%s: unknown interface %s\n", __func__, argv[1]);
		return -EINVAL;
	}
	app_show_port_stat(&ports[i], 0);

	return 0;
}

static int port_link_status_cmd_cb(void *arg, int argc, char *argv[])
{
	struct glb_common_args *g_cmn_args = (struct glb_common_args *)arg;
	struct port_desc *ports = (struct port_desc *)g_cmn_args->plat;
	struct neta_ppio *ppio = NULL;
	int i, state;

	if (argc < 2) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n",
		       __func__, argc);
		return -EINVAL;
	}

	for (i = 0; i < g_cmn_args->num_ports; i++) {
		if (!strcmp(ports[i].name, argv[1])) {
			ppio = ports[i].ppio;
			break;
		}
	}
	if (!ppio) {
		pr_err("%s: unknown interface %s\n", __func__, argv[1]);
		return -EINVAL;
	}
	neta_ppio_get_link_state(ppio, &state);
	pr_info("%s link is %s\n", argv[1], (state) ? "UP" : "DOWN");

	return 0;
}

/*
 * set mac address example
 */
static int neta_cli_mac_addr_set(void *arg, int argc, char *argv[])
{
	struct glb_common_args *g_cmn_args = (struct glb_common_args *)arg;
	struct port_desc *ports = (struct port_desc *)g_cmn_args->plat;
	struct neta_ppio *ppio = NULL;
	int rc;
	u8 mac[ETH_ALEN];
	int i;

	if (argc != 3) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}
	for (i = 0; i < g_cmn_args->num_ports; i++) {
		if (!strcmp(ports[i].name, argv[1])) {
			ppio = ports[i].ppio;
			break;
		}
	}
	if (!ppio) {
		pr_err("%s: unknown interface %s\n", __func__, argv[1]);
		return -EINVAL;
	}

	rc = app_parse_mac_address(argv[2], mac);
	if (rc) {
		pr_err("parsing fail, wrong input for mac_address set\n");
		return -EINVAL;
	}
	printf("mac_addr set %02x:%02x:%2x:%2x:%2x:%2x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	rc = neta_ppio_set_mac_addr(ppio, mac);
	if (rc) {
		pr_err("Unable to set mac address\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * add mac address example
 */
static int neta_cli_mac_addr_add(void *arg, int argc, char *argv[])
{
	struct glb_common_args *g_cmn_args = (struct glb_common_args *)arg;
	struct port_desc *ports = (struct port_desc *)g_cmn_args->plat;
	struct neta_ppio *ppio = NULL;
	int rc, i;
	u8 mac[ETH_ALEN];

	if (argc != 3) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	for (i = 0; i < g_cmn_args->num_ports; i++) {
		if (!strcmp(ports[i].name, argv[1])) {
			ppio = ports[i].ppio;
			break;
		}
	}
	if (!ppio) {
		pr_err("%s: unknown interface %s\n", __func__, argv[1]);
		return -EINVAL;
	}

	rc = app_parse_mac_address(argv[2], mac);
	if (rc) {
		printf("parsing fail, wrong input for mac_address set\n");
		return -EINVAL;
	}

	printf("mac_addr set %2x:%2x:%2x:%2x:%2x:%2x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

	rc = neta_ppio_add_mac_addr(ppio, mac);
	if (rc) {
		printf("Unable to set mac address\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * enable or disable promisc mode
 */
static int neta_cli_promisc_state_set(void *arg, int argc, char *argv[])
{
	struct glb_common_args *g_cmn_args = (struct glb_common_args *)arg;
	struct port_desc *ports = (struct port_desc *)g_cmn_args->plat;
	struct neta_ppio *ppio = NULL;
	int en, rc;
	char *ret_ptr;
	int i;

	if (argc != 3) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	for (i = 0; i < g_cmn_args->num_ports; i++) {
		if (!strcmp(ports[i].name, argv[1])) {
			ppio = ports[i].ppio;
			break;
		}
	}
	if (!ppio) {
		pr_err("%s: unknown interface %s\n", __func__, argv[1]);
		return -EINVAL;
	}

	en = strtoul(argv[2], &ret_ptr, 0);
	if (en != 0 && en != 1) {
		printf("wrong value for enable = %d\n", en);
		return -EINVAL;
	}
	rc = neta_ppio_set_promisc(ppio, en);
	if (rc) {
		printf("Unable to enable unicast promiscuous mode\n");
		return -rc;
	}

	return 0;
}

int register_cli_ftr_cmds(struct glb_common_args *garg)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "mac_addr_set";
	cmd_params.desc		= "Set ppio MAC address\n";
	cmd_params.format	= "<if_name> <xx:xx:xx:xx:xx:xx>";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))neta_cli_mac_addr_set;
	mvapp_register_cli_cmd(&cmd_params);

	cmd_params.name		= "mac_addr_add";
	cmd_params.desc		= "Add ppio MAC address\n";
	cmd_params.format	= "<if_name> <xx:xx:xx:xx:xx:xx>";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))neta_cli_mac_addr_add;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "promisc_state";
	cmd_params.desc		= "Enable/disable promiscuous mode\n";
	cmd_params.format	= "<if_name> 0 - for disable, 1 - for enable";
	cmd_params.cmd_arg	= garg;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))neta_cli_promisc_state_set;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

int app_register_cli_common_cmds(struct glb_common_args *glb_args)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pstat";
	cmd_params.desc		= "Show port MAC statistics\n";
	cmd_params.format	= "<if_name>";
	cmd_params.cmd_arg	= glb_args;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))port_stat_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "plink";
	cmd_params.desc		= "Show port link status\n";
	cmd_params.format	= "<if_name>";
	cmd_params.cmd_arg	= glb_args;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))port_link_status_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

int app_find_port_info(struct port_desc *port_desc)
{
	/* TBD: get port info */
	return 0;
}

int app_port_init(struct port_desc *port, u16 mtu, u16 pkt_offset)
{
	struct neta_ppio_params		*port_params = &port->port_params;
	char				name[MVAPPS_PPIO_NAME_MAX];
	int				i, err = 0;

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "neta-%d", port->ppio_id);
	pr_debug("found port: %s\n", name);
	port_params->match = name;
	port_params->inqs_params.num_tcs = port->num_tcs;
	port_params->inqs_params.mtu = mtu;

	for (i = 0; i < port->num_tcs; i++) {
		port_params->inqs_params.tcs_params[i].pkt_offset =
			(pkt_offset) ? pkt_offset : MVAPPS_NETA_PKT_OFFS;
		port_params->inqs_params.tcs_params[i].size = port->inq_size;

	}
	port_params->outqs_params.num_outqs = port->num_outqs;
	for (i = 0; i < port->num_outqs; i++) {
		port_params->outqs_params.outqs_params[i].size = port->outq_size;
		port_params->outqs_params.outqs_params[i].weight = 1;
	}

	err = neta_ppio_init(port_params, &port->ppio);
	if (err) {
		pr_err("PP-IO init failed (error: %d)!\n", err);
		return err;
	}

	if (!port->ppio) {
		pr_err("PP-IO init failed!\n");
		return -EIO;
	}
	return err;
}

int app_port_enable(struct port_desc *port)
{
	int err;

	err = neta_ppio_enable(port->ppio);
	port->initialized = 1;

	return err;
}

void app_port_local_init(int id, int lcl_id, struct lcl_port_desc *lcl_port, struct port_desc *port)
{
	int i;

	lcl_port->lcl_id	= lcl_id;
	lcl_port->port_id	= port->ppio_id;
	lcl_port->ppio		= port->ppio;

	lcl_port->num_shadow_qs = port->num_outqs;
	lcl_port->shadow_q_size	= port->outq_size * 2; /* Shadow queue must be larger than TXQ at least on burst size */
	lcl_port->shadow_qs = (struct tx_shadow_q *)malloc(port->num_outqs * sizeof(struct tx_shadow_q));

	for (i = 0; i < lcl_port->num_shadow_qs; i++) {
		lcl_port->shadow_qs[i].read_ind = 0;
		lcl_port->shadow_qs[i].write_ind = 0;

		lcl_port->shadow_qs[i].ents =
			(struct tx_shadow_q_entry *)malloc(lcl_port->shadow_q_size * sizeof(struct tx_shadow_q_entry));
	}
}

void app_port_local_deinit(struct lcl_port_desc *lcl_port)
{
	int i, cnt = 0;

	for (i = 0; i < lcl_port->num_shadow_qs; i++) {
		struct tx_shadow_q *shadow_q = &lcl_port->shadow_qs[i];

		if (shadow_q->read_ind > shadow_q->write_ind) {
			cnt = lcl_port->shadow_q_size - shadow_q->read_ind + shadow_q->write_ind;
			tx_shadow_q_buf_free_cnt[lcl_port->lcl_id] += cnt;
		} else {
			cnt = shadow_q->write_ind - shadow_q->read_ind;
			tx_shadow_q_buf_free_cnt[lcl_port->lcl_id] += cnt;
		}
		pr_debug("Release %d buffers from shadow_q-%d port:%d\n",
			 cnt, i, lcl_port->port_id);
		shadow_q->read_ind = 0;
		shadow_q->write_ind = 0;
		free(shadow_q->ents);
	}

	free(lcl_port->shadow_qs);
}

static void free_rx_queues(struct neta_ppio *port, u16 num_tcs, u16 num_inqs[])
{
	struct neta_ppio_desc	descs[MVAPPS_MAX_BURST_SIZE];
	u8			tc = 0, qid = 0;
	u16			num;

	for (tc = 0; tc < num_tcs; tc++) {
		for (qid = 0; qid < num_inqs[tc]; qid++) {
			num = MVAPPS_MAX_BURST_SIZE;
			while (num) {
				neta_ppio_recv(port, qid, descs, &num);
				hw_rxq_buf_free_cnt += num;
			}
		}
	}
}

void app_disable_all_ports(struct port_desc *ports, int num_ports)
{
	int i;

	for (i = 0;  i < num_ports; i++) {
		if (ports[i].ppio) {
			neta_ppio_disable(ports[i].ppio);
			free_rx_queues(ports[i].ppio, ports[i].num_tcs, ports[i].num_inqs);
		}
	}
}

void app_deinit_all_ports(struct port_desc *ports, int num_ports)
{
	int i;

	for (i = 0;  i < num_ports; i++) {
		if (ports[i].ppio)
			neta_ppio_deinit(ports[i].ppio);
	}

	/* Calculate number of buffers released from PPIO */
	for (i = 0; i < MVAPPS_NETA_MAX_NUM_CORES; i++)
		hw_buf_free_cnt += tx_shadow_q_buf_free_cnt[i];
	hw_buf_free_cnt += hw_bm_buf_free_cnt + hw_rxq_buf_free_cnt;

	if (buf_free_cnt != buf_alloc_cnt) {
		pr_err("Not all buffers were released: allocated: %lu, freed: %lu\n",
		       buf_alloc_cnt, buf_free_cnt);

		if (buf_free_cnt != hw_buf_free_cnt) {
			pr_err("neta freed: %lu bm free: %lu, rxq free: %lu, tx free: %lu !\n",
			       hw_buf_free_cnt, hw_bm_buf_free_cnt, hw_rxq_buf_free_cnt,
				(hw_buf_free_cnt - hw_bm_buf_free_cnt - hw_rxq_buf_free_cnt));
		}
	}

	pr_debug("allocated: %lu, app freed: %lu, bm free: %lu, rxq free: %lu, tx free: %lu !\n",
		 buf_alloc_cnt, buf_free_cnt, hw_bm_buf_free_cnt, hw_rxq_buf_free_cnt,
		(hw_buf_free_cnt - hw_bm_buf_free_cnt - hw_rxq_buf_free_cnt));
}
