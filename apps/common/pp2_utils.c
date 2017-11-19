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
#include <limits.h>

#include "mv_std.h"
#include "lib/lib_misc.h"
#include "env/mv_sys_dma.h"

#include "mvapp.h"
#include "mv_pp2.h"
#include "mv_pp2_bpool.h"
#include "pp2_utils.h"
#include "src/drivers/ppv2/pp2.h"

static u16 used_bpools[MVAPPS_PP2_MAX_PKT_PROC] = {MVAPPS_PP2_BPOOLS_RSRV, MVAPPS_PP2_BPOOLS_RSRV};
static u16 used_hifs = MVAPPS_PP2_HIFS_RSRV;

static u64 buf_alloc_cnt;
static u64 buf_free_cnt;
static u64 hw_rxq_buf_free_cnt;
static u64 hw_bm_buf_free_cnt;
static u64 hw_buf_free_cnt;
static u64 tx_shadow_q_buf_free_cnt[MVAPPS_MAX_NUM_CORES];

void app_show_rx_queue_stat(struct port_desc *port_desc, u8 tc, u8 q_start, int num_qs, int reset)
{
	int i;
	u8 q_stop;
	struct pp2_ppio_inq_statistics rxstats;

	if (tc > port_desc->num_tcs) {
		printf("\nWrong tc parameters: tc=%d, num_tcs=%d", tc, port_desc->num_tcs);
		return;
	}

	if (q_start + num_qs >= port_desc->num_inqs[tc]) {
		printf("\nWrong queue parameters: first_qid=%d, num=%d", q_start, num_qs);
		return;
	}

	if (num_qs)
		q_stop = num_qs + q_start;
	else
		q_stop = port_desc->num_inqs[tc] - q_start;

	for (i = q_start; i < q_stop; i++) {
		pp2_ppio_inq_get_statistics(port_desc->ppio, tc, i, &rxstats, reset);
		printf("\tPort %d:%d Tc #%d Rxq #%d statistics:\n", port_desc->pp_id, port_desc->ppio_id, tc, i);
		printf("\t\tEnqueued packets:      %" PRIu64 "\n", rxstats.enq_desc);
		printf("\t\tFull queue drops:      %u\n", rxstats.drop_fullq);
		printf("\t\tBuffer Manager drops:  %u\n", rxstats.drop_bm);
		printf("\t\tEarly drops:           %u\n", rxstats.drop_early);
	}
	printf("\n");
}

void app_show_tx_queue_stat(struct port_desc *port_desc, int reset)
{
	int i;
	struct pp2_ppio_outq_statistics txstats;

	for (i = 0; i < port_desc->num_outqs; i++) {
		printf("\tPort %d:%d Txq #%d statistics:\n", port_desc->pp_id, port_desc->ppio_id, i);
		pp2_ppio_outq_get_statistics(port_desc->ppio, i, &txstats, reset);
		printf("\t\tEnqueued packets:      %" PRIu64 "\n", txstats.enq_desc);
		printf("\t\tDequeued packets:      %" PRIu64 "\n", txstats.deq_desc);
		printf("\t\tEnque desc to DDR:     %" PRIu64 "\n", txstats.enq_dec_to_ddr);
		printf("\t\tEnque buffers to DDR:  %" PRIu64 "\n", txstats.enq_buf_to_ddr);
		printf("\n");
	}
}

void app_show_port_stat(struct port_desc *port_desc, int reset)
{
	struct pp2_ppio_statistics stats;

	printf("\n--------  Port %d:%d stats --------\n", port_desc->pp_id, port_desc->ppio_id);
	pp2_ppio_get_statistics(port_desc->ppio, &stats, reset);
	printf("\t Rx statistics:\n");
	printf("\t\tReceived bytes:          %" PRIu64 "\n", stats.rx_bytes);
	printf("\t\tReceived packets:        %" PRIu64 "\n", stats.rx_packets);
	printf("\t\tReceived unicast:        %" PRIu64 "\n", stats.rx_unicast_packets);
	printf("\t\tReceived errors:         %" PRIu64 "\n", stats.rx_errors);
	printf("\t\tFull queue drops:        %" PRIu64 "\n", stats.rx_fullq_dropped);
	printf("\t\tBuffer Manager drops:    %u\n", stats.rx_bm_dropped);
	printf("\t\tEarly drops:             %u\n", stats.rx_early_dropped);
	printf("\t\tFIFO overrun drops:      %u\n", stats.rx_fifo_dropped);
	printf("\t\tClassifier drops:        %u\n", stats.rx_cls_dropped);
	printf("\n");
	printf("\t Tx statistics:\n");
	printf("\t\tSent bytes:              %" PRIu64 "\n", stats.tx_bytes);
	printf("\t\tSent packets:            %" PRIu64 "\n", stats.tx_packets);
	printf("\t\tSent unicast:            %" PRIu64 "\n", stats.tx_unicast_packets);
	printf("\t\tSent errors:             %" PRIu64 "\n", stats.tx_errors);
}

void app_show_port_eth_tool_get(struct port_desc *port_desc)
{
	char buf[50] = "ethtool ";

	strcat(buf, port_desc->name);
	system(buf);
}

void app_set_port_enable(struct port_desc *port_desc, int enable)
{
	if (enable)
		pp2_ppio_enable(port_desc->ppio);
	else
		pp2_ppio_disable(port_desc->ppio);
}

static int queue_stat_cmd_cb(void *arg, int argc, char *argv[])
{
	int i, j, reset = 0;
	u8 qid = 0, port_id = 0, tc = (~0);
	struct glb_common_args *glb_args = (struct glb_common_args *) arg;
	struct port_desc *port_desc = ((struct pp2_glb_common_args *) glb_args->plat)->ports_desc;
	int ports_num = glb_args->num_ports;
	int queues_num = 0;
	char *ret_ptr;
	int option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"port", required_argument, 0, 'p'},
		{"tc", required_argument, 0, 't'},
		{"queue", required_argument, 0, 'q'},
		{"reset", no_argument, 0, 'r'},
		{0, 0, 0, 0}
	};

	if (argc > 6) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		switch (option) {
		case 'p':
			port_id = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (port_id < 0)) {
				printf("parsing fail, wrong input for --port\n");
				return -EINVAL;
			}
			ports_num = 1;
			break;
		case 't':
			tc = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (tc < 0)) {
				printf("parsing fail, wrong input for --tc\n");
				return -EINVAL;
			}
			break;
		case 'q':
			qid = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (qid < 0)) {
				printf("parsing fail, wrong input for --queue\n");
				return -EINVAL;
			}
			queues_num = 1;
			break;
		case 'r':
			reset = 1;
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	for (i = port_id; i < (port_id + ports_num); i++) {
		if (port_desc[i].initialized) {
			if (tc == (u8)(~0)) {
				for (j = 0; j < port_desc->num_tcs; j++)
					app_show_rx_queue_stat(&port_desc[i], j, qid, queues_num, reset);
			} else {
				app_show_rx_queue_stat(&port_desc[i], tc, qid, queues_num, reset);
			}
			app_show_tx_queue_stat(&port_desc[i], reset);
		}
	}

	return 0;
}

static int port_stat_cmd_cb(void *arg, int argc, char *argv[])
{
	int i, reset = 0;
	u8 portid = 0;
	struct glb_common_args *glb_args = (struct glb_common_args *) arg;
	struct port_desc *port_desc = ((struct pp2_glb_common_args *) glb_args->plat)->ports_desc;
	int ports_num = glb_args->num_ports;
	char *ret_ptr;
	int option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"port", required_argument, 0, 'p'},
		{"reset", no_argument, 0, 'r'},
		{0, 0, 0, 0}
	};

	if (argc > 4) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		switch (option) {
		case 'p':
			portid = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (portid < 0)) {
				printf("parsing fail, wrong input for --port\n");
				return -EINVAL;
			}
			ports_num = 1;
			break;
		case 'r':
			reset = 1;
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	for (i = portid; i < (portid + ports_num); i++) {
		if (port_desc[i].initialized)
			app_show_port_stat(&port_desc[i], reset);
	}

	return 0;
}

static int port_enable_cmd_cb(void *arg, int argc, char *argv[])
{
	int enable = 0, portid = 0, port_set = 0;
	struct port_desc *port_desc = (struct port_desc *)arg;
	char *ret_ptr;
	int option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"port", required_argument, 0, 'p'},
		{"enable", required_argument, 0, 'e'},
		{0, 0, 0, 0}
	};

	if (argc > 5) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		switch (option) {
		case 'p':
			portid = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (portid < 0)) {
				printf("parsing fail, wrong input for --port\n");
				return -EINVAL;
			}
			port_set = 1;
			break;
		case 'e':
			enable = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (enable < 0) || (enable > 1)) {
				printf("parsing fail, wrong input for --enable\n");
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}
	if (!port_set) {
		pr_err("port option must be set for %s\n", __func__);
		return -EINVAL;
	}
	app_set_port_enable(&port_desc[portid], enable);

	return 0;
}

static int txqueue_enable_cmd_cb(void *arg, int argc, char *argv[])
{
	int enable = -1, portid = 0, qid = 0, en;
	struct glb_common_args *glb_args = (struct glb_common_args *) arg;
	struct port_desc *port_desc = ((struct pp2_glb_common_args *) glb_args->plat)->ports_desc;
	int ports_num = glb_args->num_ports;
	int queues_num = 0, num;
	char *ret_ptr;
	int option = 0, i, j;
	int long_index = 0;
	struct option long_options[] = {
		{"port", required_argument, 0, 'p'},
		{"queue", required_argument, 0, 'q'},
		{"enable", required_argument, 0, 'e'},
		{0, 0, 0, 0}
	};

	if (argc > 7) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		switch (option) {
		case 'p':
			portid = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (portid < 0)) {
				printf("parsing fail, wrong input for --port\n");
				return -EINVAL;
			}
			ports_num = 1;
			break;
		case 'q':
			qid = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (qid < 0)) {
				printf("parsing fail, wrong input for --tc\n");
				return -EINVAL;
			}
			queues_num = 1;
			break;
		case 'e':
			enable = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (enable < 0) || (enable > 1)) {
				printf("parsing fail, wrong input for --enable\n");
				return -EINVAL;
			}
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	if (portid == -1 || qid == -1) {
		pr_err("Invalid port or queue id: portid: %d, qid: %d\n", portid, qid);
		return -EINVAL;
	}

	for (i = portid; i < (portid + ports_num); i++) {
		num = (queues_num) ? queues_num : port_desc[portid].num_outqs;
		for (j = qid; j < qid + num; j++) {
			if (enable != (-1)) {
				pp2_ppio_set_outq_state(port_desc[i].ppio, j, enable);
				printf("%s queue %d of port %d\n", (enable) ? "Enable" : "Disable", j, i);
			}
			pp2_ppio_get_outq_state(port_desc[i].ppio, j, &en);
			printf("Queue %d of port %d is %s\n", j, i, (en) ? "enabled" : "disabled");
		}
	}

	return 0;

}

static int port_ethtool_get_cmd_cb(void *arg, int argc, char *argv[])
{
	int i;
	u8 portid = 0;
	struct glb_common_args *glb_args = (struct glb_common_args *) arg;
	struct port_desc *port_desc = ((struct pp2_glb_common_args *) glb_args->plat)->ports_desc;
	int ports_num = glb_args->num_ports;
	char *ret_ptr;
	int option = 0;
	int long_index = 0;
	struct option long_options[] = {
		{"port", required_argument, 0, 'p'},
		{0, 0, 0, 0}
	};

	if (argc > 3) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	/* Get parameters */
	while ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1) {
		switch (option) {
		case 'p':
			portid = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (portid < 0)) {
				printf("parsing fail, wrong input for --port\n");
				return -EINVAL;
			}
			ports_num = 1;
			break;
		default:
			printf("parsing fail, wrong input, line = %d\n", __LINE__);
			return -EINVAL;
		}
	}

	for (i = portid; i < (portid + ports_num); i++) {
		if (port_desc[i].initialized)
			app_show_port_eth_tool_get(&port_desc[i]);
	}

	return 0;
}

static char *pp2_desc_l2_info_str(unsigned int l2_info)
{
	switch (l2_info << MVPP2_PRS_RI_L2_CAST_OFFS) {
	case MVPP2_PRS_RI_L2_UCAST:
		return "Ucast";
	case MVPP2_PRS_RI_L2_MCAST:
		return "Mcast";
	case MVPP2_PRS_RI_L2_BCAST:
		return "Bcast";
	default:
		return "Unknown";
	}
	return NULL;
}

static char *pp2_desc_vlan_info_str(unsigned int vlan_info)
{
	switch (vlan_info << MVPP2_PRS_RI_VLAN_OFFS) {
	case MVPP2_PRS_RI_VLAN_NONE:
		return "None";
	case MVPP2_PRS_RI_VLAN_SINGLE:
		return "Single";
	case MVPP2_PRS_RI_VLAN_DOUBLE:
		return "Double";
	case MVPP2_PRS_RI_VLAN_TRIPLE:
		return "Triple";
	default:
		return "Unknown";
	}
	return NULL;
}

static void pp2_rx_descriptor_dump(struct port_desc *ports_desc)
{
	int i, j;
	struct mv_pp2x_rx_desc *desc = NULL;

	for (j = 0; j < MVAPPS_MAX_NUM_CORES; j++) {
		if (!ports_desc->lcl_ports_desc[j])
			continue;

		if (ports_desc->lcl_ports_desc[j]->save_desc_data) {
			desc = (struct mv_pp2x_rx_desc *)&ports_desc->lcl_ports_desc[j]->last_rx_descr_data.cmds[0];
			pr_info("Core %d - RX desc - %p: ", j, (void *)desc);
			for (i = 0; i < PP2_PPIO_DESC_NUM_WORDS; i++)
				pr_info("%8.8x ", ports_desc->lcl_ports_desc[j]->last_rx_descr_data.cmds[i]);
			pr_info("\n");

			pr_info("pkt_size=%d, L3_offs=%d, IP_hlen=%d, ",
			       desc->data_size,
			       (desc->status & MVPP2_RXD_L3_OFFSET_MASK) >> MVPP2_RXD_L3_OFFSET_OFFS,
			       (desc->status & MVPP2_RXD_IP_HLEN_MASK) >> MVPP2_RXD_IP_HLEN_OFFS);

			pr_info("L2=%s, ",
				pp2_desc_l2_info_str((desc->rsrvd_parser & MVPP2_RXD_L2_CAST_MASK) >>
						     MVPP2_RXD_L2_CAST_OFFS));

			pr_info("VLAN=");
			pr_info("%s, ",
				pp2_desc_vlan_info_str((desc->rsrvd_parser & MVPP2_RXD_VLAN_INFO_MASK) >>
							MVPP2_RXD_VLAN_INFO_OFFS));

			pr_info("L3=");
			if (MVPP2_RXD_L3_IS_IP4(desc->status))
				pr_info("IPv4 (hdr=%s), ", MVPP2_RXD_IP4_HDR_ERR(desc->status) ? "bad" : "ok");
			else if (MVPP2_RXD_L3_IS_IP4_OPT(desc->status))
				pr_info("IPv4 Options (hdr=%s), ", MVPP2_RXD_IP4_HDR_ERR(desc->status) ? "bad" : "ok");
			else if (MVPP2_RXD_L3_IS_IP4_OTHER(desc->status))
				pr_info("IPv4 Other (hdr=%s), ", MVPP2_RXD_IP4_HDR_ERR(desc->status) ? "bad" : "ok");
			else if (MVPP2_RXD_L3_IS_IP6(desc->status))
				pr_info("IPv6, ");
			else if (MVPP2_RXD_L3_IS_IP6_EXT(desc->status))
				pr_info("IPv6 Ext, ");
			else
				pr_info("Unknown, ");

			if (desc->status & MVPP2_RXD_IP_FRAG_MASK)
				pr_info("Frag, ");

			printk("L4=");
			if (MVPP2_RXD_L4_IS_TCP(desc->status))
				pr_info("TCP (csum=%s)", (desc->status & MVPP2_RXD_L4_CHK_OK_MASK) ? "Ok" : "Bad");
			else if (MVPP2_RXD_L4_IS_UDP(desc->status))
				pr_info("UDP (csum=%s)", (desc->status & MVPP2_RXD_L4_CHK_OK_MASK) ? "Ok" : "Bad");
			else
				pr_info("Unknown");

			pr_info("\n");

			pr_info("Lookup_ID=0x%x, cpu_code=0x%x\n",
				(desc->rsrvd_parser & MVPP2_RXD_LKP_ID_MASK) >> MVPP2_RXD_LKP_ID_OFFS,
				(desc->rsrvd_parser & MVPP2_RXD_CPU_CODE_MASK) >> MVPP2_RXD_CPU_CODE_OFFS);
		} else {
			pr_info("saving descriptor data is disabled\n");
		}
	}
}

static void pp2_tx_descriptor_dump(struct port_desc *ports_desc)
{
	int i, j;
	struct mv_pp2x_tx_desc *desc = NULL;

	for (j = 0; j < MVAPPS_MAX_NUM_CORES; j++) {
		if (!ports_desc->lcl_ports_desc[j])
			continue;

		if (ports_desc->lcl_ports_desc[j]->save_desc_data) {
			desc = (struct mv_pp2x_tx_desc *)&ports_desc->lcl_ports_desc[j]->last_tx_descr_data.cmds[0];

			pr_info("TX desc - %p: ", (void *)desc);
			for (i = 0; i < PP2_PPIO_DESC_NUM_WORDS; i++)
				pr_info("%8.8x ", ports_desc->lcl_ports_desc[j]->last_tx_descr_data.cmds[i]);
			pr_info("\n");
		} else {
			pr_info("saving descriptor data is disabled\n");
		}
	}
}

static int pp2_descriptor_params(void *arg, int argc, char *argv[])
{
	struct port_desc *ports_desc = (struct port_desc *)arg;
	int i, option;
	int long_index = 0;
	u8 port_id = 0;
	char *ret_ptr;
	struct option long_options[] = {
		{"port", required_argument, 0, 'p'},
		{"rx", no_argument, 0, 'r'},
		{"tx", no_argument, 0, 't'},
		{"set", no_argument, 0, 's'},
		{"clear", no_argument, 0, 'c'},
		{0, 0, 0, 0}
	};

	if  (argc < 1 || argc > 4) {
		pr_err("Invalid number of arguments for %s command! number of arguments = %d\n", __func__, argc);
		return -EINVAL;
	}

	/* every time starting getopt we should reset optind */
	optind = 0;
	for (i = 0; ((option = getopt_long_only(argc, argv, "", long_options, &long_index)) != -1); i++) {
		/* Get parameters */
		switch (option) {
		case 'p':
			port_id = strtoul(optarg, &ret_ptr, 0);
			if ((optarg == ret_ptr) || (port_id < 0)) {
				printf("parsing fail, wrong input for --port\n");
				return -EINVAL;
			}
			break;
		case 'r':
			pp2_rx_descriptor_dump(&ports_desc[port_id]);
			break;
		case 't':
			pp2_tx_descriptor_dump(&ports_desc[port_id]);
			break;
		case 's':
			for (i = 0; i < MVAPPS_MAX_NUM_CORES; i++) {
				if (ports_desc[port_id].lcl_ports_desc[i]) {
					pr_info("setting save_desc_data on core %d\n", i);
					ports_desc[port_id].lcl_ports_desc[i]->save_desc_data = 1;
				}
			}
			break;
		case 'c':
			for (i = 0; i < MVAPPS_MAX_NUM_CORES; i++) {
				if (ports_desc[port_id].lcl_ports_desc[i])
					ports_desc[port_id].lcl_ports_desc[i]->save_desc_data = 0;
			}
			break;
		default:
			printf("(%d) parsing fail, wrong input\n", __LINE__);
			return -EINVAL;
		}
	}

	return 0;
}

int app_register_cli_common_cmds(struct glb_common_args *glb_args)
{
	struct cli_cmd_params cmd_params;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) glb_args->plat;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "qstat";
	cmd_params.desc		= "Show queues statistics";
	cmd_params.format	= "--port --queue --reset\n"
				  "\t\t--port,  -p	port number, if not specified, show for all ports\n"
				  "\t\t--tc,    -t	tc number, if not specified, show for all tcs\n"
				  "\t\t--queue, -q	queue index, if not specified, show for all queues.\n"
				  "\t\t--reset, -r	reset statistics\n";
	cmd_params.cmd_arg	= glb_args;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))queue_stat_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "pstat";
	cmd_params.desc		= "Show ports statistics";
	cmd_params.format	= "--port --reset\n"
				  "\t\t--port, -p	port number, if not specified, show for all ports\n"
				  "\t\t--reset, -r	reset statistics\n";
	cmd_params.cmd_arg	= glb_args;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))port_stat_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	memset(&cmd_params, 0, sizeof(cmd_params));

	cmd_params.name		= "pmac";
	cmd_params.desc		= "Show port ethtool get";
	cmd_params.format	= "--port\n"
				  "\t\t--port, -p	port number, if not specified, show for all ports\n";
	cmd_params.cmd_arg	= glb_args;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))port_ethtool_get_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	cmd_params.name		= "penable";
	cmd_params.desc		= "Port enable/disable";
	cmd_params.format	= "--port --enable\n"
				  "\t\t--port, -p	port number, required param\n"
				  "\t\t--enable, -e	0-disable, 1-enable, disable if unspecified\n";
	cmd_params.cmd_arg	= pp2_args->ports_desc;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))port_enable_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	cmd_params.name		= "txqstate";
	cmd_params.desc		= "Tx Queue enable/disable";
	cmd_params.format	= "--port --queue --enable\n"
				  "\t\t--port, -p	port number, if not specified, applied for all ports\n"
				  "\t\t--queue, -q	tx queue index, if not specified, applied for all queues\n"
				  "\t\t--enable, -e	0-disable, 1-enable, show current if unspecified\n";

	cmd_params.cmd_arg	= glb_args;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))txqueue_enable_cmd_cb;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

int app_register_cli_desc_cmds(struct port_desc *port_desc)
{
	struct cli_cmd_params cmd_params;

	memset(&cmd_params, 0, sizeof(cmd_params));
	cmd_params.name		= "descriptors";
	cmd_params.desc		= "dump information on rx/tx descriptors";
	cmd_params.format	= "\t\t--port --set/--clear --rx/--tx\n"
				  "\t\t\t\t--port, -p	port number, if not specified, show for port 0\n"
				  "\t\t\t\t--set, -s	start collecting information on last descriptor\n"
				  "\t\t\t\t--clear, -c	clear collecting information on last descriptor\n"
				  "\t\t\t\t--rx, -r	dump last received RX descriptor\n"
				  "\t\t\t\t--tx, -t	dump last received TX descriptor\n";
	cmd_params.cmd_arg	= port_desc;
	cmd_params.do_cmd_cb	= (int (*)(void *, int, char *[]))pp2_descriptor_params;
	mvapp_register_cli_cmd(&cmd_params);

	return 0;
}

static int find_free_bpool(u32 pp_id)
{
	int i;

	for (i = 0; i < PP2_BPOOL_NUM_POOLS; i++) {
		if (!((1 << i) & used_bpools[pp_id])) {
			used_bpools[pp_id] |= (1 << i);
			break;
		}
	}
	if (i == PP2_BPOOL_NUM_POOLS) {
		pr_err("no free BPool found!\n");
		return -ENOSPC;
	}
	return i;
}

static int find_free_hif(void)
{
	int i;

	for (i = 0; i < MVAPPS_PP2_TOTAL_NUM_HIFS; i++) {
		if (!((1 << i) & used_hifs)) {
			used_hifs |= (1 << i);
			break;
		}
	}
	if (i == MVAPPS_PP2_TOTAL_NUM_HIFS) {
		pr_err("no free HIF found!\n");
		return -ENOSPC;
	}

	return i;
}

int app_hif_init(struct pp2_hif **hif, u32 queue_size)
{
	int hif_id;
	char name[15];
	struct pp2_hif_params hif_params;
	int err;

	hif_id = find_free_hif();
	if (hif_id < 0) {
		pr_err("free HIF not found!\n");
		return hif_id;
	}

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "hif-%d", hif_id);
	pr_debug("found hif: %s\n", name);
	memset(&hif_params, 0, sizeof(hif_params));
	hif_params.match = name;
	hif_params.out_size = queue_size;
	err = pp2_hif_init(&hif_params, hif);
	if (err)
		return err;
	if (!hif) {
		pr_err("HIF init failed!\n");
		return -EIO;
	}

	return 0;
}

int app_build_all_bpools(struct bpool_desc ***ppools, int num_pools, struct bpool_inf infs[], struct pp2_hif *hif)
{
	struct pp2_bpool_params		bpool_params;
	int				i, j, k, err, pool_id;
	char				name[15];
	struct bpool_desc		**pools = NULL;
	struct pp2_buff_inf		*buffs_inf = NULL;
	u8  pp2_num_inst = pp2_get_num_inst();

	buf_alloc_cnt = 0;

	if (num_pools > MVAPPS_PP2_MAX_NUM_BPOOLS) {
		pr_err("only %d pools allowed!\n", MVAPPS_PP2_MAX_NUM_BPOOLS);
		return -EINVAL;
	}
/* TODO: release memory on error */
	pools = (struct bpool_desc **)malloc(pp2_num_inst * sizeof(struct bpool_desc *));
	if (!pools) {
		pr_err("no mem for bpool_desc array!\n");
		return -ENOMEM;
	}

	*ppools = pools;

	for (i = 0; i < pp2_num_inst; i++) {
		pools[i] = (struct bpool_desc *)malloc(num_pools * sizeof(struct bpool_desc));
		if (!pools[i]) {
			pr_err("no mem for bpool_desc array!\n");
			return -ENOMEM;
		}

		for (j = 0; j < num_pools; j++) {
			pool_id = find_free_bpool(i);
			if (pool_id < 0) {
				pr_err("free bpool not found!\n");
				return pool_id;
			}
			memset(name, 0, sizeof(name));
			snprintf(name, sizeof(name), "pool-%d:%d", i, pool_id);
			memset(&bpool_params, 0, sizeof(bpool_params));
			bpool_params.match = name;
			bpool_params.buff_len = infs[j].buff_size;

			pr_info("%s: buff_size %d, num_buffs %d\n", name, infs[j].buff_size, infs[j].num_buffs);
			err = pp2_bpool_init(&bpool_params, &pools[i][j].pool);
			if (err)
				return err;

			if (!pools[i][j].pool) {
				pr_err("BPool id%d init failed!\n", pool_id);
				return -EIO;
			}

			pools[i][j].buffs_inf =
				(struct pp2_buff_inf *)malloc(infs[j].num_buffs * sizeof(struct pp2_buff_inf));

			if (!pools[i][j].buffs_inf) {
				pr_err("no mem for bpools-inf array!\n");
				return -ENOMEM;
			}

			buffs_inf = pools[i][j].buffs_inf;
			pools[i][j].num_buffs = infs[j].num_buffs;

			for (k = 0; k < infs[j].num_buffs; k++) {
				void *buff_virt_addr;

				buff_virt_addr = mv_sys_dma_mem_alloc(infs[j].buff_size, 4);
				if (!buff_virt_addr) {
					pr_err("failed to allocate mem (%d)!\n", k);
					return -1;
				}
				buffs_inf[k].addr = mv_sys_dma_mem_virt2phys(buff_virt_addr);
				buffs_inf[k].cookie = (uintptr_t)buff_virt_addr;
			}

			for (k = 0; k < infs[j].num_buffs; k++) {
				struct pp2_buff_inf	tmp_buff_inf;

				tmp_buff_inf.cookie = buffs_inf[k].cookie;
				tmp_buff_inf.addr   = buffs_inf[k].addr;
				err = pp2_bpool_put_buff(hif, pools[i][j].pool, &tmp_buff_inf);
				if (err)
					return err;
				buf_alloc_cnt++;
			}
		}
	}
	return 0;
}

int app_find_port_info(struct port_desc *port_desc)
{
	char		 name[20];
	u8		 pp, ppio;
	int		 err;

	if (!port_desc->name) {
		pr_err("No port name given!\n");
		return -1;
	}

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "%s", port_desc->name);
	err = pp2_netdev_get_ppio_info(name, &pp, &ppio);
	if (err) {
		pr_err("PP2 Port %s not found!\n", port_desc->name);
		return err;
	}

	port_desc->ppio_id = ppio;
	port_desc->pp_id = pp;

	return 0;
}

int app_port_init(struct port_desc *port, int num_pools, struct bpool_desc *pools, u16 mtu, u16 pkt_offset)
{
	struct pp2_ppio_params		*port_params = &port->port_params;
	struct pp2_ppio_inq_params	inq_params;
	char				name[MVAPPS_PPIO_NAME_MAX];
	int				i, j, err = 0;
	u16				curr_mtu;
	char				file[PP2_MAX_BUF_STR_LEN];
	u32				kernel_first_rxq, kernel_num_rx_queues;

	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "ppio-%d:%d",
		 port->pp_id, port->ppio_id);
	pr_debug("found port: %s\n", name);
	port_params->match = name;
	port_params->type = port->ppio_type;
	port_params->inqs_params.num_tcs = port->num_tcs;
	port_params->inqs_params.hash_type = port->hash_type;

	if (port->ppio_type == PP2_PPIO_T_LOG) {
		sprintf(file, "%s/%s", PP2_SYSFS_MUSDK_PATH, PP2_SYSFS_RX_FIRST_RXQ_FILE);
		kernel_first_rxq = appp_pp2_sysfs_param_get(port->name, file);

		sprintf(file, "%s/%s", PP2_SYSFS_MUSDK_PATH, PP2_SYSFS_RX_NUM_RXQ_FILE);
		kernel_num_rx_queues = appp_pp2_sysfs_param_get(port->name, file);

		port_params->specific_type_params.log_port_params.first_inq = port->first_inq +
									      kernel_first_rxq + kernel_num_rx_queues;
	} else {
		port_params->specific_type_params.log_port_params.first_inq = port->first_inq;
	}

	for (i = 0; i < port->num_tcs; i++) {
		port_params->inqs_params.tcs_params[i].pkt_offset = (pkt_offset) ? pkt_offset : MVAPPS_PP2_PKT_DEF_OFFS;
		port_params->inqs_params.tcs_params[i].num_in_qs = port->num_inqs[i];
		inq_params.size = port->inq_size;
		port_params->inqs_params.tcs_params[i].inqs_params = &inq_params;

		for (j = 0; j < num_pools; j++)
			port_params->inqs_params.tcs_params[i].pools[j] = pools[j].pool;
	}
	port_params->outqs_params.num_outqs = port->num_outqs;
	for (i = 0; i < port->num_outqs; i++) {
		port_params->outqs_params.outqs_params[i].size = port->outq_size;
	}

	err = pp2_ppio_init(port_params, &port->ppio);
	if (err) {
		pr_err("PP-IO init failed (error: %d)!\n", err);
		return err;
	}

	if (!port->ppio) {
		pr_err("PP-IO init failed!\n");
		return -EIO;
	}

	if (mtu) {
		/* Change port MTU if needed */
		pp2_ppio_get_mtu(port->ppio, &curr_mtu);
		if (curr_mtu != mtu) {
			pp2_ppio_set_mtu(port->ppio, mtu);
			pp2_ppio_set_mru(port->ppio, MVAPPS_MTU_TO_MRU(mtu));
			pr_info("Set port ppio-%d:%d MTU to %d\n",
				port->pp_id, port->ppio_id, mtu);
		}
	}

	err = pp2_ppio_enable(port->ppio);
	port->initialized = 1;

	return err;
}

void app_port_local_init(int id, int lcl_id, struct lcl_port_desc *lcl_port, struct port_desc *port)
{
	int i;
	char	file[PP2_MAX_BUF_STR_LEN];
	u32	kernel_num_tx_queues;

	lcl_port->id		= id;
	lcl_port->lcl_id	= lcl_id;
	lcl_port->pp_id		= port->pp_id;
	lcl_port->ppio_id	= port->ppio_id;
	lcl_port->ppio		= port->ppio;

	lcl_port->num_shadow_qs = port->num_outqs;
	/* Below shadow_q size is enough to hold tx_q + any_size burst from all a thread's rx_qs */
	lcl_port->shadow_q_size	= port->outq_size + port->num_tcs*port->inq_size;
	lcl_port->shadow_qs = (struct tx_shadow_q *)malloc(port->num_outqs * sizeof(struct tx_shadow_q));

	for (i = 0; i < lcl_port->num_shadow_qs; i++) {
		lcl_port->shadow_qs[i].read_ind = 0;
		lcl_port->shadow_qs[i].write_ind = 0;

		lcl_port->shadow_qs[i].ents =
			(struct tx_shadow_q_entry *)malloc(lcl_port->shadow_q_size * sizeof(struct tx_shadow_q_entry));
	}
	for (i = 0; i < port->num_tcs; i++)
		lcl_port->pkt_offset[i] = port->port_params.inqs_params.tcs_params[i].pkt_offset;

	if (port->ppio_type == PP2_PPIO_T_LOG) {
		sprintf(file, "%s/%s", PP2_SYSFS_MUSDK_PATH, PP2_SYSFS_TX_NUM_TXQ_FILE);
		kernel_num_tx_queues = appp_pp2_sysfs_param_get(port->name, file);
		if (kernel_num_tx_queues < PP2_PPIO_MAX_NUM_OUTQS) {
			lcl_port->first_txq = kernel_num_tx_queues;
		} else {
			pr_err("All TX queues are reserved by Kernel");
			lcl_port->first_txq = PP2_PPIO_MAX_NUM_OUTQS;
		}
	} else {
		lcl_port->first_txq = 0;
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
		pr_debug("Release %d buffers from shadow_q-%d port:%d:%d\n",
			 cnt, i, lcl_port->pp_id, lcl_port->ppio_id);
		shadow_q->read_ind = 0;
		shadow_q->write_ind = 0;
		free(shadow_q->ents);
	}

	free(lcl_port->shadow_qs);
}

static void free_rx_queues(struct pp2_ppio *port, u16 num_tcs, u16 num_inqs[])
{
	struct pp2_ppio_desc	descs[MVAPPS_MAX_BURST_SIZE];
	u8			tc = 0, qid = 0;
	u16			num;

	for (tc = 0; tc < num_tcs; tc++) {
		for (qid = 0; qid < num_inqs[tc]; qid++) {
			num = MVAPPS_MAX_BURST_SIZE;
			while (num) {
				pp2_ppio_recv(port, tc, qid, descs, &num);
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
			pp2_ppio_disable(ports[i].ppio);
			free_rx_queues(ports[i].ppio, ports[i].num_tcs, ports[i].num_inqs);
		}
	}
}

void app_deinit_all_ports(struct port_desc *ports, int num_ports)
{
	int i;

	for (i = 0;  i < num_ports; i++) {
		if (ports[i].ppio)
			pp2_ppio_deinit(ports[i].ppio);
	}

	/* Calculate number of buffers released from PP2 */
	for (i = 0; i < MVAPPS_MAX_NUM_CORES; i++)
		hw_buf_free_cnt += tx_shadow_q_buf_free_cnt[i];
	hw_buf_free_cnt += hw_bm_buf_free_cnt + hw_rxq_buf_free_cnt;

	if (buf_free_cnt != buf_alloc_cnt) {
		pr_err("Not all buffers were released: allocated: %" PRIu64 ", freed: %" PRIu64 "\n",
		       buf_alloc_cnt, buf_free_cnt);

		if (buf_free_cnt != hw_buf_free_cnt) {
			pr_err("pp2 freed : %" PRIu64 "\n", hw_buf_free_cnt);
			pr_err("bm freed  : %" PRIu64 "\n", hw_bm_buf_free_cnt);
			pr_err("rxq freed : %" PRIu64 "\n", hw_rxq_buf_free_cnt);
			pr_err("tx freed  : %" PRIu64 "\n",
				(hw_buf_free_cnt - hw_bm_buf_free_cnt - hw_rxq_buf_free_cnt));
		}
	}

	pr_debug("allocated : %" PRIu64 "\n", buf_alloc_cnt);
	pr_debug("app freed : %" PRIu64 "\n", buf_free_cnt);
	pr_debug("pp2 freed : %" PRIu64 "\n", hw_buf_free_cnt);
	pr_debug("bm freed  : %" PRIu64 "\n", hw_bm_buf_free_cnt);
	pr_debug("rxq freed : %" PRIu64 "\n", hw_rxq_buf_free_cnt);
	pr_debug("tx freed  : %" PRIu64 "\n",
		(hw_buf_free_cnt - hw_bm_buf_free_cnt - hw_rxq_buf_free_cnt));
}

static void flush_pool(struct pp2_bpool *bpool, struct pp2_hif *hif)
{
	u32 i, buf_num, cnt = 0, err = 0;

	pp2_bpool_get_num_buffs(bpool, &buf_num);
	for (i = 0; i < buf_num; i++) {
		struct pp2_buff_inf buff;

		err = 0;
		while (pp2_bpool_get_buff(hif, bpool, &buff)) {
			err++;
			if (err == 10000) {
				buff.cookie = 0;
				break;
			}
		}

		if (err) {
			if (err == 10000) {
				pr_err("flush_pool: p2_id=%d, pool_id=%d: Got NULL buf (%d of %d)\n",
				       bpool->pp2_id, bpool->id, i, buf_num);
				continue;
			}
			pr_warn("flush_pool: p2_id=%d, pool_id=%d: Got buf (%d of %d) after %d retries\n",
				bpool->pp2_id, bpool->id, i, buf_num, err);
		}
		cnt++;
	}
	hw_bm_buf_free_cnt += cnt;
	pp2_bpool_deinit(bpool);
}

static void free_pool_buffers(struct pp2_buff_inf *buffs, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		void *buff_virt_addr = (void *)(uintptr_t)buffs[i].cookie;

		mv_sys_dma_mem_free(buff_virt_addr);
		buf_free_cnt++;
	}
}

void app_free_all_pools(struct bpool_desc **pools, int num_pools, struct pp2_hif *hif)
{
	int i, j;
	u8  pp2_num_inst = pp2_get_num_inst();

	if (pools) {
		for (i = 0; i < pp2_num_inst; i++) {
			if (pools[i]) {
				for (j = 0; j < num_pools; j++)
					if (pools[i][j].pool) {
						flush_pool(pools[i][j].pool, hif);
						free_pool_buffers(pools[i][j].buffs_inf, pools[i][j].num_buffs);
						free(pools[i][j].buffs_inf);
					}
				free(pools[i]);
			}
		}
		free(pools);
	}
}

u32 appp_pp2_sysfs_param_get(char *if_name, char *file)
{
	FILE *fp;
	char r_buf[PP2_MAX_BUF_STR_LEN];
	char w_buf[PP2_MAX_BUF_STR_LEN];
	u32 param = 0, scanned;

	sprintf(w_buf, "echo %s > %s/%s", if_name, PP2_SYSFS_MUSDK_PATH, PP2_SYSFS_DEBUG_PORT_SET_FILE);
	system(w_buf);

	fp = fopen(file, "r");
	if (!fp) {
		pr_err("error opening file %s\n", file);
		return -EEXIST;
	}

	fgets(r_buf, sizeof(r_buf), fp);
	scanned = sscanf(r_buf, "%d\n", &param);
	if (scanned != 1) {
		pr_err("Invalid number of parameters read %s\n", r_buf);
		fclose(fp);
		return -EINVAL;
	}

	fclose(fp);
	return param;
}


void apps_pp2_deinit_local(void *arg)
{
	struct local_common_args *common_arg = (struct local_common_args *)arg;
	struct pp2_lcl_common_args *pp2_args = (struct pp2_lcl_common_args *) common_arg->plat;
	int i;

	if (!common_arg)
		return;

	if (pp2_args->lcl_ports_desc) {
		for (i = 0; i < common_arg->num_ports; i++)
			app_port_local_deinit(&pp2_args->lcl_ports_desc[i]);
		free(pp2_args->lcl_ports_desc);
	}

	if (pp2_args->hif)
		pp2_hif_deinit(pp2_args->hif);
	free((void *)pp2_args);
	free(common_arg);
}

void apps_pp2_destroy_local_modules(void *arg)
{
	struct glb_common_args *common_arg = (struct glb_common_args *)arg;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) common_arg->plat;

	app_disable_all_ports(pp2_args->ports_desc, common_arg->num_ports);
	app_free_all_pools(pp2_args->pools_desc, pp2_args->num_pools, pp2_args->hif);
	app_deinit_all_ports(pp2_args->ports_desc, common_arg->num_ports);

	if (pp2_args->hif)
		pp2_hif_deinit(pp2_args->hif);
}

void apps_pp2_destroy_all_modules(void)
{
	pp2_deinit();
	mv_sys_dma_mem_destroy();
}


void apps_pp2_deinit_global(void *arg)
{
	struct glb_common_args *common_arg = (struct glb_common_args *)arg;

	if (!common_arg)
		return;

	if (common_arg->cli && common_arg->cli_unregister_cb)
		common_arg->cli_unregister_cb(common_arg);
	apps_pp2_destroy_local_modules(common_arg);
	apps_pp2_destroy_all_modules();

	if (common_arg->plat)
		free(common_arg->plat);
}

int apps_pp2_stat_cmd_cb(void *arg, int argc, char *argv[])
{
	int i, j, reset = 0;
	struct glb_common_args *g_cmn_args = (struct glb_common_args *)arg;
	struct pp2_glb_common_args *pp2_args = (struct pp2_glb_common_args *) g_cmn_args->plat;
	struct pp2_counters *cntrs;

	if (argc > 1)
		reset = 1;
	printf("reset stats: %d\n", reset);
	for (j = 0; j < g_cmn_args->num_ports; j++) {
		printf("Port%d stats:\n", j);
		for (i = 0; i < g_cmn_args->cpus; i++) {
			cntrs = &(pp2_args->ports_desc[j].lcl_ports_desc[i]->cntrs);
			printf("cpu%d: rx_bufs=%u, tx_bufs=%u, free_bufs=%u, tx_resend=%u, max_retries=%u, ",
			       i,  cntrs->rx_buf_cnt, cntrs->tx_buf_cnt, cntrs->free_buf_cnt,
				cntrs->tx_buf_retry, cntrs->tx_max_resend);
			printf(" tx_drops=%u, max_burst=%u\n", cntrs->tx_buf_drop, cntrs->tx_max_burst);
			if (reset) {
				cntrs->rx_buf_cnt = 0;
				cntrs->tx_buf_cnt = 0;
				cntrs->free_buf_cnt = 0;
				cntrs->tx_buf_retry = 0;
				cntrs->tx_buf_drop = 0;
				cntrs->tx_max_burst = 0;
				cntrs->tx_max_resend = 0;
			}
		}
	}
	return 0;
}

