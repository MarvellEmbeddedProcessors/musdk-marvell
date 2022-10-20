/* Copyright (c) 2014 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/phy.h>
#include "giu_nic.h"


#define AGNIC_STATS_LEN		ARRAY_SIZE(agnic_gstrings_stats)

static const char agnic_gstrings_stats[][ETH_GSTRING_LEN] = {
	/* device-specific stats */
	"pp_rx_bytes", "pp_rx_enq_frames", "pp_rx_unicast_frames", "pp_rx_errors", "pp_rx_fullq_dropped",
	"pp_rx_bm_dropped", "pp_rx_early_dropped", "pp_rx_fifo_dropped", "pp_rx_cls_dropped",
	"pp_tx_bytes", "pp_tx_deq_frames", "pp_tx_unicast_frames", "pp_tx_errors",
	"sw_rx_frames", "sw_rx_dropped", "sw_tx_frames", "sw_tx_dropped",
	"fw_rx_dropped", "gp_rx_frames", "gp_tx_frames",
};

static char *agnic_gstring_q_stats[ETH_GSTRING_LEN];
static int   gstring_q_num_lines;

static int agnic_get_settings(struct net_device *netdev,
		struct ethtool_cmd *ecmd)
{
	struct agnic_adapter *adapter = netdev_priv(netdev);
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_resp resp;
	u32 phy_mode;
	int ret;

	/* Build Link Status cmd descriptor */
	msg_params.cmd_code = CC_PF_LINK_INFO;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = NULL; /* no params */
	msg_params.msg_len = 0;
	msg_params.resp_msg = &resp;
	msg_params.resp_msg_len = sizeof(struct agnic_mgmt_cmd_resp);

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret) {
		pr_err("%s - Link info failed\n", __func__);
		return 0;
	}

	if (resp.link_info.link_up) {
		switch (resp.link_info.speed) {
		case NET_LINK_SPEED_10000:
			ethtool_cmd_speed_set(ecmd, SPEED_10000);
			break;
		case NET_LINK_SPEED_2500:
			ethtool_cmd_speed_set(ecmd, SPEED_2500);
			break;
		case NET_LINK_SPEED_1000:
			ethtool_cmd_speed_set(ecmd, SPEED_1000);
			break;
		case NET_LINK_SPEED_100:
			ethtool_cmd_speed_set(ecmd, SPEED_100);
			break;
		case NET_LINK_SPEED_10:
			ethtool_cmd_speed_set(ecmd, SPEED_10);
			break;
		default:
			ethtool_cmd_speed_set(ecmd, SPEED_UNKNOWN);
			break;
		}

		switch (resp.link_info.duplex) {
		case NET_LINK_DUPLEX_FULL:
			ecmd->duplex = DUPLEX_FULL;
			break;
		case NET_LINK_DUPLEX_HALF:
			ecmd->duplex = DUPLEX_HALF;
			break;
		default:
			ecmd->duplex = DUPLEX_UNKNOWN;
			break;
		}
	} else { /* No link */
		ethtool_cmd_speed_set(ecmd, SPEED_UNKNOWN);
		ecmd->duplex = DUPLEX_UNKNOWN;
		ecmd->autoneg = AUTONEG_DISABLE;
		ecmd->port = PORT_OTHER;
	}

	phy_mode = resp.link_info.phy_mode;
	if ((phy_mode == PHY_INTERFACE_MODE_XAUI) ||
		    (phy_mode == PHY_INTERFACE_MODE_RXAUI) ||
#ifdef CONFIG_LK4_4_COMPAT
		    (phy_mode == PHY_INTERFACE_MODE_KR)   ||
		    (phy_mode == PHY_INTERFACE_MODE_SFI) ||
		    (phy_mode == PHY_INTERFACE_MODE_XFI)) {
#else
		/* in kernel 4.14, 10GBASE-KR, XFI, SFI - single lane 10G Serdes
		 * renamed to PHY_INTERFACE_MODE_10GKR
		 */
		    (phy_mode == PHY_INTERFACE_MODE_10GKR)) {
#endif /* CONFIG_LK4_4_COMPAT */
		ecmd->autoneg = AUTONEG_DISABLE;
		ecmd->supported = (SUPPORTED_10000baseT_Full |
			SUPPORTED_FIBRE);
		ecmd->advertising = (ADVERTISED_10000baseT_Full |
			ADVERTISED_FIBRE);
		ecmd->transceiver = XCVR_EXTERNAL;
		ecmd->port = PORT_FIBRE;
	} else {
		ecmd->autoneg = AUTONEG_DISABLE; /* TODO: get also autoneg status */
		ecmd->supported = (SUPPORTED_10baseT_Half |
			SUPPORTED_10baseT_Full |
			SUPPORTED_100baseT_Half |
			SUPPORTED_100baseT_Full |
			SUPPORTED_Autoneg | SUPPORTED_TP |
			SUPPORTED_MII | SUPPORTED_1000baseT_Full);
		ecmd->advertising = (ADVERTISED_10baseT_Half |
			ADVERTISED_10baseT_Full |
			ADVERTISED_100baseT_Half |
			ADVERTISED_100baseT_Full |
			ADVERTISED_1000baseT_Full |
			ADVERTISED_Autoneg | ADVERTISED_TP |
			ADVERTISED_MII);
		ecmd->transceiver = XCVR_INTERNAL;
		ecmd->port = PORT_MII;
	}

	return 0;
}

static int agnic_set_settings(struct net_device *netdev,
			      struct ethtool_cmd *ecmd)
{
	return -EOPNOTSUPP;
}

static u32 agnic_get_link(struct net_device *netdev)
{
	struct agnic_adapter *adapter = netdev_priv(netdev);
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_resp resp;
	int ret;

	/* Build Link Status cmd descriptor */
	msg_params.cmd_code = CC_PF_LINK_STATUS;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = NULL; /* no params */
	msg_params.msg_len = 0;
	msg_params.resp_msg = &resp;
	msg_params.resp_msg_len = sizeof(struct agnic_mgmt_cmd_resp);

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret) {
		pr_err("%s - Link status check failed\n", __func__);
		return 0;
	}

	/* Get the link status from the response descriptor */
	return resp.link_status;
}

static void agnic_get_pauseparam(struct net_device *netdev,
				 struct ethtool_pauseparam *pause)
{
	struct agnic_adapter *adapter = netdev_priv(netdev);
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_resp resp;
	int ret;

	/* Build FC Get cmd descriptor */
	msg_params.cmd_code = CC_PF_PAUSE_GET;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = NULL; /* no params */
	msg_params.msg_len = 0;
	msg_params.resp_msg = &resp;
	msg_params.resp_msg_len = sizeof(struct agnic_mgmt_cmd_resp);

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret) {
		pr_err("%s - pause get failed\n", __func__);
		return;
	}

	pause->rx_pause = resp.pause_params.rx;
	pause->tx_pause = resp.pause_params.tx;
}

static int agnic_set_pauseparam(struct net_device *netdev,
				struct ethtool_pauseparam *pause)
{
	struct agnic_adapter *adapter = netdev_priv(netdev);
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	int ret;

	cmd_params.pause_params.tx = pause->tx_pause;
	cmd_params.pause_params.rx = pause->rx_pause;

	/* Build FC Set cmd descriptor */
	msg_params.cmd_code = CC_PF_PAUSE_SET;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = sizeof(cmd_params);

	/* don't wait for a response */
	msg_params.resp_msg = NULL;
	msg_params.resp_msg_len = 0;

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret)
		pr_err("%s - pause set failed\n", __func__);

	return ret;
}

static u32 agnic_get_msglevel(struct net_device *netdev)
{
	struct agnic_adapter *adapter = netdev_priv(netdev);

	return adapter->msg_enable;
}

static void agnic_set_msglevel(struct net_device *netdev, u32 data)
{
	struct agnic_adapter *adapter = netdev_priv(netdev);

	adapter->msg_enable = data;
}

static int agnic_get_regs_len(struct net_device *netdev)
{
	return 0;
}

static void agnic_get_regs(struct net_device *netdev, struct ethtool_regs *regs,
			   void *p)
{
}

static void agnic_get_drvinfo(struct net_device *netdev,
			      struct ethtool_drvinfo *drvinfo)
{
}

static void agnic_get_ringparam(struct net_device *netdev,
				struct ethtool_ringparam *ring)
{
}

static int agnic_set_ringparam(struct net_device *netdev,
			       struct ethtool_ringparam *ring)
{
	return -EOPNOTSUPP;
}

static void agnic_diag_test(struct net_device *netdev,
			    struct ethtool_test *eth_test, u64 *data)
{
}

static int agnic_set_phys_id(struct net_device *netdev,
			     enum ethtool_phys_id_state state)
{
	return -EOPNOTSUPP;
}

static int agnic_get_coalesce(struct net_device *netdev,
			      struct ethtool_coalesce *ec)
{
	return -EOPNOTSUPP;
}

static int agnic_set_coalesce(struct net_device *netdev,
			      struct ethtool_coalesce *ec)
{
	return -EOPNOTSUPP;
}

static int agnic_nway_reset(struct net_device *netdev)
{
	return -EOPNOTSUPP;
}

static int agnic_get_pp_stats(struct net_device *netdev, u64 *data)
{
	struct agnic_adapter *adapter = netdev_priv(netdev);
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_mgmt_cmd_resp resp;
	int msg_len = sizeof(struct agnic_mgmt_cmd_params);
	int resp_msg_len = sizeof(struct agnic_mgmt_cmd_resp);
	int ret, pos = 0;

	/* agnic get statistics has an option to reset the counters. Nevertheless,
	 * ethtool has no such option, so reset flag should be set to 0 when
	 * get statistics is called from ethtool
	 */
	cmd_params.pf_get_statistics.reset = 0;

	/* Build statistics cmd descriptor */
	msg_params.cmd_code = CC_PF_GET_STATISTICS;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &resp;
	msg_params.resp_msg_len = resp_msg_len;

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret) {
		pr_err("%s - statistics send cmd failed\n", __func__);
		return -EINVAL;
	}

	if (resp.status != AGNIC_NOTIF_STATUS_OK) {
		pr_err("%s - statistics response cmd failed\n", __func__);
		return -EINVAL;
	}

	data[pos++] = resp.agnic_stats.rx_bytes;
	data[pos++] = resp.agnic_stats.rx_packets;
	data[pos++] = resp.agnic_stats.rx_unicast_packets;
	data[pos++] = resp.agnic_stats.rx_errors;
	data[pos++] = resp.agnic_stats.rx_fullq_dropped;
	data[pos++] = resp.agnic_stats.rx_bm_dropped;
	data[pos++] = resp.agnic_stats.rx_early_dropped;
	data[pos++] = resp.agnic_stats.rx_fifo_dropped;
	data[pos++] = resp.agnic_stats.rx_cls_dropped;
	data[pos++] = resp.agnic_stats.tx_bytes;
	data[pos++] = resp.agnic_stats.tx_packets;
	data[pos++] = resp.agnic_stats.tx_unicast_packets;
	data[pos++] = resp.agnic_stats.tx_errors;

	return pos;
}

static int agnic_get_gp_stats(struct net_device *netdev, u64 *data)
{
	struct agnic_adapter *adapter = netdev_priv(netdev);
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_mgmt_cmd_resp resp;
	int msg_len = sizeof(struct agnic_mgmt_cmd_params);
	int resp_msg_len = sizeof(struct agnic_mgmt_cmd_resp);
	int ret, pos = 0;

	/* Build statistics cmd descriptor */
	msg_params.cmd_code = CC_PF_GET_GP_STATS;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &resp;
	msg_params.resp_msg_len = resp_msg_len;

	ret = agnic_mgmt_command_send(adapter, &msg_params);
	if (ret) {
		pr_err("%s - gp statistics send cmd failed\n", __func__);
		return -EINVAL;
	}

	if (resp.status != AGNIC_NOTIF_STATUS_OK) {
		pr_err("%s - gp statistics response cmd failed\n", __func__);
		return -EINVAL;
	}

	data[pos++] = resp.gp_stats.gp_rx_fullq_dropped; /* fw_rx_dropped */
	data[pos++] = resp.gp_stats.gp_rx_packets;
	data[pos++] = resp.gp_stats.gp_tx_packets;

	return pos;
}

static int agnic_get_adapter_stats(struct net_device *netdev, u64 *data)
{
	struct agnic_adapter *adapter = netdev_priv(netdev);
	int pos = 0;

	data[pos++] = adapter->stats.rx_packets;
	data[pos++] = adapter->stats.rx_dropped;
	data[pos++] = adapter->stats.tx_packets;
	data[pos++] = adapter->stats.tx_dropped;

	return pos;
}

static int agnic_get_gp_queues_stats(struct net_device *netdev, u64 *data)
{
	struct agnic_adapter *adapter = netdev_priv(netdev);
	struct agnic_msg_params msg_params;
	struct agnic_mgmt_cmd_params cmd_params;
	struct agnic_mgmt_cmd_resp resp;
	int msg_len = sizeof(struct agnic_mgmt_cmd_params);
	int resp_msg_len = sizeof(struct agnic_mgmt_cmd_resp);
	int ret, pos = 0, tc_idx, q_idx;

	/* Build queues statistics cmd descriptor */
	msg_params.cmd_code = CC_PF_GET_GP_QUEUE_STATS;
	msg_params.client_id = 0;
	msg_params.client_type = CDT_PF;
	msg_params.msg = &cmd_params;
	msg_params.msg_len = msg_len;
	msg_params.resp_msg = &resp;
	msg_params.resp_msg_len = resp_msg_len;
	cmd_params.pf_q_get_statistics.reset = 0;

	for (tc_idx = 0 ; tc_idx < adapter->num_tcs ; tc_idx++) {
		for (q_idx = 0 ; q_idx < adapter->num_qs_per_tc ; q_idx++) {
			cmd_params.pf_q_get_statistics.qid = q_idx;
			cmd_params.pf_q_get_statistics.tc = tc_idx;
			cmd_params.pf_q_get_statistics.out = 0;
			ret = agnic_mgmt_command_send(adapter, &msg_params);
			if (ret) {
				pr_err("%s - tc_idx-%d queue-%d statistics send cmd failed\n",
					   __func__, tc_idx, q_idx);
				return -EINVAL;
			}

			if (resp.status != AGNIC_NOTIF_STATUS_OK) {
				pr_err("%s - gp statistics response cmd failed\n", __func__);
				return -EINVAL;
			}
			data[pos++] = resp.q_stats.q_packets;

			/* send another request - for out direction */
			cmd_params.pf_q_get_statistics.out = 1;
			ret = agnic_mgmt_command_send(adapter, &msg_params);
			if (ret) {
				pr_err("%s - tc_idx-%d queue-%d statistics send cmd failed\n",
					   __func__, tc_idx, q_idx);
				return -EINVAL;
			}

			if (resp.status != AGNIC_NOTIF_STATUS_OK) {
				pr_err("%s - gp statistics response cmd failed\n", __func__);
				return -EINVAL;
			}
			data[pos++] = resp.q_stats.q_packets;
		}
	}

	return pos;
}

static void agnic_get_ethtool_stats(struct net_device *netdev,
				    struct ethtool_stats *stats, u64 *data)
{
	int len, pos = 0;

	len = agnic_get_pp_stats(netdev, data);
	if (len < 0) {
		pr_err("%s - failed to get pp statistics\n", __func__);
		return;
	}

	pos += len;
	len = agnic_get_adapter_stats(netdev, data + pos);
	if (len < 0) {
		pr_err("%s - failed to get sw statistics\n", __func__);
		return;
	}

	pos += len;
	len = agnic_get_gp_stats(netdev, data + pos);
	if (len < 0) {
		pr_err("%s - failed to get pp statistics\n", __func__);
		return;
	}

	pos += len;
	len = agnic_get_gp_queues_stats(netdev, data + pos);
	if (len < 0) {
		pr_err("%s - failed to get queues statistics\n", __func__);
		return;
	}
}

static void agnic_get_strings(struct net_device *netdev, u32 stringset,
			      u8 *data)
{
	int i;
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(data, *agnic_gstrings_stats, sizeof(agnic_gstrings_stats));
		for (i = 0 ; i < gstring_q_num_lines ; i++)
			memcpy(data + sizeof(agnic_gstrings_stats) + (i * ETH_GSTRING_LEN),
				   agnic_gstring_q_stats[i], ETH_GSTRING_LEN);
		break;
	default:
		break;
	}
}

static int agnic_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return AGNIC_STATS_LEN + gstring_q_num_lines;
	default:
		return -EOPNOTSUPP;
	}
}

static const struct ethtool_ops agnic_ethtool_ops = {
	.get_settings		= agnic_get_settings,
	.set_settings		= agnic_set_settings,
	.get_drvinfo		= agnic_get_drvinfo,
	.get_regs_len		= agnic_get_regs_len,
	.get_regs		= agnic_get_regs,
	.get_msglevel		= agnic_get_msglevel,
	.set_msglevel		= agnic_set_msglevel,
	.nway_reset		= agnic_nway_reset,
	.get_link		= agnic_get_link,
	.get_ringparam		= agnic_get_ringparam,
	.set_ringparam		= agnic_set_ringparam,
	.get_pauseparam		= agnic_get_pauseparam,
	.set_pauseparam		= agnic_set_pauseparam,
	.self_test		= agnic_diag_test,
	.get_strings		= agnic_get_strings,
	.set_phys_id		= agnic_set_phys_id,
	.get_ethtool_stats	= agnic_get_ethtool_stats,
	.get_sset_count		= agnic_get_sset_count,
	.get_coalesce		= agnic_get_coalesce,
	.set_coalesce		= agnic_set_coalesce,
	.get_ts_info		= ethtool_op_get_ts_info,
};

static void agnic_set_ethtool_ops(struct net_device *netdev)
{
	netdev->ethtool_ops = &agnic_ethtool_ops;
}

void agnic_ethtool_release(void)
{
	int i;

	for (i = 0 ; i < gstring_q_num_lines ; i++) {
		kfree(agnic_gstring_q_stats[i]);
		agnic_gstring_q_stats[i] = NULL;
	}
}

static int agnic_init_q_gstring(struct net_device *netdev)
{
	struct agnic_adapter *adapter = netdev_priv(netdev);
	int tc_idx, q_idx;
	int line = 0, i;

	/* allocate the dynamic part of ethtool's gstring */
	gstring_q_num_lines = adapter->num_tcs * adapter->num_qs_per_tc * 2;
	for (i = 0 ; i < gstring_q_num_lines ; i++) {
		agnic_gstring_q_stats[i] = kmalloc(ETH_GSTRING_LEN, GFP_KERNEL);
		if (!agnic_gstring_q_stats[i]) {
			agnic_ethtool_release();
			return -ENOMEM;
		}
	}

	for (tc_idx = 0 ; tc_idx < adapter->num_tcs ; tc_idx++) {
		for (q_idx = 0 ; q_idx < adapter->num_qs_per_tc ; q_idx++) {
			sprintf(agnic_gstring_q_stats[line], "gp_tc%d_rx%d_frames", tc_idx, q_idx);
			sprintf(agnic_gstring_q_stats[line+1], "gp_tc%d_tx%d_frames", tc_idx, q_idx);
			line += 2;
		}
	}

	return 0;
}

int agnic_ethtool_init(struct net_device *netdev)
{
	int ret;

	/* Register ethtool ops */
	agnic_set_ethtool_ops(netdev);

	/* Allocate string array */
	ret = agnic_init_q_gstring(netdev);
	if (ret) {
		pr_err("%s: gsting init failed (%d)\n", __func__, ret);
		return ret;
	}

	return 0;
}

