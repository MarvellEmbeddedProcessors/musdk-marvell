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

/**
 * @file pp2_port_ks.c
 *
 * Port I/O routines - kernel space specific
 */

#include "std_internal.h"

#include "pp2_types.h"
#include "pp2.h"
#include "pp2_dm.h"
#include "pp2_port.h"


/* Helper function to access kernel network device */
struct net_device *pp2_port_get_netdev(struct pp2_port *port)
{
	return dev_get_by_name(&init_net, port->linux_name);
}

/* Port Control routines */

/* Set MAC address */
int pp2_port_set_mac_addr(struct pp2_port *port, const uint8_t *addr)
{
	struct net_device *netdev = pp2_port_get_netdev(port);
	struct sockaddr sa;

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	sa.sa_family = netdev->type;
	strncpy(sa.sa_data, addr, 6);

	if (!netdev->netdev_ops->ndo_set_mac_address) {
		pr_err("%s: Failed to find netdev op.\n", __func__);
		return -EINVAL;
	}

	return netdev->netdev_ops->ndo_set_mac_address(netdev, &sa);
}

/* Get MAC address */
int pp2_port_get_mac_addr(struct pp2_port *port, uint8_t *addr)
{
	struct net_device *netdev = pp2_port_get_netdev(port);

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	ether_addr_copy(addr, netdev->dev_addr);

	return 0;
}

/* Set Unicast promiscuous */
int pp2_port_set_promisc(struct pp2_port *port, uint32_t en)
{
	struct net_device *netdev = pp2_port_get_netdev(port);
	unsigned int new_flags;
	int rc;

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	if (en)
		new_flags = netdev->flags | IFF_PROMISC;
	else
		new_flags = netdev->flags & ~IFF_PROMISC;

	rtnl_lock();
	rc = dev_change_flags(netdev, new_flags);
	rtnl_unlock();

	return rc;
}

/* Check if unicast promiscuous */
int pp2_port_get_promisc(struct pp2_port *port, uint32_t *en)
{
	struct net_device *netdev = pp2_port_get_netdev(port);

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	*en = !!(netdev->flags & IFF_PROMISC);

	return 0;
}

/* Set Multicast promiscuous */
int pp2_port_set_mc_promisc(struct pp2_port *port, uint32_t en)
{
	struct net_device *netdev = pp2_port_get_netdev(port);
	unsigned int new_flags;
	int rc;

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	if (en)
		new_flags = netdev->flags | IFF_ALLMULTI;
	else
		new_flags = netdev->flags & ~IFF_ALLMULTI;

	rtnl_lock();
	rc = dev_change_flags(netdev, new_flags);
	rtnl_unlock();

	return rc;
}

/* Check if Multicast promiscuous */
int pp2_port_get_mc_promisc(struct pp2_port *port, uint32_t *en)
{
	struct net_device *netdev = pp2_port_get_netdev(port);

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	*en = !!(netdev->flags & IFF_ALLMULTI);

	return 0;
}

/* Add MAC address */
int pp2_port_add_mac_addr(struct pp2_port *port, const uint8_t *addr)
{
	struct net_device *netdev = pp2_port_get_netdev(port);

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	if (mv_check_eaddr_mc(addr))
		return dev_mc_add(netdev, addr);
	else
		return dev_uc_add(netdev, addr);
}

/* Remove MAC address */
int pp2_port_remove_mac_addr(struct pp2_port *port, const uint8_t *addr)
{
	struct net_device *netdev = pp2_port_get_netdev(port);

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	if (mv_check_eaddr_mc(addr))
		return dev_mc_del(netdev, addr);
	else
		return dev_uc_del(netdev, addr);
}

int pp2_port_flush_mac_addrs(struct pp2_port *port, uint32_t uc, uint32_t mc)
{
	struct net_device *netdev = pp2_port_get_netdev(port);

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	if (mc)
		dev_mc_flush(netdev);

	if (uc)
		dev_uc_flush(netdev);

	return 0;
}

/* Add vlan */
int pp2_port_add_vlan(struct pp2_port *port, u16 vlan)
{
	struct net_device *netdev = pp2_port_get_netdev(port);
	int rc;

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	rtnl_lock();
	rc = vlan_vid_add(netdev, cpu_to_be16((u16)0x8100), vlan);
	rtnl_unlock();

	return rc;
}

/* Remove vlan */
int pp2_port_remove_vlan(struct pp2_port *port, u16 vlan)
{
	struct net_device *netdev = pp2_port_get_netdev(port);

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	rtnl_lock();
	vlan_vid_del(netdev, cpu_to_be16((u16)0x8100), vlan);
	rtnl_unlock();

	return 0;
}

/* Get Link State */
int pp2_port_get_link_state(struct pp2_port *port, int  *en)
{
	struct net_device *netdev = pp2_port_get_netdev(port);

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	*en = !!(netdev->flags & IFF_RUNNING);

	return 0;
}

int pp2_port_get_rx_pause(struct pp2_port *port, int *en)
{
	struct net_device *netdev = pp2_port_get_netdev(port);
	struct ethtool_pauseparam param;

	memset(&param, 0, sizeof(struct ethtool_pauseparam));

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	if (!netdev->ethtool_ops->get_pauseparam) {
		pr_err("%s: Failed to find ethtool op.\n", __func__);
		return -EINVAL;
	}

	param.cmd = ETHTOOL_GPAUSEPARAM;
	netdev->ethtool_ops->get_pauseparam(netdev, &param);
	*en = param.rx_pause;

	return 0;
}

int pp2_port_set_rx_pause(struct pp2_port *port, int en)
{
	struct net_device *netdev = pp2_port_get_netdev(port);
	struct ethtool_pauseparam param;

	memset(&param, 0, sizeof(struct ethtool_pauseparam));

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	if (!netdev->ethtool_ops->set_pauseparam) {
		pr_err("%s: Failed to find ethtool op.\n", __func__);
		return -EINVAL;
	}

	param.cmd = ETHTOOL_SPAUSEPARAM;
	param.rx_pause = en;
	netdev->ethtool_ops->set_pauseparam(netdev, &param);

	return 0;
}

int pp2_port_initialize_statistics(struct pp2_port *port)
{
	struct net_device *netdev = pp2_port_get_netdev(port);
	uint32_t len;
	int rc;

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	if (!netdev->ethtool_ops->get_sset_count || !netdev->ethtool_ops->get_strings) {
		pr_err("%s: Failed to find ethtool op.\n", __func__);
		return -EINVAL;
	}

	rc = netdev->ethtool_ops->get_sset_count(netdev, ETH_SS_STATS);
	if (rc < 0) {
		pr_err("%s: unable to get stringset length\n", __func__);
		return rc;
	}

	len = rc;
	if (!len) {
		pr_err("%s: stringset length is zero\n", __func__);
		return -1;
	}

	port->stats_name = kcalloc(1, sizeof(struct ethtool_gstrings) + len * ETH_GSTRING_LEN, GFP_KERNEL);
	if (!port->stats_name)
		return -ENOMEM;

	port->stats_name->cmd = ETHTOOL_GSTRINGS;
	port->stats_name->string_set = ETH_SS_STATS;
	port->stats_name->len = len;
	netdev->ethtool_ops->get_strings(netdev, ETH_SS_STATS, port->stats_name->data);

	return 0;
}

int pp2_port_get_statistics(struct pp2_port *port, struct pp2_ppio_statistics *stats)
{
	struct net_device *netdev = pp2_port_get_netdev(port);
	struct ethtool_stats estats;
	u32 i;
	int n_stats;
	u64 *data;

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	if (!port->stats_name)
		return -1;

	if (!netdev->ethtool_ops->get_ethtool_stats || !netdev->ethtool_ops->get_sset_count) {
		pr_err("%s: Failed to find ethtool op.\n", __func__);
		return -EOPNOTSUPP;
	}

	n_stats = netdev->ethtool_ops->get_sset_count(netdev, ETH_SS_STATS);

	if (n_stats < 0) {
		pr_err("Failed to get number of statistics strings.\n");
		return n_stats;
	}

	if (n_stats > S32_MAX / sizeof(u64)) {
		pr_err("Too many statistics strings.\n");
		return -ENOMEM;
	}

	data = kcalloc(n_stats, sizeof(u64), GFP_KERNEL);

	if (!data)
		return -ENOMEM;

	estats.n_stats = n_stats;
	estats.cmd = ETHTOOL_GSTATS;

	netdev->ethtool_ops->get_ethtool_stats(netdev, &estats, data);

	for (i = 0; i < n_stats; i++) {
		char *cnt = (char *)&port->stats_name->data[i * ETH_GSTRING_LEN];
		uint64_t val = data[i];

		if (!strcmp(cnt, "rx_bytes"))
			stats->rx_bytes = val;
		else if (!strcmp(cnt, "rx_frames"))
			stats->rx_packets = val;
		else if (!strcmp(cnt, "rx_unicast"))
			stats->rx_unicast_packets = val;
		else if (!strcmp(cnt, "rx_ppv2_overrun"))
			stats->rx_fifo_dropped = val;
		else if (!strcmp(cnt, "rx_cls_drop"))
			stats->rx_cls_dropped = val;
		else if (!strcmp(cnt, "rx_total_err"))
			stats->rx_errors = val;
		else if (!strcmp(cnt, "tx_bytes"))
			stats->tx_bytes = val;
		else if (!strcmp(cnt, "tx_frames"))
			stats->tx_packets = val;
		else if (!strcmp(cnt, "tx_unicast"))
			stats->tx_unicast_packets = val;
		else if (!strcmp(cnt, "collision"))
			stats->tx_errors += val;
		else if (!strcmp(cnt, "late_collision"))
			stats->tx_errors += val;
		else if (!strcmp(cnt, "tx_crc_sent"))
			stats->tx_errors += val;
	}

	return 0;
}

/* Set Port enabled */
int pp2_port_set_enable(struct pp2_port *port, uint32_t en)
{
	struct net_device *netdev = pp2_port_get_netdev(port);
	int err = 0;

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}
	rtnl_lock();
	if (en)
		err = dev_open(netdev);
	else
		dev_close(netdev);
	rtnl_unlock();

	if (err < 0)  {
		pr_err("%s: Failed to set network device '%s' to %s.\n", __func__, port->linux_name,
			(en) ? "enabled" : "disabled");
		return -EINVAL;
	}

	return 0;
}

/* Check if Port enabled */
int pp2_port_get_enable(struct pp2_port *port, uint32_t *en)
{
	struct net_device *netdev = pp2_port_get_netdev(port);

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}

	*en = !!(netdev->flags & IFF_UP);
	return 0;
}

int pp2_port_open_uio(struct pp2_port *port)
{
	struct net_device *netdev = pp2_port_get_netdev(port);
	int err;

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}
	err = mv_pp2x_port_musdk_set(netdev_priv(netdev));
	if (err)
		pr_err("%s: Failed to set musdk_mode on network device '%s'", __func__, port->linux_name);
	return err;
}
int pp2_port_close_uio(struct pp2_port *port)
{
	struct net_device *netdev = pp2_port_get_netdev(port);
	int err;

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}
	err = mv_pp2x_port_musdk_clear(netdev_priv(netdev));

	return err;
}
int pp2_port_set_priv_flags(struct pp2_port *port, u32 val)
{
	struct net_device *netdev = pp2_port_get_netdev(port);
	int err;

	if (!netdev) {
		pr_err("%s: Failed to locate network device '%s'.\n", __func__, port->linux_name);
		return -EINVAL;
	}
	if (!netdev->ethtool_ops->set_priv_flags) {
		pr_err("%s: Failed to find ethtool op.\n", __func__);
		return -EINVAL;
	}
	err = netdev->ethtool_ops->set_priv_flags(netdev, val);

	return err;
}
