/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"
#include "lib/net.h"
#include "lib/list.h"
#include "drivers/mv_neta_ppio.h"
#include "drivers/mv_neta.h"
#include "neta_ppio.h"
#include "neta_hw.h"

#define MAX_BUF_STR_LEN		256

/* Port Control routines */
static int parse_hex(char *str, u8 *addr, size_t size)
{
	int len = 0;

	while (*str && (len < 2 * size)) {
		int tmp;

		if (str[1] == 0)
			return -1;
		if (sscanf(str, "%02x", &tmp) != 1)
			return -1;
		addr[len] = tmp;
		len++;
		str += 2;
	}
	return len;
}

static void neta_port_uc_mac_addr_list_reconfig(struct neta_port *port)
{
	struct neta_port_uc_addr_node *uc_addr_node;

	LIST_FOR_EACH_OBJECT(uc_addr_node, struct neta_port_uc_addr_node,
			     &port->added_uc_addr, list_node) {
		neta_add_ucast_addr(port, uc_addr_node->addr[5], MVNETA_DEFAULT_RXQ);
		pr_debug("add Eth addr %x:%x:%x:%x:%x:%x added to uc mac table\n",
			 uc_addr_node->addr[0], uc_addr_node->addr[1],
			 uc_addr_node->addr[2], uc_addr_node->addr[3],
			 uc_addr_node->addr[4], uc_addr_node->addr[5]);
	}
}

static void neta_port_uc_mac_addr_list_flush(struct neta_port *port)
{
	struct neta_port_uc_addr_node *uc_addr_node, *tmp;

	LIST_FOR_EACH_OBJECT_SAFE(uc_addr_node, tmp, &port->added_uc_addr,
				  struct neta_port_uc_addr_node, list_node) {
		pr_debug("removed %x:%x:%x:%x:%x:%x from port_list\n",
			uc_addr_node->addr[0], uc_addr_node->addr[1],
			uc_addr_node->addr[2], uc_addr_node->addr[3],
			uc_addr_node->addr[4], uc_addr_node->addr[5]);

		list_del(&uc_addr_node->list_node);
		kfree(uc_addr_node);
	}
}

static int neta_port_uc_mac_addr_list_remove(struct neta_port *port, const uint8_t *addr)
{
	struct neta_port_uc_addr_node *uc_addr_node;

	LIST_FOR_EACH_OBJECT(uc_addr_node, struct neta_port_uc_addr_node,
			     &port->added_uc_addr, list_node) {
		if (mv_eaddr_identical(uc_addr_node->addr, addr)) {
			list_del(&uc_addr_node->list_node);
			kfree(uc_addr_node);
			pr_debug("removed %x:%x:%x:%x:%x:%x from port_list\n",
				 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
			return 1;
		}
	}
	return 0;
}

static int neta_port_uc_mac_addr_list_exist(struct neta_port *port, const uint8_t *addr)
{
	struct neta_port_uc_addr_node *uc_addr_node;

	LIST_FOR_EACH_OBJECT(uc_addr_node, struct neta_port_uc_addr_node,
			     &port->added_uc_addr, list_node) {
		if (mv_eaddr_identical(uc_addr_node->addr, addr))
			return 1;
	}
	return 0;
}

/* Set MAC address */
int neta_port_set_mac_addr(struct neta_port *port, const uint8_t *addr)
{
	int rc = 0;
	struct ifreq s;
	int i;

	if (!mv_check_eaddr_valid(addr)) {
		pr_err("PORT: not a valid eth address\n");
		return -EINVAL;
	}

	strcpy(s.ifr_name, port->if_name);
	s.ifr_hwaddr.sa_family = ARPHRD_ETHER;

	for (i = 0; i < ETH_ALEN; i++)
		s.ifr_hwaddr.sa_data[i] = addr[i];

	rc = mv_netdev_ioctl(SIOCSIFHWADDR, &s);
	if (rc)
		return rc;

	mv_cp_eaddr(port->mac, (const uint8_t *)addr);

	/* flush uc mac table:
	 * internal DB is flushed. HW regs will be flushed only if
	 * the port is not in promisc mode
	 */
	if (!port->is_promisc)
		neta_flush_ucast_table(port);
	neta_port_uc_mac_addr_list_flush(port);
	port->num_added_uc_addr = 0;

	/* add primary address to uc mac table */
	neta_port_add_mac_addr(port, addr);

	return 0;
}

/* Get MAC address */
int neta_port_get_mac_addr(struct neta_port *port, uint8_t *addr)
{
	int rc;
	struct ifreq s;
	int i;

	strcpy(s.ifr_name, port->if_name);
	rc = mv_netdev_ioctl(SIOCGIFHWADDR, &s);
	if (rc)
		return rc;

	for (i = 0; i < ETH_ALEN; i++)
		addr[i] = s.ifr_hwaddr.sa_data[i];

	return 0;
}

/* Get Link State */
int neta_port_get_link_state(struct neta_port *port, int  *en)
{
	int rc;
	struct ifreq s;

	strcpy(s.ifr_name, port->if_name);
	rc = mv_netdev_ioctl(SIOCGIFFLAGS, &s);
	if (rc)
		return rc;

	(s.ifr_flags & IFF_RUNNING) ? (*en = 1) : (*en = 0);
	return 0;
}

/* Set promiscuous */
int neta_port_set_promisc(struct neta_port *port, uint32_t en)
{
	int rc;
	struct ifreq s;

	strcpy(s.ifr_name, port->if_name);
	rc = mv_netdev_ioctl(SIOCGIFFLAGS, &s);
	if (rc) {
		pr_err("PORT: unable to read promisc mode from HW\n");
		return rc;
	}

	if (en)
		s.ifr_flags |= IFF_PROMISC;
	else
		s.ifr_flags &= ~IFF_PROMISC;

	rc = mv_netdev_ioctl(SIOCSIFFLAGS, &s);
	if (rc) {
		pr_err("PORT: unable to set promisc mode to HW\n");
		return rc;
	}

	port->is_promisc = en ? 1 : 0;

	if (en == 0)
		/* go through UC mac list and reconfigure all entries again */
		neta_port_uc_mac_addr_list_reconfig(port);

	return 0;
}

/* Check if promiscuous */
int neta_port_get_promisc(struct neta_port *port, uint32_t *en)
{
	int rc;
	struct ifreq s;

	*en = 0;

	strcpy(s.ifr_name, port->if_name);
	rc = mv_netdev_ioctl(SIOCGIFFLAGS, &s);
	if (rc) {
		pr_err("PORT: unable to read promisc mode from HW\n");
		return rc;
	}

	if (s.ifr_flags & IFF_PROMISC)
		*en = 1;
	return 0;
}

/* Set Multicast promiscuous */
int neta_port_set_mc_promisc(struct neta_port *port, uint32_t en)
{
	int rc;
	struct ifreq s;

	strcpy(s.ifr_name, port->if_name);
	rc = mv_netdev_ioctl(SIOCGIFFLAGS, &s);
	if (rc) {
		pr_err("PORT: unable to read all-multicast mode from HW\n");
		return rc;
	}

	if (en)
		s.ifr_flags |= IFF_ALLMULTI;
	else
		s.ifr_flags &= ~IFF_ALLMULTI;

	rc = mv_netdev_ioctl(SIOCSIFFLAGS, &s);
	if (rc) {
		pr_err("PORT: unable to set all-multicast mode to HW\n");
		return rc;
	}
	return 0;
}

/* Check if Multicast promiscuous */
int neta_port_get_mc_promisc(struct neta_port *port, uint32_t *en)
{
	int rc;
	struct ifreq s;

	*en = 0;
	strcpy(s.ifr_name, port->if_name);
	rc = mv_netdev_ioctl(SIOCGIFFLAGS, &s);
	if (rc) {
		pr_err("PORT: unable to read all-multicast mode from HW\n");
		return rc;
	}

	if (s.ifr_flags & IFF_ALLMULTI)
		*en = 1;
	return 0;
}

/* Add MAC address */
int neta_port_add_mac_addr(struct neta_port *port, const uint8_t *addr)
{
	int rc;
	struct neta_port_uc_addr_node *uc_addr_node;

	if (mv_check_eaddr_mc(addr)) {
		struct ifreq s;
		int i;

		strcpy(s.ifr_name, port->if_name);
		s.ifr_hwaddr.sa_family = AF_UNSPEC;
		for (i = 0; i < ETH_ALEN; i++)
			s.ifr_hwaddr.sa_data[i] = addr[i];

		rc = mv_netdev_ioctl(SIOCADDMULTI, &s);
		if (rc) {
			pr_err("PORT: unable to add mac sddress\n");
			return rc;
		}
		pr_debug("PORT: Ethernet address %x:%x:%x:%x:%x:%x added to mc list\n",
			 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
	} else {
		/* Verify that MAC address is the same as the primary MAC */
		/* except 4 LSB bits */
		if (mv_eaddr_identical_except_4LSB(port->mac, addr) == 0) {
			pr_err("PORT: uc mac address should be equal to the primary mac address except 4 LSB bits\n");
			return -1;
		}
		if (neta_port_uc_mac_addr_list_exist(port, addr)) {
			pr_debug("PORT: uc mac address already exists\n");
			return 0;
		}

		/* Add MAC to uc mac list */
		uc_addr_node = kmalloc(sizeof(*uc_addr_node), GFP_KERNEL);
		if (!uc_addr_node)
			return -ENOMEM;

		mv_cp_eaddr(uc_addr_node->addr, addr);
		list_add_to_tail(&uc_addr_node->list_node,
				 &port->added_uc_addr);
		port->num_added_uc_addr++;

		/* Add address to Unicast Table */
		neta_add_ucast_addr(port, addr[5], MVNETA_DEFAULT_RXQ);

		pr_debug("PORT: Ethernet address %x:%x:%x:%x:%x:%x added to uc list\n",
			 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		pr_debug("PORT: num_added_uc_addr:%d\n", port->num_added_uc_addr);
	}
	return 0;
}

/* Remove MAC address */
int neta_port_remove_mac_addr(struct neta_port *port, const uint8_t *addr)
{
	int rc;

	if (mv_check_eaddr_mc(addr)) {
		struct ifreq s;
		int i;

		strcpy(s.ifr_name, port->if_name);
		s.ifr_hwaddr.sa_family = AF_UNSPEC;
		for (i = 0; i < ETH_ALEN; i++)
			s.ifr_hwaddr.sa_data[i] = addr[i];

		rc = mv_netdev_ioctl(SIOCDELMULTI, &s);
		if (rc) {
			pr_err("PORT: unable to remove mac sddress\n");
			return rc;
		}
		pr_debug("PORT: Ethernet address %x:%x:%x:%x:%x:%x removed from mc list\n",
			 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
	} else {
		/* Check if MAC address to remove exists in uc list */
		if (!neta_port_uc_mac_addr_list_remove(port, addr)) {
			pr_err("PORT: Ethernet address %x:%x:%x:%x:%x:%x doesn't exist\n",
			       addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
			return -1;
		}

		port->num_added_uc_addr--;

		/* Remove address from Unicast Table */
		neta_del_ucast_addr(port, addr[5]);

		pr_debug("PORT: Ethernet address %x:%x:%x:%x:%x:%x removed from uc list\n",
			 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		pr_debug("PORT: num_added_uc_addr:%d\n", port->num_added_uc_addr);
	}
	return 0;
}

int neta_port_flush_mac_addrs(struct neta_port *port)
{
	int rc;
	char buf[MAX_BUF_STR_LEN];
	char name[IFNAMSIZ];
	char addr_str[MAX_BUF_STR_LEN];
	u8 mac[ETH_ALEN];
	FILE *fp = fopen("/proc/net/dev_mcast", "r");
	int len = 0;
	int st;

	if (!fp)
		return -EACCES;

	while (fgets(buf, sizeof(buf), fp)) {
		if (sscanf(buf, "%*d%s%*d%d%s", name, &st, addr_str) != 3) {
			pr_err("address not found in file\n");
			return -EFAULT;
		}

		if ((strcmp(port->if_name, name)) || (!st))
			continue;

		len = parse_hex(addr_str, mac, ETH_ALEN);
		if (len != ETH_ALEN) {
			pr_err("len parsing error\n");
			return -EFAULT;
		}

		rc = neta_port_remove_mac_addr(port, mac);
		if (rc)
			return rc;
	}
	fclose(fp);

	/* flush uc mac table */
	neta_flush_ucast_table(port);
	neta_port_uc_mac_addr_list_flush(port);
	port->num_added_uc_addr = 0;

	return 0;
}

int neta_port_initialize_statistics(struct neta_port *port)
{
	struct {
		struct ethtool_sset_info hdr;
		uint32_t buf[1];
	} sset_info;
	uint32_t len = 0;
	struct ifreq ifr;
	int rc;

	strcpy(ifr.ifr_name, port->if_name);

	sset_info.hdr.cmd = ETHTOOL_GSSET_INFO;
	sset_info.hdr.reserved = 0;
	sset_info.hdr.sset_mask = 1ULL << ETH_SS_STATS;
	ifr.ifr_data =  &sset_info;
	rc = mv_netdev_ioctl(SIOCETHTOOL, &ifr);
	if (rc) {
		pr_err("PORT: unable to get stringset length\n");
		return -1;
	}
	if (sset_info.hdr.sset_mask)
		memcpy(&len, sset_info.hdr.data, sizeof(uint32_t));

	if (!len) {
		pr_err("PORT: stringset length is zero\n");
		return -1;
	}

	port->stats_name = calloc(1, sizeof(struct ethtool_gstrings) + len * ETH_GSTRING_LEN);
	if (!port->stats_name) {
		pr_err("PORT: alloc failed\n");
		return -1;
	}

	port->stats_name->cmd = ETHTOOL_GSTRINGS;
	port->stats_name->string_set = ETH_SS_STATS;
	port->stats_name->len = len;
	ifr.ifr_data = port->stats_name;
	rc = mv_netdev_ioctl(SIOCETHTOOL, &ifr);
	if (rc) {
		free(port->stats_name);
		port->stats_name = NULL;
		pr_err("PORT: unable to get stringset\n");
		return -1;
	}

	return 0;
}

int neta_port_get_statistics(struct neta_port *port, struct neta_ppio_statistics *stats)
{
	struct ifreq ifr;
	struct ethtool_stats *estats;
	u32 i;
	int rc;

	if (!port->stats_name)
		return -1;

	estats = calloc(1, port->stats_name->len * sizeof(uint64_t) +
			sizeof(struct ethtool_stats));
	if (!estats) {
		pr_err("PORT: alloc failed\n");
		return -1;
	}

	strcpy(ifr.ifr_name, port->if_name);

	estats->cmd = ETHTOOL_GSTATS;
	estats->n_stats = port->stats_name->len;
	ifr.ifr_data = estats;
	rc = mv_netdev_ioctl(SIOCETHTOOL, &ifr);
	if (rc) {
		pr_err("PORT: unable to get statistics\n");
		free(estats);
		return rc;
	}

	for (i = 0; i < port->stats_name->len; i++) {
		char *cnt = (char *)&port->stats_name->data[i * ETH_GSTRING_LEN];
		uint64_t val = estats->data[i];

		if (!strcmp(cnt, "good_octets_received"))
			stats->rx_bytes = val;
		else if (!strcmp(cnt, "good_frames_received"))
			stats->rx_packets = val;
		else if (!strcmp(cnt, "bad_octets_received"))
			stats->rx_bytes_err = val;
		else if (!strcmp(cnt, "bad_frames_received"))
			stats->rx_packets_err = val;
		else if (!strcmp(cnt, "broadcast_frames_received"))
			stats->rx_broadcast_packets = val;
		else if (!strcmp(cnt, "multicast_frames_received"))
			stats->rx_multicast_packets = val;
		else if (!strcmp(cnt, "undersize_received"))
			stats->rx_undersize = val;
		else if (!strcmp(cnt, "fragments_received"))
			stats->rx_fragments = val;
		else if (!strcmp(cnt, "oversize_received"))
			stats->rx_oversize = val;
		else if (!strcmp(cnt, "mac_receive_error"))
			stats->rx_errors = val;
		else if (!strcmp(cnt, "bad_crc_event"))
			stats->rx_crc_error = val;
		else if (!strcmp(cnt, "rx_discard"))
			stats->rx_discard = val;
		else if (!strcmp(cnt, "rx_overrun"))
			stats->rx_overrun = val;
		else if (!strcmp(cnt, "good_octets_sent"))
			stats->tx_bytes = val;
		else if (!strcmp(cnt, "good_frames_sent"))
			stats->tx_packets = val;
		else if (!strcmp(cnt, "broadcast_frames_sent"))
			stats->tx_broadcast_packets = val;
		else if (!strcmp(cnt, "multicast_frames_sent"))
			stats->tx_multicast_packets = val;
		else if (!strcmp(cnt, "internal_mac_transmit_err"))
			stats->tx_errors = val;
	}

	free(estats);

	return 0;
}
