/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pp2_port.c
 *
 * Port I/O routines - user space specific
 */

#include "std_internal.h"

#include "pp2_types.h"

#include "pp2_hw_type.h"
#include "pp2.h"
#include "drivers/mv_pp2_bpool.h"
#include "pp2_dm.h"
#include "pp2_port.h"
#include "pp2_bm.h"
#include "cls/pp2_hw_cls.h"
#include "cls/pp2_prs.h"
#include "lib/uio_helper.h"

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

/* Set MAC address */
int pp2_port_set_mac_addr(struct pp2_port *port, const uint8_t *addr)
{
	int rc = 0;
	struct ifreq s;
	int i;

	if (!mv_check_eaddr_valid(addr)) {
		pr_err("PORT: not a valid eth address\n");
		return -EINVAL;
	}

	strcpy(s.ifr_name, port->linux_name);
	s.ifr_hwaddr.sa_family = ARPHRD_ETHER;

	for (i = 0; i < ETH_ALEN; i++)
		s.ifr_hwaddr.sa_data[i] = addr[i];

	rc = mv_netdev_ioctl(SIOCSIFHWADDR, &s);
	if (rc)
		return rc;

	mv_cp_eaddr(port->mac_data.mac, (const uint8_t *)addr);
	return 0;
}

/* Get MAC address */
int pp2_port_get_mac_addr(struct pp2_port *port, uint8_t *addr)
{
	int rc;
	struct ifreq s;
	int i;

	strcpy(s.ifr_name, port->linux_name);
	rc = mv_netdev_ioctl(SIOCGIFHWADDR, &s);
	if (rc)
		return rc;

	for (i = 0; i < ETH_ALEN; i++)
		addr[i] = s.ifr_hwaddr.sa_data[i];
	return 0;
}

/* Get Link State */
int pp2_port_get_link_state(struct pp2_port *port, int  *en)
{
	int rc;
	struct ifreq s;

	strcpy(s.ifr_name, port->linux_name);
	rc = mv_netdev_ioctl(SIOCGIFFLAGS, &s);
	if (rc)
		return rc;

	(s.ifr_flags & IFF_RUNNING) ? (*en = 1) : (*en = 0);
	return 0;
}

/* Get Rx Pause FC status */
int pp2_port_get_rx_pause(struct pp2_port *port, int *en)
{
	*en = port->rx_pause_en;
	pr_debug("PORT: rx pause is %s\n", (*en) ? "enabled" : "disabled");
	return 0;
}

/* Set Rx Pause FC */
int pp2_port_set_rx_pause(struct pp2_port *port, int en)
{
	struct ifreq ifr;
	struct ethtool_pauseparam param;
	int rc;

	if (port->rx_pause_en == en)
		return 0;

	memset(&param, 0, sizeof(param));
	strcpy(ifr.ifr_name, port->linux_name);

	param.cmd = ETHTOOL_SPAUSEPARAM;
	param.rx_pause = en;
	param.tx_pause = port->tx_pause_en;
	param.autoneg = 1;
	ifr.ifr_data = &param;
	rc = mv_netdev_ioctl(SIOCETHTOOL, &ifr);
	if (rc) {
		pr_err("PORT: unable to %s rx pause\n",
			(en) ? "enable" : "disable");
		return rc;
	}

	port->rx_pause_en = en;
	pr_debug("PORT: rx pause is %s\n", (en) ? "enabled" : "disabled");
	return 0;
}

/* Enable Tx Pause FC in Phy */
int pp2_port_set_phy_tx_pause(struct pp2_port *port, int en)
{
	struct ifreq ifr;
	struct ethtool_pauseparam param;
	int rc;

	memset(&param, 0, sizeof(param));
	strcpy(ifr.ifr_name, port->linux_name);

	param.cmd = ETHTOOL_SPAUSEPARAM;
	param.tx_pause = en;
	param.rx_pause = port->rx_pause_en;
	param.autoneg = 1;
	ifr.ifr_data = &param;
	rc = mv_netdev_ioctl(SIOCETHTOOL, &ifr);
	if (rc) {
		pr_err("PORT: unable to %s tx pause\n",
			(en) ? "enable" : "disable");
		return rc;
	}

	return 0;
}

/* Set promiscuous */
int pp2_port_set_promisc(struct pp2_port *port, uint32_t en)
{
	int rc;
	struct ifreq s;

	strcpy(s.ifr_name, port->linux_name);
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
	return 0;
}

/* Check if promiscuous */
int pp2_port_get_promisc(struct pp2_port *port, uint32_t *en)
{
	int rc;
	struct ifreq s;

	*en = 0;

	strcpy(s.ifr_name, port->linux_name);
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
int pp2_port_set_mc_promisc(struct pp2_port *port, uint32_t en)
{
	int rc;
	struct ifreq s;

	strcpy(s.ifr_name, port->linux_name);
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
int pp2_port_get_mc_promisc(struct pp2_port *port, uint32_t *en)
{
	int rc;
	struct ifreq s;

	*en = 0;
	strcpy(s.ifr_name, port->linux_name);
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
int pp2_port_add_mac_addr(struct pp2_port *port, const uint8_t *addr)
{
	int rc;
	struct port_uc_addr_node *uc_addr_node;

	if (mv_check_eaddr_mc(addr)) {
		struct ifreq s;
		int i;

		if (port->num_added_mc_addr == PP2_PPIO_MAX_MC_ADDR) {
			pr_err("PORT: reached multicast address limit (%d)\n", PP2_PPIO_MAX_MC_ADDR);
			return -ENOSPC;
		}

		strcpy(s.ifr_name, port->linux_name);
		s.ifr_hwaddr.sa_family = AF_UNSPEC;
		for (i = 0; i < ETH_ALEN; i++)
			s.ifr_hwaddr.sa_data[i] = addr[i];

		rc = mv_netdev_ioctl(SIOCADDMULTI, &s);
		if (rc) {
			pr_err("PORT: unable to add mac sddress\n");
			return rc;
		}
		port->num_added_mc_addr++;

		pr_debug("PORT: Ethernet address %x:%x:%x:%x:%x:%x added to mc list\n",
			 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		pr_debug("num_mc:%d\n", port->num_added_mc_addr);
	} else if (mv_check_eaddr_uc(addr)) {
		int fd;
		char buf[PP2_MAX_BUF_STR_LEN];
		char da[PP2_MAX_BUF_STR_LEN];

		if (port->num_added_uc_addr == PP2_PPIO_MAX_UC_ADDR) {
			pr_err("PORT: reached unicast address limit (%d)\n", PP2_PPIO_MAX_UC_ADDR);
			return -ENOSPC;
		}
		uc_addr_node = kmalloc(sizeof(*uc_addr_node), GFP_KERNEL);
		if (!uc_addr_node)
			return -ENOMEM;
		mv_cp_eaddr(uc_addr_node->addr, addr);

		strcpy(buf, port->linux_name);
		sprintf(da, " %x:%x:%x:%x:%x:%x", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		strcat(buf, da);
		fd = open("/sys/devices/platform/pp2/debug/uc_filter_add", O_WRONLY);
		if (fd == -1) {
			pr_debug("PORT: unable to open sysfs, updating prs_table internally instead\n");
			mv_pp2x_prs_mac_da_accept(port, addr, true);
		} else {
			rc = write(fd, &buf, strlen(buf) + 1);
			close(fd);
			if (rc < 0) {
				pr_err("PORT: unable to write to sysfs\n");
				kfree(uc_addr_node);
				return -EFAULT;
			}
		}

		/* Add uc_address to list */
		list_add_to_tail(&uc_addr_node->list_node, &port->added_uc_addr);
		port->num_added_uc_addr++;

		pr_debug("PORT: Ethernet address %x:%x:%x:%x:%x:%x added to uc list\n",
			 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		pr_debug("num_uc:%d\n", port->num_added_uc_addr);
	} else {
		pr_err("PORT: Ethernet address is not unicast/multicast. Request ignored\n");
	}
	return 0;
}

static int pp2_port_uc_mac_addr_check(struct pp2_port *port, const uint8_t *addr)
{
	struct port_uc_addr_node *uc_addr_node;

	LIST_FOR_EACH_OBJECT(uc_addr_node, struct port_uc_addr_node,
			     &port->added_uc_addr, list_node) {
			if (mv_eaddr_identical(uc_addr_node->addr, addr))
				return 1;
	}

	return 0;
}


static int pp2_port_uc_mac_addr_list_remove(struct pp2_port *port, const uint8_t *addr)
{
	struct port_uc_addr_node *uc_addr_node;

	LIST_FOR_EACH_OBJECT(uc_addr_node, struct port_uc_addr_node,
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


/* Remove MAC address */
int pp2_port_remove_mac_addr(struct pp2_port *port, const uint8_t *addr)
{
	int rc;

	if (mv_check_eaddr_mc(addr)) {
		struct ifreq s;
		int i;

		strcpy(s.ifr_name, port->linux_name);
		s.ifr_hwaddr.sa_family = AF_UNSPEC;
		for (i = 0; i < ETH_ALEN; i++)
			s.ifr_hwaddr.sa_data[i] = addr[i];

		rc = mv_netdev_ioctl(SIOCDELMULTI, &s);
		if (rc) {
			pr_err("PORT: unable to remove mac sddress\n");
			return rc;
		}
		port->num_added_mc_addr--;
		pr_debug("PORT: Ethernet address %x:%x:%x:%x:%x:%x removed from mc list\n",
			 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		pr_debug("num_mc:%d\n", port->num_added_mc_addr);
	} else if (mv_check_eaddr_uc(addr)) {
		int fd;
		char buf[PP2_MAX_BUF_STR_LEN];
		char da[PP2_MAX_BUF_STR_LEN];

		strcpy(buf, port->linux_name);
		sprintf(da, " %x:%x:%x:%x:%x:%x", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		strcat(buf, da);
		fd = open("/sys/devices/platform/pp2/debug/uc_filter_del", O_WRONLY);
		if (fd == -1) {
			pr_debug("PORT: unable to open sysfs, updating prs_table internally instead\n");
			mv_pp2x_prs_mac_da_accept(port, addr, false);
		} else {
			rc = write(fd, &buf, strlen(buf) + 1);
			close(fd);
			if (rc < 0) {
				pr_err("PORT: unable to write to sysfs\n");
				return -EFAULT;
			}
		}

		pp2_port_uc_mac_addr_list_remove(port, addr);
		port->num_added_uc_addr--;

		pr_debug("PORT: Ethernet address %x:%x:%x:%x:%x:%x removed from uc list\n",
			 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		pr_debug("num_uc:%d\n", port->num_added_uc_addr);
	} else {
		pr_err("PORT: Ethernet address is not unicast/multicast. Request ignored\n");
	}
	return 0;
}

static int pp2_port_clear_kernel_unicast(struct pp2_port *port)
{
	char name[IFNAMSIZ];
	char command[PP2_MAX_BUF_STR_LEN], full_name[32];
	struct dirent *dent;
	char *dir;
	DIR *d;
	int rc;
	u8 id;

	d = opendir("/sys/class/net/");
	if (!d)
		return -EACCES;

	while (1) {
		dent = readdir(d);
		if (!dent)
			break;
		dir = dent->d_name;

		if (!strcmp(dir, ".") || !strcmp(dir, ".."))
			continue;

		if (sscanf(dir, "%[^.].%02hhu", name, &id) != 2)
			continue;

		if (strcmp(port->linux_name, name))
			continue;

		sprintf(full_name, "%s.%u\n", name, id);
		sprintf(command, "ip -d link show %s | grep macvlan", full_name);
		rc = system(command);
		if (rc == 0)
			/* mac interface found */
			/* same function can be used to remove macvlan interface */
			pp2_port_clear_vlan(port, id);
	}

	return 0;
}

int pp2_port_flush_mac_addrs(struct pp2_port *port, uint32_t uc, uint32_t mc)
{
	int rc;
	u8 mac[ETH_ALEN];
	struct port_uc_addr_node *uc_addr_node;

	if (mc) {
		char buf[PP2_MAX_BUF_STR_LEN];
		char name[IFNAMSIZ];
		char addr_str[PP2_MAX_BUF_STR_LEN];
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

			if ((strcmp(port->linux_name, name)) || (!st))
				continue;

			len = parse_hex(addr_str, mac, ETH_ALEN);
			if (len != ETH_ALEN) {
				pr_err("len parsing error\n");
				return -EFAULT;
			}

			rc = pp2_port_remove_mac_addr(port, mac);
			if (rc)
				return rc;
		}
		fclose(fp);
	}

	if (uc) {
		int fd;
		char buf[PP2_MAX_BUF_STR_LEN];

		strcpy(buf, port->linux_name);
		fd = open("/sys/devices/platform/pp2/debug/uc_filter_flush", O_WRONLY);
		if (fd == -1) {
			pr_debug("PORT: unable to open sysfs, updating prs_table internally instead\n");
			while (!list_is_empty(&port->added_uc_addr)) {
				uc_addr_node = LIST_FIRST_OBJECT((&port->added_uc_addr),
								 struct port_uc_addr_node, list_node);
				pp2_port_remove_mac_addr(port, uc_addr_node->addr);
			}
			pp2_port_clear_kernel_unicast(port);
		} else {
			rc = write(fd, &buf, strlen(buf) + 1);
			close(fd);
			if (rc < 0) {
				pr_err("PORT: unable to write to sysfs\n");
				return -EFAULT;
			}
		}

	}
	return 0;
}

/* Set vlan filtering */
int pp2_port_set_vlan_filtering(struct pp2_port *port, int enable)
{
	const char *featstr = "rx-vlan-filter";
	int rc;

	if (!enable && port->num_vlans != 0) {
		pr_err("disable vlan filtering not allowed until all the vlans removed\n");
		return -EPERM;
	}

	rc = mv_netdev_feature_set(port->linux_name, featstr, enable);
	if (rc != 0) {
		if (enable)
			pr_err("failed to enable vlan filtering\n");
		else
			pr_err("failed to disable vlan filtering\n");

		return rc;
	}

	port->vlan_enable = enable;
	return 0;
}

/* Add vlan */
int pp2_port_add_vlan(struct pp2_port *port, u16 vlan)
{
	int rc, vidx, vbit;

	char buf[PP2_MAX_BUF_STR_LEN];

	if (vlan >= 4095) {
		pr_err("invalid vid (%d). Range: 0:4094\n", vlan);
		return -EINVAL;
	}

	if ((port->num_vlans == 0) && !port->vlan_enable) {
		pr_err("vlan filtering should be set before adding vlan\n");
		return -EPERM;
	}

	/* build manually the system command */
	/* [TODO] check other alternatives for setting vlan id */
	sprintf(buf, "ip link add link %s name %s.%d type vlan id %d", port->linux_name, port->linux_name, vlan, vlan);
	rc = system(buf);
	if (rc != 0) {
		pr_err("add vlan operation failed\n");
		return rc;
	}

	vidx = vlan / 64;
	vbit = vlan % 64;
	port->vlan_ids[vidx] |= UINT64_C(1) << vbit;

	port->num_vlans++;
	return 0;
}

/* Remove vlan */
int pp2_port_remove_vlan(struct pp2_port *port, u16 vlan)
{
	int rc, vidx, vbit;
	char buf[PP2_MAX_BUF_STR_LEN];

	if (vlan >= 4095) {
		pr_err("invalid vid (%d). Range: 0:4094\n", vlan);
		return -EINVAL;
	}

	if (port->num_vlans == 0) {
		pr_err("no vlans configured\n");
		return -EINVAL;
	}

	/* build manually the system command */
	/* [TODO] check other alternatives for setting vlan id */
	sprintf(buf, "ip link delete %s.%d", port->linux_name, vlan);
	rc = system(buf);
	if (rc != 0) {
		pr_err("remove vlan operation failed\n");
		return rc;
	}

	vidx = vlan / 64;
	vbit = vlan % 64;
	port->vlan_ids[vidx] &= ~(UINT64_C(1) << vbit);

	port->num_vlans--;
	return 0;
}

/* Clear vlan */
int pp2_port_clear_vlan(struct pp2_port *port, u16 vlan)
{
	int rc;
	char buf[PP2_MAX_BUF_STR_LEN];

	/* build manually the system command */
	/* [TODO] check other alternatives for setting vlan id */
	sprintf(buf, "ip link delete %s.%d", port->linux_name, vlan);
	rc = system(buf);
	if (rc != 0) {
		pr_err("clear vlan operation failed\n");
		return rc;
	}

	return 0;
}

/* Clear all vlans */
int pp2_port_clear_all_vlans(struct pp2_port *port)
{
	int vidx, vbit, rc;
	uint16_t vlan_id;

	for (vlan_id = 0; vlan_id < 4095; vlan_id++) {
		vidx = vlan_id / 64;
		vbit = vlan_id % 64;

		/* Each bit corresponds to a VLAN id */
		if (port->vlan_ids[vidx] & (UINT64_C(1) << vbit)) {
			rc = pp2_port_remove_vlan(port, vlan_id);
			if (rc)
				return rc;
		}
	}

	return 0;
}

int pp2_port_initialize_statistics(struct pp2_port *port)
{
	struct {
		struct ethtool_sset_info hdr;
		uint32_t buf[1];
	} sset_info;
	uint32_t len = 0;
	struct ifreq ifr;
	int rc;

	strcpy(ifr.ifr_name, port->linux_name);

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

int pp2_port_get_statistics(struct pp2_port *port, struct pp2_ppio_statistics *stats)
{
	struct ifreq ifr;
	struct ethtool_stats *estats;
	u32 i;
	int rc;
	enum musdk_lnx_id lnx_id = lnx_id_get();

	if (!port->stats_name)
		return -1;

	estats = calloc(1, port->stats_name->len * sizeof(uint64_t) +
			sizeof(struct ethtool_stats));
	if (!estats) {
		pr_err("PORT: alloc failed\n");
		return -1;
	}

	strcpy(ifr.ifr_name, port->linux_name);

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

		if (lnx_is_mainline(lnx_id)) {
			if (!strcmp(cnt, "good_octets_received"))
				stats->rx_bytes = val;
			else if (!strcmp(cnt, "unicast_frames_received")) {
				stats->rx_unicast_packets = val;
				stats->rx_packets += val;
			} else if (!strcmp(cnt, "broadcast_frames_received"))
				stats->rx_packets += val;
			else if (!strcmp(cnt, "multicast_frames_received"))
				stats->rx_packets += val;
			else if (!strcmp(cnt, "rx_fifo_overrun"))
				stats->rx_errors += val;
			else if (!strcmp(cnt, "undersize_received"))
				stats->rx_errors += val;
			else if (!strcmp(cnt, "fragments_err_received"))
				stats->rx_errors += val;
			else if (!strcmp(cnt, "oversize_received"))
				stats->rx_errors += val;
			else if (!strcmp(cnt, "jabber_received"))
				stats->rx_errors += val;
			else if (!strcmp(cnt, "mac_receive_error"))
				stats->rx_errors += val;
			else if (!strcmp(cnt, "bad_crc_event"))
				stats->rx_errors += val;
			else if (!strcmp(cnt, "rx_ppv2_overrun"))
				stats->rx_fifo_dropped = val;
			else if (!strcmp(cnt, "rx_cls_drop"))
				stats->rx_cls_dropped = val;
			else if (!strcmp(cnt, "good_octets_sent"))
				stats->tx_bytes = val;
			else if (!strcmp(cnt, "unicast_frames_sent")) {
				stats->tx_unicast_packets = val;
				stats->tx_packets += val;
			} else if (!strcmp(cnt, "multicast_frames_sent"))
				stats->tx_packets += val;
			else if (!strcmp(cnt, "broadcast_frames_sent"))
				stats->tx_packets += val;
			else if (!strcmp(cnt, "collision"))
				stats->tx_errors += val;
			else if (!strcmp(cnt, "late_collision"))
				stats->tx_errors += val;
			else if (!strcmp(cnt, "crc_errors_sent"))
				stats->tx_errors += val;
		} else {
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
	}

	free(estats);

	return 0;
}
int pp2_port_set_priv_flags(struct pp2_port *port, u32 val)
{
	struct ifreq ifr;
	struct ethtool_value param;
	int rc;

	strcpy(ifr.ifr_name, port->linux_name);

	param.cmd =  ETHTOOL_SPFLAGS;
	param.data = val;
	ifr.ifr_data = &param;
	rc = mv_netdev_ioctl(SIOCETHTOOL, &ifr);
	if (rc) {
		pr_err("PORT: unable to set priv_flags\n");
		return rc;
	}

	return 0;
}
int pp2_port_open_uio(struct pp2_port *port)
{
	char *tmp_name;
	char dev_name[16];
	struct uio_info_t *uio_info;
	int fd;
	int max_uio_port_str_size = sizeof(UIO_PORT_STRING) + 8;

	tmp_name = kmalloc(max_uio_port_str_size, GFP_KERNEL);
	snprintf(tmp_name, max_uio_port_str_size, UIO_PORT_STRING, port->parent->id, port->id);
	uio_info = uio_find_devices_byname(tmp_name);
	if (!uio_info) {
		pr_err("UIO device (%s) not found!\n", tmp_name);
		kfree(tmp_name);
		return -ENODEV;
	}
	kfree(tmp_name);
	snprintf(dev_name, sizeof(dev_name), "/dev/uio%d", uio_info->uio_num);
	fd = open(dev_name, O_RDWR);
	if (fd < 0) {
		pr_err("Could not open file (%s)\n", dev_name);
		return errno;
	}
	port->uio_port.fd = fd;
	return 0;
}

int pp2_port_close_uio(struct pp2_port *port)
{
	int err;

	err = close(port->uio_port.fd);
	if (err < 0)
		pr_err(" Could not close file (%s)\n", strerror(errno));
	port->uio_port.fd = -1;
	return err;
}

/* Set Port enabled */
int pp2_port_set_enable(struct pp2_port *port, uint32_t en)
{
	int rc;
	struct ifreq s;

	strcpy(s.ifr_name, port->linux_name);
	pr_debug("pp2_port_set_enable : port->id %d, port->linux_name: %s, enable(%d)\n", port->id,
		 port->linux_name, en);
	rc = mv_netdev_ioctl(SIOCGIFFLAGS, &s);
	if (rc) {
		pr_err("PORT: unable to read port enabled\n");
		return rc;
	}

	if (en)
		s.ifr_flags |= IFF_UP;
	else
		s.ifr_flags &= ~IFF_UP;

	rc = mv_netdev_ioctl(SIOCSIFFLAGS, &s);
	if (rc) {
		pr_err("PORT: unable to set port enabled\n");
		return rc;
	}
	return 0;
}

/* Check if Port enabled */
int pp2_port_get_enable(struct pp2_port *port, uint32_t *en)
{
	int rc;
	struct ifreq s;

	*en = 0;
	strcpy(s.ifr_name, port->linux_name);
	rc = mv_netdev_ioctl(SIOCGIFFLAGS, &s);
	if (rc) {
		pr_err("PORT: unable to read port enabled\n");
		return rc;
	}

	if (s.ifr_flags & IFF_UP)
		*en = 1;
	return 0;
}
