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
#include "pp2_gop_dbg.h"
#include "cls/pp2_hw_cls.h"

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

	if (mv_check_eaddr_mc(addr)) {
		struct ifreq s;
		int i;

		strcpy(s.ifr_name, port->linux_name);
		s.ifr_hwaddr.sa_family = AF_UNSPEC;
		for (i = 0; i < ETH_ALEN; i++)
			s.ifr_hwaddr.sa_data[i] = addr[i];

		rc = mv_netdev_ioctl(SIOCADDMULTI, &s);
		if (rc) {
			pr_err("PORT: unable to add mac sddress\n");
			return rc;
		}
		pr_info("PORT: Ethernet address %x:%x:%x:%x:%x:%x added to mc list\n",
			 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
	} else if (mv_check_eaddr_uc(addr)) {
		int fd;
		char buf[PP2_MAX_BUF_STR_LEN];
		char da[PP2_MAX_BUF_STR_LEN];

		strcpy(buf, port->linux_name);
		sprintf(da, " %x:%x:%x:%x:%x:%x", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		strcat(buf, da);
		fd = open("/sys/devices/platform/pp2/debug/uc_filter_add", O_WRONLY);
		if (fd == -1) {
			pr_err("PORT: unable to open sysfs\n");
			return -EFAULT;
		}
		rc = write(fd, &buf, strlen(buf) + 1);
		if (rc < 0) {
			pr_err("PORT: unable to write to sysfs\n");
			return -EFAULT;
		}
		pr_info("PORT: Ethernet address %x:%x:%x:%x:%x:%x added to uc list\n",
			 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		close(fd);
	} else {
		pr_err("PORT: Ethernet address is not unicast/multicast. Request ignored\n");
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
		pr_info("PORT: Ethernet address %x:%x:%x:%x:%x:%x removed from mc list\n",
			 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
	} else if (mv_check_eaddr_uc(addr)) {
		int fd;
		char buf[PP2_MAX_BUF_STR_LEN];
		char da[PP2_MAX_BUF_STR_LEN];

		strcpy(buf, port->linux_name);
		sprintf(da, " %x:%x:%x:%x:%x:%x", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		strcat(buf, da);
		fd = open("/sys/devices/platform/pp2/debug/uc_filter_del", O_WRONLY);
		if (fd == -1) {
			pr_err("PORT: unable to open sysfs\n");
			return -EFAULT;
		}
		rc = write(fd, &buf, strlen(buf) + 1);
		if (rc < 0) {
			pr_err("PORT: unable to write to sysfs\n");
			return -EFAULT;
		}
		pr_info("PORT: Ethernet address %x:%x:%x:%x:%x:%x removed from uc list\n",
			 addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
		close(fd);
	} else {
		pr_err("PORT: Ethernet address is not unicast/multicast. Request ignored\n");
	}
	return 0;
}

int pp2_port_flush_mac_addrs(struct pp2_port *port, uint32_t uc, uint32_t mc)
{
	int rc;

	if (mc) {
		char buf[PP2_MAX_BUF_STR_LEN];
		char name[IFNAMSIZ];
		char addr_str[PP2_MAX_BUF_STR_LEN];
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
			pr_err("PORT: unable to open sysfs\n");
			return -EFAULT;
		}
		rc = write(fd, &buf, strlen(buf) + 1);
		if (rc < 0) {
			pr_err("PORT: unable to write to sysfs\n");
			return -EFAULT;
		}
		close(fd);
	}
	return 0;
}

/* Add vlan */
int pp2_port_add_vlan(struct pp2_port *port, u16 vlan)
{
	char buf[PP2_MAX_BUF_STR_LEN];

	if ((vlan < 1) || (vlan >= 4095)) {
		pr_err("invalid vid. Range: 1:4095\n");
		return -EINVAL;
	}

	/* build manually the system command */
	/* [TODO] check other alternatives for setting vlan id */
	sprintf(buf, "ip link add link %s name %s.%d type vlan id %d", port->linux_name, port->linux_name, vlan, vlan);
	system(buf);
	return 0;
}

/* Remove vlan */
int pp2_port_remove_vlan(struct pp2_port *port, u16 vlan)
{
	char buf[PP2_MAX_BUF_STR_LEN];

	if ((vlan < 1) || (vlan >= 4095)) {
		pr_err("invalid vid. Range: 1:4095\n");
		return -EINVAL;
	}

	/* build manually the system command */
	/* [TODO] check other alternatives for setting vlan id */
	sprintf(buf, "ip link delete %s.%d", port->linux_name, vlan);
	system(buf);
	return 0;
}

int pp2_port_initialize_statistics(struct pp2_port *port)
{
	struct {
		struct ethtool_sset_info hdr;
		uint32_t buf[1];
	} sset_info;
	uint32_t len;
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
	len = sset_info.hdr.sset_mask ? sset_info.hdr.data[0] : 0;

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

	free(estats);

	return 0;
}

