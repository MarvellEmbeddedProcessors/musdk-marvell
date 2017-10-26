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

#include "std_internal.h"
#include "lib/net.h"
#include "drivers/mv_neta_ppio.h"
#include "neta_ppio.h"

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
		pr_err("PORT: Ethernet address is not multicast. Request ignored\n");
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
		pr_err("PORT: Ethernet address is not multicast. Request ignored\n");
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

	return 0;
}
