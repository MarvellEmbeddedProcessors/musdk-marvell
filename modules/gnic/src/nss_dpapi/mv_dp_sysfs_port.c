
/*******************************************************************************
*  Copyright (c) 2018 Marvell.
*
*  This program is free software: you can redistribute it and/or
*  modify it under the terms of the GNU General Public License as
*  published by the Free Software Foundation, either version 2 of the
*  License, or any later version.
*
*  This program is distributed in the hope that it will be useful, but
*  WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*  General Public License for more details.
*
*******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/capability.h>
#include <linux/inet.h>

#include "mv_dp_defs.h"
#include "mv_dp_sysfs.h"
#include "mv_nss_dp.h"
#include "mv_dp_main.h"


#define MV_DP_SYSFS_PORTS_MAX		(256)
#define MV_DP_SYSFS_PORTS_NUM_OK(n)	((n) >= 0 && (n) < allocated)


static  mv_nss_dp_port_t	*ports;
static  int			allocated; /*number of allocated entries*/


/* Port update FW commands */
static void mv_dp_port_commit(int i);
static void mv_dp_port_read(int i, int id);
static void mv_dp_port_dflt_dest_set(int i);

/* Port update Cache commands */
static void mv_dp_port_alloc(int n);
static void mv_dp_port_set_type(int i, int type);
static void mv_dp_port_release(void);
static void mv_dp_port_clear(int i);
static void mv_dp_port_show(int i);
static void mv_dp_port_link_state_show_single(mv_nss_dp_eth_link_state_t *state);

static void mv_dp_port_set_id(int i, int id);
static void mv_dp_port_set_type_idx(int i, int idx);
static void mv_dp_port_set_state(int i, int state);

static void mv_dp_port_set_eth_options(int i, int options);
static void mv_dp_port_set_eth_mtu(int i, int mtu);
static void mv_dp_port_set_eth_tpid(int i, int tpid);
static void mv_dp_port_set_eth_native_vlan(int i, int native_vlan);
static void mv_dp_port_set_eth_policy(int i, int policy);

static void mv_dp_port_set_lag_links(int i, int lag_links);
static void mv_dp_port_set_lag_hash_prof(int i, int lag_hash_prof);
static void mv_dp_port_set_cw_remote_mac(int i, unsigned char *mac);
static void mv_dp_port_set_cw_local_mac(int i, unsigned char *mac);
static void mv_dp_port_set_cw_bssid(int i, unsigned char *mac);
static void mv_dp_port_set_cw_remote_ip(int i, int af, unsigned char *ip);
static void mv_dp_port_set_cw_local_ip(int i, int af, unsigned char *ip);
static void mv_dp_port_set_cw_port(int i, int port_id);
static void mv_dp_port_set_cw_proto(int i, int proto);
static void mv_dp_port_set_cw_csum(int i, int val);
static void mv_dp_port_set_cw_ttl(int i, int ttl);
static void mv_dp_port_set_cw_flow_lbl(int i, int flow_lbl);
static void mv_dp_port_set_cw_dtls_index(int i, int dtls_index);
static void mv_dp_port_set_cw_remote_port(int i, int port);
static void mv_dp_port_set_cw_local_port(int i, int port);
static void mv_dp_port_set_cw_pmtu(int i, int pmtu);
static void mv_dp_port_set_cw_vlan_id(int i, int vlan_id);
static void mv_dp_port_set_cw_options(int i, int options);
static void mv_dp_port_set_cw_uc_qos(int i, int uc_qos);
static void mv_dp_port_set_cw_mc_qos(int i, int mc_qos);
/*static void mv_dp_port_set_cw_l4_prt_range(int i, int l4_prt_range);*/
static void mv_dp_port_set_cpu_core_mask(int i, int core_mask);
static void mv_dp_port_set_cpu_hash_prof(int i, int cpu_hash_prof);
static void mv_dp_port_set_vxlan_remote_mac(int i, unsigned char *mac);
static void mv_dp_port_set_vxlan_local_mac(int i, unsigned char *mac);
static void mv_dp_port_set_vxlan_remote_ip(int i, int af, unsigned char *ip);
static void mv_dp_port_set_vxlan_local_ip(int i, int af, unsigned char *ip);
static void mv_dp_port_set_vxlan_port(int i, int port_id);
static void mv_dp_port_set_vxlan_proto(int i, int proto);
static void mv_dp_port_set_vxlan_csum(int i, int val);
static void mv_dp_port_set_vxlan_ttl(int i, int ttl);
static void mv_dp_port_set_vxlan_flow_lbl(int i, int flow_lbl);
static void mv_dp_port_set_vxlan_remote_port(int i, int port);
static void mv_dp_port_set_vxlan_local_port(int i, int port);
static void mv_dp_port_set_vxlan_vlan_id(int i, int vlan_id);
static void mv_dp_port_set_vxlan_options(int i, int options);
static void mv_dp_port_set_vxlan_uc_qos(int i, int uc_qos);
static void mv_dp_port_set_vxlan_mc_qos(int i, int mc_qos);

/* Port get FW commands */
static void mv_dp_port_get(int id);
static void mv_dp_port_link_state_get(int id);
static void mv_dp_port_stats_get(int id);
static void mv_dp_port_stats_bulk_get(int i, int cnt);
static void mv_dp_port_stats_reset(int id);

/* Port delete FW commands */
static void mv_dp_port_delete(int id);

static void mv_dp_vxlan_vni_cfg_set(int id, int ind, int vni);
static void mv_dp_vxlan_vni_cfg_get(int id, int ind);
static void mv_dp_vxlan_vni_cfg_delete(int id, int ind);

/* Port callbacks */
static void mv_dp_port_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_port_dflt_dest_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_port_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_port_delete_cb(mv_nss_dp_event_t *event);
static void mv_dp_port_read_cb(mv_nss_dp_event_t *event);
static void mv_dp_port_stats_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_port_stats_bulk_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_port_stats_reset_cb(mv_nss_dp_event_t *event);
static void mv_dp_port_link_state_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_vxlan_vni_cfg_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_vxlan_vni_cfg_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_vxlan_vni_cfg_del_cb(mv_nss_dp_event_t *event);


static ssize_t mv_dp_port_sysfs_help(char *b)
{
	int o = 0;

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  help                   - Show Help\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat help_port_vxlan - Show Help for VXLAN port type\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  port_release           - Release Ports Cache\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]           > port_clear             - Clear Ports Cache entry [i] or -1 for ALL to 0\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [n]           > port_alloc             - Allocate Port cache of [n] entries\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [ID]      > port_read              - Read Port Record with [ID] from FW to cache index [i]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]          > port_get               - Print only Port Record with [ID] from FW (not saved)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]           > port_commit            - Save Port stored at index [i] to FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]           > port_show              - Show Port cache entry [i] or -1 for ALL\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]          > port_delete            - Delete Port Record with [ID] from FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]          > port_stats_get         - Print Statistics for Port Record with [ID] from FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]          > port_stats_reset       - Reset Statistics for Port Record with [ID] in FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [T]       > port_type_set          - Set Port [i]'s port type [T]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [ID]      > port_id_set            - Set Port [i]'s port id [ID]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [IDX]     > port_type_idx_set      - Set Port [i]'s port type index [IDX]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [ST]      > port_state_set         - Set Port [i]'s port state [ST]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [MAC]     > port_cw_remote_mac_set - Set Port [i]'s capwap remote [MAC]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [MAC]     > port_cw_local_mac_set  - Set Port [i]'s capwap local [MAC]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [MAC]     > port_cw_bssid_set      - Set Port [i]'s capwap bssid [MAC]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [AF] [IP] > port_cw_remote_ip_set  - Set Port [i]'s capwap remote [IP]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [AF] [IP] > port_cw_local_ip_set   - Set Port [i]'s capwap local [IP]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [ID]      > port_cw_port_set       - Set Port [i]'s tunnel CPU or ETH port [ID]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [L4]      > port_cw_proto_set      - Set Port [i]'s capwap transport protocol [L4]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [L4]      > port_cw_csum_set       - Set Port [i]'s L4 protocol checksum: 1 or 0\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [TT]      > port_cw_ttl_set        - Set Port [i]'s capwap TTL [TT]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [FL]      > port_cw_flow_lbl_set   - Set Port [i]'s capwap Flow Label [FL]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [DI]      > port_cw_dtls_index_set - Set Port [i]'s capwap DTLS index [DI]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [P4]      > port_cw_remote_port_set - Set Port [i]'s tunnel L4 remote port [L4]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [P4]      > port_cw_local_port_set - Set Port [i]'s tunnel L4 local port [L4]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PMTU]    > port_cw_pmtu_set       - Set Port [i]'s tunnel [PMTU]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [VLID]    > port_cw_vlan_id_set    - Set Port [i]'s tunnel VLAN ID [V]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [O]       > port_cw_options_set    - Set Port [i]'s tunnel options [O] (hex)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PH]      > port_cw_uc_qos_set     - Set Port [i]'s tunnel UC QOS policy [PH]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PH]      > port_cw_mc_qos_set     - Set Port [i]'s tunnel MC QOS policy [PH]\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		     "\n     [MAC] = MAC address as 00:00:00:00:00:00\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "     [AF]  = Inet address family: 4 for IPv4 and 6 for IPv6\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "     [IP]  = IP address as 0.0.0.0 or 0:0:0:0:0:0:0:0\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "     [ID]  = Virtual port ID: %d-%d\n",
			   (MV_NSS_DP_PORT_ID_NONE + 1), (MV_NSS_DP_PORT_ALL - 1));
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "     [T]   = Virtual port type: 0 - None, 4-CAPWAP, 5-VXLAN\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "     [ST]  = Virtual port state: 0-DOWN, 1-UP\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "     [L4]  = Transport protocol: 0-UDP, 1-UDP Lite\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "     [DI]  = DTLS index 0..0xFFFE or 0xFFFF for no encryption\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "     [O]   = CAPWAP Tunnel options bitmask\n");

	return o;
}

static ssize_t mv_dp_port_sysfs_help_vxlan(char *b)
{
	int o = 0;

	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [MAC]     > port_vxlan_remote_mac_set - Set Port [i]'s VXLAN remote [MAC]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [MAC]     > port_vxlan_local_mac_set  - Set Port [i]'s VXLAN local [MAC]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [AF] [IP] > port_vxlan_remote_ip_set  - Set Port [i]'s VXLAN remote [IP]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [AF] [IP] > port_vxlan_local_ip_set   - Set Port [i]'s VXLAN local [IP]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [ID]      > port_vxlan_port_set       - Set Port [i]'s tunnel virtual port [ID]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [L4]      > port_vxlan_proto_set      - Set Port [i]'s VXLAN transport protocol [L4]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [L4]      > port_vxlan_csum_set       - Set Port [i]'s L4 protocol checksum: 1 or 0\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [TT]      > port_vxlan_ttl_set        - Set Port [i]'s VXLAN TTL [TT]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [FL]      > port_vxlan_flow_lbl_set   - Set Port [i]'s VXLAN Flow Label [FL]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [P4]      > port_vxlan_remote_port_set - Set Port [i]'s tunnel L4 remote port [L4]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [P4]      > port_vxlan_local_port_set - Set Port [i]'s tunnel L4 local port [L4]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [VLID]    > port_vxlan_vlan_id_set    - Set Port [i]'s tunnel VLAN ID [V]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [O]       > port_vxlan_options_set    - Set Port [i]'s tunnel options [O] (hex)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [PH]      > port_vxlan_uc_qos_set     - Set Port [i]'s tunnel UC QOS policy [PH]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [i] [PH]      > port_vxlan_mc_qos_set     - Set Port [i]'s tunnel MC QOS policy [PH]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [ID] [IND] [VNI]    > vxlan_vni_cfg_set    - Set Port[ID] VXLAN mapping of index[IND] to VNI\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [ID] [IND]          > vxlan_vni_cfg_get    - Get Port[ID] VXLAN mapping of index[IND] to VNI\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"echo [ID] [IND]          > vxlan_vni_cfg_delete - Delete Port[ID] VXLAN mapping of index[IND] to VNI\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		     "\n[MAC] = MAC address as 00:00:00:00:00:00\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "[AF] = Inet address family: 4 for IPv4 and 6 for IPv6\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "[IP] = IP address as 0.0.0.0 or 0:0:0:0:0:0:0:0\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
			"[ID] = Virtual port ID: %d-%d\n",
					(MV_NSS_DP_PORT_ID_NONE + 1), (MV_NSS_DP_PORT_ALL - 1));
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "[L4] = Transport protocol: 0-UDP, 1-UDP Lite\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "[O] = VXLAN Tunnel options bitmask\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "[IND] = VXLAN Tunnel VNI index 0-31\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "[VNI] = VXLAN Tunnel VNI value 0x0-0xFFFFFF\n");
	return o;
}

static ssize_t mv_dp_port_sysfs_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int off = 0;
	const char  *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "help")) {
		return mv_dp_port_sysfs_help(buf);
	} else if (!strcmp(name, "help_port_vxlan")) {
		return mv_dp_port_sysfs_help_vxlan(buf);
	} else if (!strcmp(name, "port_release")) {
			mv_dp_port_release();
	} else {
		MV_DP_CLI_FAIL("Parse Error CMD: <%s>\n", MV_NSS_DP_INVALID_PARAM, name);
		off = mv_dp_port_sysfs_help(buf);
	}
	return off;
}

static ssize_t mv_dp_port_sysfs_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t len)
{
	const char *name = attr->attr.name;
	unsigned int    a, b, c;
	unsigned char mac[6];
	unsigned char ip[INET6_ADDRSTRLEN];


	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = b = c = 0;

	if (!strcmp(name, "port_read")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_read(a, b);
	} else if (!strcmp(name, "port_clear")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_port_clear(a);
	} else if (!strcmp(name, "port_get")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_port_get(a);
	} else if (!strcmp(name, "port_stats_get")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_port_stats_get(a);
	} else if (!strcmp(name, "port_link_state_get")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_port_link_state_get(a);
	} else if (!strcmp(name, "port_stats_reset")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_port_stats_reset(a);
	} else if (!strcmp(name, "port_dflt_dest_set")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_port_dflt_dest_set(a);
	} else if (!strcmp(name, "port_stats_bulk_get")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_stats_bulk_get(a, b);
	} else if (!strcmp(name, "port_commit")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_port_commit(a);
	} else if (!strcmp(name, "port_show")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_port_show(a);
	} else if (!strcmp(name, "port_alloc")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_port_alloc(a);
	} else if (!strcmp(name, "port_delete")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_port_delete(a);
	} else if (!strcmp(name, "port_type_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_type(a, b);
	} else if (!strcmp(name, "port_id_set")) {
		if (2 != sscanf(buf, "%d %i", &a, &b))
			goto err;
		mv_dp_port_set_id(a, b);
	} else if (!strcmp(name, "port_type_idx_set")) {
		if (sscanf(buf, "%d %i", &a, &b) != 2)
			goto err;
		mv_dp_port_set_type_idx(a, b);
	} else if (!strcmp(name, "port_state_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_state(a, b);
	} else if (!strcmp(name, "port_eth_options_set")) {
		if (2 != sscanf(buf, "%d 0x%X", &a, &b))
			goto err;
		mv_dp_port_set_eth_options(a, b);
	} else if (!strcmp(name, "port_eth_mtu_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_eth_mtu(a, b);
	} else if (!strcmp(name, "port_eth_tpid_set")) {
		if (2 != sscanf(buf, "%d 0x%X", &a, &b))
			goto err;
		mv_dp_port_set_eth_tpid(a, b);
	} else if (!strcmp(name, "port_eth_native_vlan_set")) {
		if (2 != sscanf(buf, "%d %i", &a, &b))
			goto err;
		mv_dp_port_set_eth_native_vlan(a, b);
	} else if (!strcmp(name, "port_eth_policy_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_eth_policy(a, b);
	} else if (!strcmp(name, "port_lag_links_set")) {
		if (2 != sscanf(buf, "%d 0x%X", &a, &b))
			goto err;
		mv_dp_port_set_lag_links(a, b);
	} else if (!strcmp(name, "port_lag_hash_prof_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_lag_hash_prof(a, b);
	} else if (!strcmp(name, "port_cw_remote_mac_set")) {
		if (7 != sscanf(buf, "%d %hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		       &a, &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
			goto err;
		mv_dp_port_set_cw_remote_mac(a, mac);
	} else if (!strcmp(name, "port_cw_local_mac_set")) {
		if (7 != sscanf(buf, "%d %hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		       &a, &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
			goto err;
		mv_dp_port_set_cw_local_mac(a, mac);
	} else if (!strcmp(name, "port_cw_bssid_set")) {
		if (7 != sscanf(buf, "%d %hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		       &a, &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
			goto err;
		mv_dp_port_set_cw_bssid(a, mac);
	} else if (!strcmp(name, "port_cw_remote_ip_set")) {
		if (3 != sscanf(buf, "%d %d %s", &a, &b, ip))
			goto err;
		mv_dp_port_set_cw_remote_ip(a, b, ip);
	} else if (!strcmp(name, "port_cw_local_ip_set")) {
		if (3 != sscanf(buf, "%d %d %s", &a, &b, ip))
			goto err;
		mv_dp_port_set_cw_local_ip(a, b, ip);
	} else if (!strcmp(name, "port_cw_port_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_cw_port(a, b);
	} else if (!strcmp(name, "port_cw_proto_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_cw_proto(a, b);
	} else if (!strcmp(name, "port_cw_csum_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_cw_csum(a, b);
	} else if (!strcmp(name, "port_cw_ttl_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_cw_ttl(a, b);
	} else if (!strcmp(name, "port_cw_flow_lbl_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_cw_flow_lbl(a, b);
	} else if (!strcmp(name, "port_cw_dtls_index_set")) {
		if (2 != sscanf(buf, "%d 0x%X", &a, &b))
			goto err;
		mv_dp_port_set_cw_dtls_index(a, b);
	} else if (!strcmp(name, "port_cw_remote_port_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_cw_remote_port(a, b);
	} else if (!strcmp(name, "port_cw_local_port_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_cw_local_port(a, b);
	} else if (!strcmp(name, "port_cw_pmtu_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_cw_pmtu(a, b);
	} else if (!strcmp(name, "port_cw_vlan_id_set")) {
		if (2 != sscanf(buf, "%d %i", &a, &b))
			goto err;
		mv_dp_port_set_cw_vlan_id(a, b);
	} else if (!strcmp(name, "port_cw_options_set")) {
		if (2 != sscanf(buf, "%d 0x%X", &a, &b))
			goto err;
		mv_dp_port_set_cw_options(a, b);
	} else if (!strcmp(name, "port_cw_uc_qos_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_cw_uc_qos(a, b);
	} else if (!strcmp(name, "port_cw_mc_qos_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_port_set_cw_mc_qos(a, b);
	} else if (!strcmp(name, "port_cpu_core_mask_set")) {
		if (2 != sscanf(buf, "%d 0x%X", &a, &b))
			goto err;
		mv_dp_port_set_cpu_core_mask(a, b);
	} else if (!strcmp(name, "port_cpu_hash_prof_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_port_set_cpu_hash_prof(a, b);
	} else if (!strcmp(name, "port_vxlan_remote_mac_set")) {
		if (sscanf(buf, "%d %hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
			   &a, &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5])  != 7)
			goto err;
		mv_dp_port_set_vxlan_remote_mac(a, mac);
	} else if (!strcmp(name, "port_vxlan_local_mac_set")) {
		if (sscanf(buf, "%d %hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
			   &a, &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) != 7)
			goto err;
		mv_dp_port_set_vxlan_local_mac(a, mac);
	} else if (!strcmp(name, "port_vxlan_remote_ip_set")) {
		if (sscanf(buf, "%d %d %s", &a, &b, ip) != 3)
			goto err;
		mv_dp_port_set_vxlan_remote_ip(a, b, ip);
	} else if (!strcmp(name, "port_vxlan_local_ip_set")) {
		if (sscanf(buf, "%d %d %s", &a, &b, ip) != 3)
			goto err;
		mv_dp_port_set_vxlan_local_ip(a, b, ip);
	} else if (!strcmp(name, "port_vxlan_port_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_port_set_vxlan_port(a, b);
	} else if (!strcmp(name, "port_vxlan_proto_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_port_set_vxlan_proto(a, b);
	} else if (!strcmp(name, "port_vxlan_csum_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_port_set_vxlan_csum(a, b);
	} else if (!strcmp(name, "port_vxlan_ttl_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_port_set_vxlan_ttl(a, b);
	} else if (!strcmp(name, "port_vxlan_flow_lbl_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_port_set_vxlan_flow_lbl(a, b);
	} else if (!strcmp(name, "port_vxlan_remote_port_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_port_set_vxlan_remote_port(a, b);
	} else if (!strcmp(name, "port_vxlan_local_port_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_port_set_vxlan_local_port(a, b);
	} else if (!strcmp(name, "port_vxlan_vlan_id_set")) {
		if (sscanf(buf, "%d %i", &a, &b) != 2)
			goto err;
		mv_dp_port_set_vxlan_vlan_id(a, b);
	} else if (!strcmp(name, "port_vxlan_options_set")) {
		if (sscanf(buf, "%d 0x%X", &a, &b) != 2)
			goto err;
		mv_dp_port_set_vxlan_options(a, b);
	} else if (!strcmp(name, "port_vxlan_uc_qos_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_port_set_vxlan_uc_qos(a, b);
	} else if (!strcmp(name, "port_vxlan_mc_qos_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_port_set_vxlan_mc_qos(a, b);
	} else if (!strcmp(name, "vxlan_vni_cfg_set")) {
		if (sscanf(buf, "%d %d 0x%X", &a, &b, &c) != 3)
			goto err;
		mv_dp_vxlan_vni_cfg_set(a, b, c);
	} else if (!strcmp(name, "vxlan_vni_cfg_get")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_vxlan_vni_cfg_get(a, b);
	} else if (!strcmp(name, "vxlan_vni_cfg_delete")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_vxlan_vni_cfg_delete(a, b);
	} else {
		goto err;
	}

	return len;
err:
	MV_DP_LOG_INF("Parse Error CMD:<%s>\n", attr->attr.name);
	return -EINVAL;
}

static DEVICE_ATTR(help,				S_IRUSR, mv_dp_port_sysfs_show, NULL);
static DEVICE_ATTR(help_port_vxlan,			S_IRUSR, mv_dp_port_sysfs_show, NULL);
static DEVICE_ATTR(port_release,			S_IRUSR, mv_dp_port_sysfs_show, NULL);
static DEVICE_ATTR(port_clear,				S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_read,				S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_get,				S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_stats_get,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_stats_reset,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_commit,				S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_show,				S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_alloc,				S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_delete,				S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_type_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_id_set,				S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_type_idx_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_state_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_remote_mac_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_local_mac_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_bssid_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_remote_ip_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_local_ip_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_port_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_proto_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_ttl_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_flow_lbl_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_dtls_index_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_remote_port_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_local_port_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_pmtu_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_vlan_id_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_options_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_uc_qos_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_mc_qos_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_cw_csum_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_remote_mac_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_local_mac_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_remote_ip_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_local_ip_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_port_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_proto_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_ttl_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_flow_lbl_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_remote_port_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_local_port_set,		S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_vlan_id_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_options_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_uc_qos_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_mc_qos_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(port_vxlan_csum_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(vxlan_vni_cfg_set,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(vxlan_vni_cfg_get,			S_IWUSR, NULL, mv_dp_port_sysfs_store);
static DEVICE_ATTR(vxlan_vni_cfg_delete,			S_IWUSR, NULL, mv_dp_port_sysfs_store);


static struct attribute *mv_dp_port_sysfs_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_help_port_vxlan.attr,
	&dev_attr_port_release.attr,
	&dev_attr_port_clear.attr,
	&dev_attr_port_read.attr,
	&dev_attr_port_get.attr,
	&dev_attr_port_stats_get.attr,
	&dev_attr_port_stats_reset.attr,
	&dev_attr_port_commit.attr,
	&dev_attr_port_show.attr,
	&dev_attr_port_alloc.attr,
	&dev_attr_port_delete.attr,
	&dev_attr_port_type_set.attr,
	&dev_attr_port_id_set.attr,
	&dev_attr_port_type_idx_set.attr,
	&dev_attr_port_state_set.attr,
	&dev_attr_port_cw_remote_mac_set.attr,
	&dev_attr_port_cw_local_mac_set.attr,
	&dev_attr_port_cw_bssid_set.attr,
	&dev_attr_port_cw_remote_ip_set.attr,
	&dev_attr_port_cw_local_ip_set.attr,
	&dev_attr_port_cw_port_set.attr,
	&dev_attr_port_cw_csum_set.attr,
	&dev_attr_port_cw_proto_set.attr,
	&dev_attr_port_cw_ttl_set.attr,
	&dev_attr_port_cw_flow_lbl_set.attr,
	&dev_attr_port_cw_dtls_index_set.attr,
	&dev_attr_port_cw_remote_port_set.attr,
	&dev_attr_port_cw_local_port_set.attr,
	&dev_attr_port_cw_pmtu_set.attr,
	&dev_attr_port_cw_vlan_id_set.attr,
	&dev_attr_port_cw_options_set.attr,
	&dev_attr_port_cw_uc_qos_set.attr,
	&dev_attr_port_cw_mc_qos_set.attr,
	&dev_attr_port_vxlan_remote_mac_set.attr,
	&dev_attr_port_vxlan_local_mac_set.attr,
	&dev_attr_port_vxlan_remote_ip_set.attr,
	&dev_attr_port_vxlan_local_ip_set.attr,
	&dev_attr_port_vxlan_port_set.attr,
	&dev_attr_port_vxlan_csum_set.attr,
	&dev_attr_port_vxlan_proto_set.attr,
	&dev_attr_port_vxlan_ttl_set.attr,
	&dev_attr_port_vxlan_flow_lbl_set.attr,
	&dev_attr_port_vxlan_remote_port_set.attr,
	&dev_attr_port_vxlan_local_port_set.attr,
	&dev_attr_port_vxlan_vlan_id_set.attr,
	&dev_attr_port_vxlan_options_set.attr,
	&dev_attr_port_vxlan_uc_qos_set.attr,
	&dev_attr_port_vxlan_mc_qos_set.attr,
	&dev_attr_vxlan_vni_cfg_set.attr,
	&dev_attr_vxlan_vni_cfg_get.attr,
	&dev_attr_vxlan_vni_cfg_delete.attr,

	NULL
};


static struct attribute_group mv_dp_port_sysfs_group = {
	.name = "port",
	.attrs = mv_dp_port_sysfs_attrs,
};



int mv_dp_port_sysfs_init(struct kobject *ko)
{
	int err;
	err = sysfs_create_group(ko, &mv_dp_port_sysfs_group);

	if (err) {
		MV_DP_LOG_INF("Port sysFS group init failed %d\n", err);
		return err;
	}
	allocated = 0;
	MV_DP_LOG_DBG1("Port sysFS INITALIZED\n");
	return err;
}

int mv_dp_port_sysfs_exit(struct kobject *ko)
{
	sysfs_remove_group(ko, &mv_dp_port_sysfs_group);
	if (allocated) {
		kfree(ports);
		allocated = 0;
	}
	return 0;
}


static void mv_dp_port_show_single(mv_nss_dp_port_t *port)
{

	unsigned char	remote_ip[INET6_ADDRSTRLEN];
	unsigned char	local_ip[INET6_ADDRSTRLEN];

	if (!port) {
		MV_DP_CLI_FAIL("Null Port", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("|ID:%3d|T:%1d|S:%1d\n", port->port_id, port->type, port->state);

	switch (port->type) {

	case MV_NSS_DP_PORT_ETH:
		MV_DP_CLI_CONT("|OPT:0x%08X|MTU:%d|TPID:0x%04X|NVLAN:%d|POL:%d|<\n",
				port->params.eth.options,
				port->params.eth.mtu,
				port->params.eth.tpid,
				port->params.eth.native_vlan,
				port->params.eth.policy);
	    break;

	case MV_NSS_DP_PORT_ETH_LAG:
		MV_DP_CLI_CONT("|LINKS:0x%08X|HASHPROF:%d|<\n",
				port->params.eth_lag.links,
				port->params.eth_lag.hash_prof_id);
		break;

	case MV_NSS_DP_PORT_CPU:
		MV_DP_CLI_CONT("|CORES:0x%08X|HASHPROF:%d|<\n",
				port->params.cpu.cores,
				port->params.cpu.hash_prof_id);
	    break;

	case MV_NSS_DP_PORT_CAPWAP:
		mv_dp_sysfs_ip_to_str(remote_ip, &(port->params.capwap.remote_ip));
		mv_dp_sysfs_ip_to_str(local_ip, &(port->params.capwap.local_ip));
		MV_DP_CLI_CONT("|RMAC:%02X%02X%02X%02X%02X%02X|LMAC:%02X%02X%02X%02X%02X%02X|BSSID:%02X%02X%02X%02X%02X%02X|\n",
				port->params.capwap.remote_mac.addr[0],
				port->params.capwap.remote_mac.addr[1],
				port->params.capwap.remote_mac.addr[2],
				port->params.capwap.remote_mac.addr[3],
				port->params.capwap.remote_mac.addr[4],
				port->params.capwap.remote_mac.addr[5],
				port->params.capwap.local_mac.addr[0],
				port->params.capwap.local_mac.addr[1],
				port->params.capwap.local_mac.addr[2],
				port->params.capwap.local_mac.addr[3],
				port->params.capwap.local_mac.addr[4],
				port->params.capwap.local_mac.addr[5],
				port->params.capwap.bssid.addr[0],
				port->params.capwap.bssid.addr[1],
				port->params.capwap.bssid.addr[2],
				port->params.capwap.bssid.addr[3],
				port->params.capwap.bssid.addr[4],
				port->params.capwap.bssid.addr[5]);
		MV_DP_CLI_CONT("|RIP:%s|LIP:%s|\n", remote_ip, local_ip);
		MV_DP_CLI_CONT("|PORT:%d|PROTO:%d|TTL:%d|FLBL:%d|DTLSIDX:%d|RPORT:%d|LPORT:%d|PMTU:%d|\n",
				port->params.capwap.port_id,
				port->params.capwap.proto,
				port->params.capwap.ttl,
				port->params.capwap.flow_lbl,
				port->params.capwap.dtls_index,
				port->params.capwap.remote_port,
				port->params.capwap.local_port,
				port->params.capwap.pmtu);
		MV_DP_CLI_CONT("|VLAN_ID:%d|OPT:0x%08X|UQOSP:%d|MQOSP:%d|CSUM:%d|\n",
				port->params.capwap.vlan_id,
				port->params.capwap.options,
				port->params.capwap.uc_qos_policy,
				port->params.capwap.mc_qos_policy,
				port->params.capwap.calc_cs);


		break;

	case MV_NSS_DP_PORT_VXLAN:
			mv_dp_sysfs_ip_to_str(remote_ip, &(port->params.vxlan.remote_ip));
			mv_dp_sysfs_ip_to_str(local_ip, &(port->params.vxlan.local_ip));
			MV_DP_CLI_CONT("|RMAC:%02X%02X%02X%02X%02X%02X|LMAC:%02X%02X%02X%02X%02X%02X|\n",
					port->params.vxlan.remote_mac.addr[0],
					port->params.vxlan.remote_mac.addr[1],
					port->params.vxlan.remote_mac.addr[2],
					port->params.vxlan.remote_mac.addr[3],
					port->params.vxlan.remote_mac.addr[4],
					port->params.vxlan.remote_mac.addr[5],
					port->params.vxlan.local_mac.addr[0],
					port->params.vxlan.local_mac.addr[1],
					port->params.vxlan.local_mac.addr[2],
					port->params.vxlan.local_mac.addr[3],
					port->params.vxlan.local_mac.addr[4],
					port->params.vxlan.local_mac.addr[5]);
			MV_DP_CLI_CONT("|RIP:%s|LIP:%s|\n", remote_ip, local_ip);
			MV_DP_CLI_CONT("|PORT:%d|PROTO:%d|TTL:%d|FLBL:%d|RPORT:%d|LPORT:%d|\n",
					port->params.vxlan.port_id,
					port->params.vxlan.proto,
					port->params.vxlan.ttl,
					port->params.vxlan.flow_lbl,
					port->params.vxlan.remote_port,
					port->params.vxlan.local_port);
			MV_DP_CLI_CONT("|VLAN_ID:%d|OPT:0x%08X|UQOSP:%d|MQOSP:%d|CSUM:%d|\n",
					port->params.vxlan.vlan_id,
					port->params.vxlan.options,
					port->params.vxlan.uc_qos_policy,
					port->params.vxlan.mc_qos_policy,
					port->params.vxlan.calc_cs);
			break;
	default:
		MV_DP_CLI_CONT("|WRONG TYPE!!!<\n");
		break;

	}

}

static void mv_dp_port_show_stats_single(mv_nss_dp_port_stats_t *port_stats)
{

	if (!port_stats) {
		MV_DP_CLI_FAIL("Null Port Statistics", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("|%3d|%1d|%10llu|%10llu|%10llu|%10llu|%10llu|%10llu|<\n",
			port_stats->port_id,
			port_stats->type,
			port_stats->rx_pkts,
			port_stats->rx_errors,
			port_stats->tx_pkts,
			port_stats->tx_errors,
			port_stats->rx_octets,
			port_stats->tx_octets);

}

static void mv_dp_port_link_state_show_single(mv_nss_dp_eth_link_state_t *state)
{

	if (!state) {
		MV_DP_CLI_FAIL("Null Port State", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("|%7u|%7d|%7d|%6d|<\n",
		       state->port_id,
		       state->speed,
		       state->is_up,
		       state->duplex);
}


static void mv_dp_port_release(void)
{

	if (!allocated) {
		MV_DP_CLI_FAIL("Not Allocated\n", MV_NSS_DP_FAILED);
		return;
	}

	kfree(ports);
	MV_DP_CLI_OK("Ports Cache Released\n");
	allocated = 0;

}

static void mv_dp_port_clear(int i)
{
	if (!allocated) {
		MV_DP_CLI_FAIL("Not allocated\n", MV_NSS_DP_FAILED);
		return;
	}

	if (i == -1) {
		/*clear all*/
		memset(ports, 0, sizeof(mv_nss_dp_port_t)*allocated);
		MV_DP_CLI_OK("Ports Cache Cleared: %d\n", allocated);
	} else if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index:%d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	} else {
		memset(&ports[i], 0, sizeof(mv_nss_dp_port_t));
		MV_DP_CLI_OK("Ports Cache entry %d cleared\n", i);
	}
}

static void mv_dp_port_read(int i, int id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;

	/*port id validation is to be performed in dpapi*/

	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	/*the index is stored in xid*/
	res.cb = mv_dp_port_read_cb;
	res.xid = i;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	if (MV_NSS_DP_OK != mv_nss_dp_port_get((mv_nss_dp_port_id_t)id, &res)) {
		MV_DP_CLI_FAIL("Get Port %d to Index: %d\n", MV_NSS_DP_FAILED, id, i);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Get Port ID %d to Index:%d\n", id, i);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Port Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}

static void mv_dp_port_get(int id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_port_get_cb;
	res.xid = id;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	if (MV_NSS_DP_OK != mv_nss_dp_port_get((mv_nss_dp_port_id_t)id, &res)) {
		MV_DP_CLI_FAIL("Get Port %d\n", MV_NSS_DP_FAILED, id);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Get Port ID %d\n", id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Port Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}

static void mv_dp_port_stats_get(int id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_port_stats_get_cb;
	res.xid = id;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	if (MV_NSS_DP_OK != mv_nss_dp_port_stats_get((mv_nss_dp_port_id_t)id, &res)) {
		MV_DP_CLI_FAIL("Get Statistics for Port %d\n", MV_NSS_DP_FAILED, id);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Get Statistics for Port ID: %d\n", id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Port Statistics Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}


static void mv_dp_port_link_state_get(int id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_port_link_state_get_cb;
	res.xid = id;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	if (MV_NSS_DP_OK != mv_nss_eth_link_state_get((mv_nss_dp_port_id_t)id, &res)) {
		MV_DP_CLI_FAIL("Get link state for Port %d\n", MV_NSS_DP_FAILED, id);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Get eth port link state for Port ID: %d\n", id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get eth port link state Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}



static void mv_dp_port_stats_reset(int id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_port_stats_reset_cb;
	res.xid = id;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	if (MV_NSS_DP_OK != mv_nss_dp_port_stats_reset((mv_nss_dp_port_id_t)id, &res)) {
		MV_DP_CLI_FAIL("Reset Statistics for Port %d\n", MV_NSS_DP_FAILED, id);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Reset Statistics for Port %d\n", id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Reset Port Statistics Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}

static void mv_dp_port_dflt_dest_set(int id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_port_dflt_dest_set_cb;
	res.xid = id;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	if (MV_NSS_DP_OK != mv_nss_dp_port_dst_set((mv_nss_dp_port_id_t)id, &res)) {
		MV_DP_CLI_FAIL("Set Dflt dest Port to %d\n", MV_NSS_DP_FAILED, id);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Set Dflt dest Port to %d\n", id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Set Dflt dest Port Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}


static void mv_dp_port_stats_bulk_get(int i, int cnt)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;
	mv_nss_dp_port_stats_t *stats_ptr;


	stats_ptr = kzalloc(sizeof(mv_nss_dp_port_stats_t)*cnt, GFP_KERNEL);
	if (!stats_ptr) {
		MV_DP_CLI_FAIL("Stats buffer alloc failed for %d entries\n", MV_NSS_DP_OUT_OF_RESOURCES, cnt);
		return;
	}

	/*the index is stored in xid*/
	res.cb = mv_dp_port_stats_bulk_get_cb;
	res.xid = cnt;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			kfree(stats_ptr);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	if (MV_NSS_DP_OK != mv_nss_dp_port_bulk_stats_get(i, cnt, stats_ptr, &res)) {
		MV_DP_CLI_FAIL("Get Bulk Statistics for %d entries\n", MV_NSS_DP_FAILED, cnt);
		if (res.cookie)
			kfree(res.cookie);
		kfree(stats_ptr);
		return;
	}

	MV_DP_CLI_TMSG("Get Bulk Statistics for %d entries from index: %d\n", cnt, i);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Bulk Port Statistics Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}

static void mv_dp_port_commit(int i)
{
	mv_nss_dp_result_spec_t		res;
	mv_nss_dp_port_t		*ptr_port;
	struct completion		*compl_ptr;


	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index:%d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	} else	{
		ptr_port = &ports[i];
	}

	/*the index is stored in xid*/
	res.cb = mv_dp_port_set_cb;
	res.xid = i;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	if (MV_NSS_DP_OK != mv_nss_dp_port_set(ptr_port, &res)) {
		MV_DP_CLI_FAIL("Commit Port %d Index: %d\n", MV_NSS_DP_FAILED, ptr_port->port_id, i);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Commit Port ID %d Index: %d\n", ptr_port->port_id, i);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Commit Port Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}

static void mv_dp_port_show(int index)
{
	int i;

	if (index == -1) {
		/*show all*/
		MV_DP_CLI_OK("|PORTS CACHE|TOTAL:%d|<\n", allocated);
		for (i = 0; i < allocated; i++) {
			MV_DP_CLI_CONT("|IDX:%3d", i);
			mv_dp_port_show_single(&ports[i]);
		}

		MV_DP_CLI_CONT("|PORTS CAHCE END|<\n");
		return;
	} else if (!MV_DP_SYSFS_PORTS_NUM_OK(index)) {
			MV_DP_CLI_FAIL("Illegal Port Records number: %d\n", MV_NSS_DP_INVALID_PARAM, index);
			return;
	}

	/*show single*/
	mv_dp_port_show_single(&ports[index]);
	MV_DP_CLI_CONT("|PORTS CACHE END|<\n");
}

static void mv_dp_port_alloc(int n)
{
	if (allocated) {
		MV_DP_CLI_FAIL("Already allocated: %d\n", MV_NSS_DP_FAILED, allocated);
		return;
	}
	if (n < 0 || n > MV_DP_SYSFS_PORTS_MAX) {
		MV_DP_CLI_FAIL("Illegal Port Records number: %d\n", MV_NSS_DP_INVALID_PARAM, n);
		return;
	}

	ports = kzalloc(sizeof(mv_nss_dp_port_t)*n, GFP_KERNEL);
	if (!ports) {
		MV_DP_CLI_FAIL("Cache alloc failed for Ports: %d\n", MV_NSS_DP_OUT_OF_RESOURCES, n);
		return;
	}

	allocated = n;
	MV_DP_CLI_OK("Allocated: %d\n", allocated);
}

static void mv_dp_port_delete(int id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;

	/*port id validation is to be performed in dpapi*/
	/*the port id is stored in xid*/
	res.cb = mv_dp_port_delete_cb;
	res.xid = id;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	if (MV_NSS_DP_OK != mv_nss_dp_port_delete((mv_nss_dp_port_id_t)id, &res)) {
		MV_DP_CLI_FAIL("Deleting Port %d", MV_NSS_DP_FAILED, id);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Deleting Port ID: %d\n", id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Deleting Port Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}


static void mv_dp_port_set_type(int i, int type)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	/* Clear entry before setting new type */
	memset(&(ports[i]), 0, sizeof(mv_nss_dp_port_t));
	ports[i].type = (mv_nss_dp_port_type_t)type;
	MV_DP_CLI_OK("Set Port Index %d Type = %d\n", i, ports[i].type);

}

static void mv_dp_port_set_id(int i, int id)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	ports[i].port_id = (mv_nss_dp_port_id_t)id;
	MV_DP_CLI_OK("Set Port Index %d ID = %d\n", i, ports[i].port_id);

}

static void mv_dp_port_set_type_idx(int i, int idx)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	ports[i].index = (mv_nss_dp_port_type_idx_t)idx;
	MV_DP_CLI_OK("Set Port Index %d IDX = %d\n", i, ports[i].index);

}

static void mv_dp_port_set_state(int i, int state)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	ports[i].state = (mv_nss_dp_port_state_t)state;
	MV_DP_CLI_OK("Set Port Index %d State = %d\n", i, ports[i].state);

}

static void mv_dp_port_set_eth_options(int i, int options)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_ETH) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.eth.options = (uint32_t)options;
	MV_DP_CLI_OK("Set Port Index %d ETH Options = 0x%X\n", i, options);

}

static void mv_dp_port_set_eth_mtu(int i, int mtu)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_ETH) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.eth.mtu = (uint16_t)mtu;
	MV_DP_CLI_OK("Set Port Index %d ETH MTU = %d\n", i, mtu);

}

static void mv_dp_port_set_eth_tpid(int i, int tpid)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_ETH) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.eth.tpid = (uint16_t)tpid;
	MV_DP_CLI_OK("Set Port Index %d ETH TPID = 0x%X\n", i, tpid);

}

static void mv_dp_port_set_eth_native_vlan(int i, int native_vlan)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_ETH) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.eth.native_vlan = (uint16_t)native_vlan;
	MV_DP_CLI_OK("Set Port Index %d ETH Native VLAN ID = %d\n", i, native_vlan);

}

static void mv_dp_port_set_eth_policy(int i, int policy)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_ETH) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.eth.policy = (uint16_t)policy;
	MV_DP_CLI_OK("Set Port Index %d ETH VLAN admission policy = %d\n", i, policy);

}

static void mv_dp_port_set_lag_links(int i, int lag_links)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_ETH_LAG) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.eth_lag.links = (mv_nss_dp_bitmask_t)lag_links;
	MV_DP_CLI_OK("Set Port Index %d LM = 0x%08X\n", i, ports[i].params.eth_lag.links);

}

static void mv_dp_port_set_lag_hash_prof(int i, int lag_hash_prof)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_ETH_LAG) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.eth_lag.hash_prof_id = (mv_nss_dp_hash_profile_id_t)lag_hash_prof;
	MV_DP_CLI_OK("Set Port Index %d LP = %d\n", i, ports[i].params.eth_lag.hash_prof_id);

}

static void mv_dp_port_set_cw_remote_mac(int i, unsigned char *mac)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	if (!mac) {
		MV_DP_CLI_FAIL("Null MAC ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	memcpy(ports[i].params.capwap.remote_mac.addr, mac, sizeof(ports[i].params.capwap.remote_mac.addr));
	MV_DP_CLI_OK("Set Port %d Remote MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", i,
			ports[i].params.capwap.remote_mac.addr[0],
			ports[i].params.capwap.remote_mac.addr[1],
			ports[i].params.capwap.remote_mac.addr[2],
			ports[i].params.capwap.remote_mac.addr[3],
			ports[i].params.capwap.remote_mac.addr[4],
			ports[i].params.capwap.remote_mac.addr[5]);
}

static void mv_dp_port_set_cw_local_mac(int i, unsigned char *mac)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	if (!mac) {
		MV_DP_CLI_FAIL("Null MAC ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	memcpy(ports[i].params.capwap.local_mac.addr, mac, sizeof(ports[i].params.capwap.local_mac.addr));
	MV_DP_CLI_OK("Set Port %d Local MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", i,
			ports[i].params.capwap.local_mac.addr[0],
			ports[i].params.capwap.local_mac.addr[1],
			ports[i].params.capwap.local_mac.addr[2],
			ports[i].params.capwap.local_mac.addr[3],
			ports[i].params.capwap.local_mac.addr[4],
			ports[i].params.capwap.local_mac.addr[5]);
}

static void mv_dp_port_set_cw_bssid(int i, unsigned char *mac)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	if (!mac) {
		MV_DP_CLI_FAIL("Null MAC ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	memcpy(ports[i].params.capwap.bssid.addr, mac, sizeof(ports[i].params.capwap.bssid.addr));
	MV_DP_CLI_OK("Set Port %d BSSID: %02X:%02X:%02X:%02X:%02X:%02X\n", i,
			ports[i].params.capwap.bssid.addr[0],
			ports[i].params.capwap.bssid.addr[1],
			ports[i].params.capwap.bssid.addr[2],
			ports[i].params.capwap.bssid.addr[3],
			ports[i].params.capwap.bssid.addr[4],
			ports[i].params.capwap.bssid.addr[5]);
}

static void mv_dp_port_set_cw_remote_ip(int i, int af, unsigned char *ip)
{
	const char	*end;
	unsigned char	ip_str[INET6_ADDRSTRLEN];


	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	if (!ip) {
		MV_DP_CLI_FAIL("Null IP ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (af == 4) {
		if (in4_pton(ip, -1, (u8 *)ports[i].params.capwap.remote_ip.ip, -1, &end)) {
			ports[i].params.capwap.remote_ip.ver = 4;
			mv_dp_sysfs_ip_to_str(ip_str, &(ports[i].params.capwap.remote_ip));
			MV_DP_CLI_OK("Set Port Index %d Remote IPv4: %s\n", i, ip_str);
		} else {
			MV_DP_CLI_FAIL("Invalid IPv4 format: %s\n", MV_NSS_DP_INVALID_PARAM, ip);
		}
	} else if (af == 6) {
		if (in6_pton(ip, -1, (u8 *)ports[i].params.capwap.remote_ip.ip, -1, &end)) {
			ports[i].params.capwap.remote_ip.ver = 6;
			mv_dp_sysfs_ip_to_str(ip_str, &(ports[i].params.capwap.remote_ip));
			MV_DP_CLI_OK("Set Port Index %d Remote IPv6: %s\n", i, ip_str);
		} else {
			MV_DP_CLI_FAIL("Invalid IPv6 format: %s\n", MV_NSS_DP_INVALID_PARAM, ip);
		}
	} else {
		MV_DP_CLI_FAIL("Invalid AF %d - shold be 4 or 6\n", MV_NSS_DP_INVALID_PARAM, af);
		return;
	}

}

static void mv_dp_port_set_cw_local_ip(int i, int af, unsigned char *ip)
{
	const char	*end;
	unsigned char	ip_str[INET6_ADDRSTRLEN];


	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	if (!ip) {
		MV_DP_CLI_FAIL("Null IP ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (af == 4) {
		if (in4_pton(ip, -1, (u8 *)ports[i].params.capwap.local_ip.ip, -1, &end)) {
			ports[i].params.capwap.local_ip.ver = 4;
			mv_dp_sysfs_ip_to_str(ip_str, &(ports[i].params.capwap.local_ip));
			MV_DP_CLI_OK("Set Port Index %d Local IPv4: %s\n", i, ip_str);
		} else {
			MV_DP_CLI_FAIL("Invalid IPv4 format: %s\n", MV_NSS_DP_INVALID_PARAM, ip);
		}
	} else if (af == 6) {
		if (in6_pton(ip, -1, (u8 *)ports[i].params.capwap.local_ip.ip, -1, &end)) {
			ports[i].params.capwap.local_ip.ver = 6;
			mv_dp_sysfs_ip_to_str(ip_str, &(ports[i].params.capwap.local_ip));
			MV_DP_CLI_OK("Set Port Index %d Local IPv6: %s\n", i, ip_str);
		} else {
			MV_DP_CLI_FAIL("Invalid IPv6 format: %s\n", MV_NSS_DP_INVALID_PARAM, ip);
		}
	} else {
		MV_DP_CLI_FAIL("Invalid AF %d - shold be 4 or 6\n", MV_NSS_DP_INVALID_PARAM, af);
		return;
	}

}

static void mv_dp_port_set_cw_port(int i, int port_id)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.port_id = (mv_nss_dp_port_id_t)port_id;
	MV_DP_CLI_OK("Set Port Index %d Port ID = %d\n", i, ports[i].params.capwap.port_id);

}

static void mv_dp_port_set_cw_proto(int i, int proto)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.proto = proto;
	MV_DP_CLI_OK("Set Port Index %d L4 Proto = %d\n", i, ports[i].params.capwap.proto);
}

static void mv_dp_port_set_cw_csum(int i, int val)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.calc_cs = (val) ? 1 : 0;
	MV_DP_CLI_OK("Set Port Index %d L4 Calculate CSUM = %d\n", i, ports[i].params.capwap.calc_cs);
}


static void mv_dp_port_set_cw_ttl(int i, int ttl)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.ttl = (uint32_t)ttl;
	MV_DP_CLI_OK("Set Port Index %d TTL = %d\n", i, ports[i].params.capwap.ttl);

}

static void mv_dp_port_set_cw_flow_lbl(int i, int flow_lbl)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.flow_lbl = (uint32_t)flow_lbl;
	MV_DP_CLI_OK("Set Port Index %d Flow Label = %d\n", i, ports[i].params.capwap.flow_lbl);

}

static void mv_dp_port_set_cw_dtls_index(int i, int dtls_index)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.dtls_index = (uint16_t)dtls_index;
	MV_DP_CLI_OK("Set Port Index %d DTLS Index = 0x%X\n", i, ports[i].params.capwap.dtls_index);

}

static void mv_dp_port_set_cw_remote_port(int i, int port)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n", MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.remote_port = (uint16_t)port;
	MV_DP_CLI_OK("Set Port Index %d Remote Port = %d\n", i, ports[i].params.capwap.remote_port);

}

static void mv_dp_port_set_cw_local_port(int i, int port)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.local_port = (uint16_t)port;
	MV_DP_CLI_OK("Set Port Index %d Local Port = %d\n", i, ports[i].params.capwap.local_port);

}

static void mv_dp_port_set_cw_pmtu(int i, int pmtu)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.pmtu = (uint16_t)pmtu;
	MV_DP_CLI_OK("Set Port Index %d PMTU = %d\n", i, ports[i].params.capwap.pmtu);

}

static void mv_dp_port_set_cw_vlan_id(int i, int vlan_id)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.vlan_id = (uint16_t)vlan_id;
	MV_DP_CLI_OK("Set Port Index %d VLAN ID = %d\n", i, ports[i].params.capwap.vlan_id);

}

static void mv_dp_port_set_cw_options(int i, int options)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.options = (uint8_t)options;
	MV_DP_CLI_OK("Set Port Index %d Options = 0x%08X\n", i, ports[i].params.capwap.options);

}

static void mv_dp_port_set_cw_uc_qos(int i, int uc_qos)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.uc_qos_policy = (uint8_t)uc_qos;
	MV_DP_CLI_OK("Set Port Index %d UC QOS = %d\n", i, ports[i].params.capwap.uc_qos_policy);

}

static void mv_dp_port_set_cw_mc_qos(int i, int mc_qos)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CAPWAP) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.capwap.mc_qos_policy = (uint8_t)mc_qos;
	MV_DP_CLI_OK("Set Port Index %d MC QOS = %d\n", i, ports[i].params.capwap.mc_qos_policy);

}


static void mv_dp_port_set_cpu_core_mask(int i, int core_mask)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CPU) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.cpu.cores = (mv_nss_dp_bitmask_t)core_mask;
	MV_DP_CLI_OK("Set Port Index %d Cores = 0x%X\n", i, ports[i].params.cpu.cores);

}

static void mv_dp_port_set_cpu_hash_prof(int i, int cpu_hash_prof)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_CPU) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.cpu.hash_prof_id = (uint8_t)cpu_hash_prof;
	MV_DP_CLI_OK("Set Port Index %d CPU Hash profile = %d\n", i, ports[i].params.cpu.hash_prof_id);

}

static void mv_dp_port_set_vxlan_remote_mac(int i, unsigned char *mac)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	if (!mac) {
		MV_DP_CLI_FAIL("Null MAC ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	memcpy(ports[i].params.vxlan.remote_mac.addr, mac, sizeof(ports[i].params.vxlan.remote_mac.addr));
	MV_DP_CLI_OK("Set Port %d Remote MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", i,
			ports[i].params.vxlan.remote_mac.addr[0],
			ports[i].params.vxlan.remote_mac.addr[1],
			ports[i].params.vxlan.remote_mac.addr[2],
			ports[i].params.vxlan.remote_mac.addr[3],
			ports[i].params.vxlan.remote_mac.addr[4],
			ports[i].params.vxlan.remote_mac.addr[5]);
}

static void mv_dp_port_set_vxlan_local_mac(int i, unsigned char *mac)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	if (!mac) {
		MV_DP_CLI_FAIL("Null MAC ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	memcpy(ports[i].params.vxlan.local_mac.addr, mac, sizeof(ports[i].params.vxlan.local_mac.addr));
	MV_DP_CLI_OK("Set Port %d local MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", i,
			ports[i].params.vxlan.local_mac.addr[0],
			ports[i].params.vxlan.local_mac.addr[1],
			ports[i].params.vxlan.local_mac.addr[2],
			ports[i].params.vxlan.local_mac.addr[3],
			ports[i].params.vxlan.local_mac.addr[4],
			ports[i].params.vxlan.local_mac.addr[5]);
}

static void mv_dp_port_set_vxlan_remote_ip(int i, int af, unsigned char *ip)
{
	const char	*end;
	unsigned char	ip_str[INET6_ADDRSTRLEN];


	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	if (!ip) {
		MV_DP_CLI_FAIL("Null IP ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (af == 4) {
		if (in4_pton(ip, -1, (u8 *)ports[i].params.vxlan.remote_ip.ip, -1, &end)) {
			ports[i].params.vxlan.remote_ip.ver = 4;
			mv_dp_sysfs_ip_to_str(ip_str, &(ports[i].params.vxlan.remote_ip));
			MV_DP_CLI_OK("Set Port Index %d Remote IPv4: %s\n", i, ip_str);
		} else {
			MV_DP_CLI_FAIL("Invalid IPv4 format: %s\n", MV_NSS_DP_INVALID_PARAM, ip);
		}
	} else if (af == 6) {
		if (in6_pton(ip, -1, (u8 *)ports[i].params.vxlan.remote_ip.ip, -1, &end)) {
			ports[i].params.vxlan.remote_ip.ver = 6;
			mv_dp_sysfs_ip_to_str(ip_str, &(ports[i].params.vxlan.remote_ip));
			MV_DP_CLI_OK("Set Port Index %d Remote IPv6: %s\n", i, ip_str);
		} else {
			MV_DP_CLI_FAIL("Invalid IPv6 format: %s\n", MV_NSS_DP_INVALID_PARAM, ip);
		}
	} else {
		MV_DP_CLI_FAIL("Invalid AF %d - shold be 4 or 6\n", MV_NSS_DP_INVALID_PARAM, af);
		return;
	}

}

static void mv_dp_port_set_vxlan_local_ip(int i, int af, unsigned char *ip)
{
	const char	*end;
	unsigned char	ip_str[INET6_ADDRSTRLEN];


	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	if (!ip) {
		MV_DP_CLI_FAIL("Null IP ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (af == 4) {
		if (in4_pton(ip, -1, (u8 *)ports[i].params.vxlan.local_ip.ip, -1, &end)) {
			ports[i].params.vxlan.local_ip.ver = 4;
			mv_dp_sysfs_ip_to_str(ip_str, &(ports[i].params.vxlan.local_ip));
			MV_DP_CLI_OK("Set Port Index %d local IPv4: %s\n", i, ip_str);
		} else {
			MV_DP_CLI_FAIL("Invalid IPv4 format: %s\n", MV_NSS_DP_INVALID_PARAM, ip);
		}
	} else if (af == 6) {
		if (in6_pton(ip, -1, (u8 *)ports[i].params.vxlan.local_ip.ip, -1, &end)) {
			ports[i].params.vxlan.local_ip.ver = 6;
			mv_dp_sysfs_ip_to_str(ip_str, &(ports[i].params.vxlan.local_ip));
			MV_DP_CLI_OK("Set Port Index %d Remote IPv6: %s\n", i, ip_str);
		} else {
			MV_DP_CLI_FAIL("Invalid IPv6 format: %s\n", MV_NSS_DP_INVALID_PARAM, ip);
		}
	} else {
		MV_DP_CLI_FAIL("Invalid AF %d - shold be 4 or 6\n", MV_NSS_DP_INVALID_PARAM, af);
		return;
	}

}

static void mv_dp_port_set_vxlan_port(int i, int port_id)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.vxlan.port_id = (mv_nss_dp_port_id_t)port_id;
	MV_DP_CLI_OK("Set Port Index %d Port ID = %d\n", i, ports[i].params.vxlan.port_id);

}

static void mv_dp_port_set_vxlan_proto(int i, int proto)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.vxlan.proto = proto;
	MV_DP_CLI_OK("Set Port Index %d L4 Proto = %d\n", i, ports[i].params.vxlan.proto);
}

static void mv_dp_port_set_vxlan_csum(int i, int val)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.vxlan.calc_cs = (val) ? 1 : 0;
	MV_DP_CLI_OK("Set Port Index %d L4 Calculate CSUM = %d\n", i, ports[i].params.vxlan.calc_cs);
}


static void mv_dp_port_set_vxlan_ttl(int i, int ttl)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.vxlan.ttl = (uint32_t)ttl;
	MV_DP_CLI_OK("Set Port Index %d TTL = %d\n", i, ports[i].params.vxlan.ttl);

}

static void mv_dp_port_set_vxlan_flow_lbl(int i, int flow_lbl)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.vxlan.flow_lbl = (uint32_t)flow_lbl;
	MV_DP_CLI_OK("Set Port Index %d Flow Label = %d\n", i, ports[i].params.vxlan.flow_lbl);

}

static void mv_dp_port_set_vxlan_remote_port(int i, int port)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.vxlan.remote_port = (uint16_t)port;
	MV_DP_CLI_OK("Set Port Index %d Remote Port = %d\n", i, ports[i].params.vxlan.remote_port);

}

static void mv_dp_port_set_vxlan_local_port(int i, int port)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.vxlan.local_port = (uint16_t)port;
	MV_DP_CLI_OK("Set Port Index %d Local Port = %d\n", i, ports[i].params.vxlan.local_port);

}

static void mv_dp_port_set_vxlan_vlan_id(int i, int vlan_id)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.vxlan.vlan_id = (uint16_t)vlan_id;
	MV_DP_CLI_OK("Set Port Index %d VLAN ID = %d\n", i, ports[i].params.vxlan.vlan_id);

}

static void mv_dp_port_set_vxlan_options(int i, int options)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.vxlan.options = (uint8_t)options;
	MV_DP_CLI_OK("Set Port Index %d Options = 0x%08X\n", i, ports[i].params.vxlan.options);

}

static void mv_dp_port_set_vxlan_uc_qos(int i, int uc_qos)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.vxlan.uc_qos_policy = (uint8_t)uc_qos;
	MV_DP_CLI_OK("Set Port Index %d UC QOS = %d\n", i, ports[i].params.vxlan.uc_qos_policy);

}

static void mv_dp_port_set_vxlan_mc_qos(int i, int mc_qos)
{
	if (!MV_DP_SYSFS_PORTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Port index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (ports[i].type != MV_NSS_DP_PORT_VXLAN) {
		MV_DP_CLI_FAIL("Illegal field for Cache Port index: %d type: %d\n",
				MV_NSS_DP_INVALID_PARAM, i, ports[i].type);
		return;
	}

	ports[i].params.vxlan.mc_qos_policy = (uint8_t)mc_qos;
	MV_DP_CLI_OK("Set Port Index %d MC QOS = %d\n", i, ports[i].params.vxlan.mc_qos_policy);

}

static void mv_dp_vxlan_vni_cfg_set(int id, int ind, int vni)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_vxlan_vni_cfg_t cfg;
	/*the index is stored in xid*/
	res.cb = mv_dp_vxlan_vni_cfg_set_cb;
	res.xid = id;
	res.cookie = 0;

	cfg.port_id = id;
	cfg.index = ind;
	cfg.vni = vni;

	if (mv_nss_dp_vxlan_vni_cfg_set(&cfg, &res) != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL("VXLAN VNI CFG Set for Port %d Index %d\n", MV_NSS_DP_FAILED, id, ind);
		return;
	}

	MV_DP_CLI_TMSG("VXLAN VNI CFG Set for Port %d Index %d\n", id, ind);

}

static void mv_dp_vxlan_vni_cfg_get(int id, int ind)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_vxlan_vni_cfg_t cfg;
	/*the index is stored in xid*/
	res.cb = mv_dp_vxlan_vni_cfg_get_cb;
	res.xid = id;
	res.cookie = 0;

	cfg.port_id = id;
	cfg.index = ind;
	cfg.vni = 0;

	if (mv_nss_dp_vxlan_vni_cfg_get(&cfg, &res) != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL("VXLAN VNI CFG Get for Port %d Index %d\n", MV_NSS_DP_FAILED, id, ind);
		return;
	}

	MV_DP_CLI_TMSG("VXLAN VNI CFG Get for Port %d Index %d\n", id, ind);

}

static void mv_dp_vxlan_vni_cfg_delete(int id, int ind)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_vxlan_vni_cfg_t cfg;
	/*the index is stored in xid*/
	res.cb = mv_dp_vxlan_vni_cfg_del_cb;
	res.xid = id;
	res.cookie = 0;

	cfg.port_id = id;
	cfg.index = ind;
	cfg.vni = 0;

	if (mv_nss_dp_vxlan_vni_cfg_delete(&cfg, &res) != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL("VXLAN VNI CFG Delete for Port %d Index %d\n", MV_NSS_DP_FAILED, id, ind);
		return;
	}

	MV_DP_CLI_TMSG("VXLAN VNI CFG Delete for Port %d Index %d\n", id, ind);

}

static void mv_dp_port_set_cb(mv_nss_dp_event_t *event)
{

	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Set Port Index: %d Status: %s\n", event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("Set Port Index: %d Port ID: %d\n", event, event->xid, event->params.port->port_id);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);
}

static void mv_dp_port_dflt_dest_set_cb(mv_nss_dp_event_t *event)
{

	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Set Dflt dest Port: %d Status: %s\n", event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("Set Dflt dest Port: %d\n", event, event->xid);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}


static void mv_dp_port_get_cb(mv_nss_dp_event_t *event)
{

	/*print response and return OK;*/
	/*Port ID is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get Port: %d Status: %s\n", event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.port) {
		MV_DP_CLI_FAIL_CB("Get Port: %d Empty Port\n", event, event->xid);
		goto err;
	}

	MV_DP_CLI_OK_CB("Get Port: %d\n", event, event->xid);
	mv_dp_port_show_single(event->params.port);
	MV_DP_CLI_CONT("|Port END|<\n");

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

static void mv_dp_port_delete_cb(mv_nss_dp_event_t *event)
{

	/*save response in the port buffer and return OK;*/
	/*Port ID is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Delete Port %d Status: %s\n", event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Deleted Port ID: %d\n", event, event->xid);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

static void mv_dp_port_read_cb(mv_nss_dp_event_t *event)
{

	/*save response in the port buffer and return OK;*/
	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get Port to Cache Index: %d Status: %s\n", event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.port) {
		MV_DP_CLI_FAIL_CB("Get Port to Cache Index: %d Empty Port\n", event, event->xid);
		goto err;
	}

	if (!MV_DP_SYSFS_PORTS_NUM_OK(event->xid)) {
		MV_DP_CLI_FAIL_CB("Illegal Cache Port index: %d\n", event, event->xid);
		goto err;
	}

	memcpy(&ports[event->xid], event->params.port, sizeof(mv_nss_dp_port_t));

	MV_DP_CLI_OK_CB("Get Port to Cache Index: %d\n", event, event->xid);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

static void mv_dp_port_stats_get_cb(mv_nss_dp_event_t *event)
{

	/*save response in the port buffer and return OK;*/
	/*Port ID is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get Port %d Stats: Status: %s\n", event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.port_stats) {
		MV_DP_CLI_FAIL_CB("Get Port %d Stats: Empty Stats\n", event, event->xid);
		goto err;
	}

	MV_DP_CLI_OK_CB("Get Port %d Stats\n", event, event->xid);
	MV_DP_CLI_CONT("|ID |T|RX PKTS   |RX ERR    |TX PKTS   |TX ERR    |RX OCTETS |TX OCTETS |<\n");
	mv_dp_port_show_stats_single(event->params.port_stats);
	MV_DP_CLI_CONT("|PORT STATS END|<\n");

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}


static void mv_dp_port_link_state_get_cb(mv_nss_dp_event_t *event)
{

	/*Port ID is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get link state Port %d Stats: Status: %s\n", event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.eth_link_state) {
		MV_DP_CLI_FAIL_CB("Get link state Port %d Stats: Empty Stats\n", event, event->xid);
		goto err;
	}

	MV_DP_CLI_OK_CB("Get Link State Port %d\n", event, event->xid);
	MV_DP_CLI_CONT("|PORT ID| SPEED | STATE | DPLX |<\n");
	mv_dp_port_link_state_show_single(event->params.eth_link_state);
	MV_DP_CLI_CONT("|PORT LINK STATE END|<\n");

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}


static void mv_dp_port_stats_bulk_get_cb(mv_nss_dp_event_t *event)
{
	mv_nss_dp_port_stats_t		*stats_ptr;
	int				i;


	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	stats_ptr = event->params.port_stats;

	if (!stats_ptr) {
		MV_DP_CLI_FAIL_CB("null stats buff ptr\n", event);
		goto err;
	}

	if (!(MV_NSS_DP_OK == event->status || MV_NSS_DP_END_OF_LIST == event->status)) {

		MV_DP_CLI_FAIL_CB("GetBulk Statistics for %d entries Status: %s\n",
				  event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("|PORTS STATS|TOTAL:%d|REQESTED:%d|<\n", event, event->count, event->xid);
	MV_DP_CLI_CONT("|IDX|ID |T|RX PKTS   |RX ERR    |TX PKTS   |TX ERR    |RX OCTETS |TX OCTETS |<\n");
	for (i = 0; i < event->count; i++) {
		MV_DP_CLI_CONT("|%3d", i);
		mv_dp_port_show_stats_single(stats_ptr + i);
	}

	MV_DP_CLI_CONT("|PORTS STATS END|<\n");

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);
	if (stats_ptr)
		kfree(stats_ptr);

}

static void mv_dp_port_stats_reset_cb(mv_nss_dp_event_t *event)
{

	/*save response in the port buffer and return OK;*/
	/*Port ID is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Reset statistics for Port %d Status: %s\n", event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Reset statistics for Port %d\n", event, event->xid);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

static void mv_dp_vxlan_vni_cfg_set_cb(mv_nss_dp_event_t *event)
{

	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (event->status != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL_CB("Set Port %d VXLAN VNI Status: %s\n",
				event, event->xid, mv_dp_err_name_get(event->status));
		return;
	}


	MV_DP_CLI_OK_CB("Set Port ID: %d VXLAN VNI\n", event, event->xid);

}

static void mv_dp_vxlan_vni_cfg_get_cb(mv_nss_dp_event_t *event)
{

	/*Port ID is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (event->status != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL_CB("Get Port %d VXLAN VNI Status: %s\n",
				event, event->xid, mv_dp_err_name_get(event->status));
		return;
	}

	if (!event->params.vxlan_vni_cfg) {
		MV_DP_CLI_FAIL_CB("Get Port %d VXLAN VNI\n", event, event->xid);
		return;
	}

	MV_DP_CLI_OK_CB("Get VXLAN VNI configuration\n", event);
	MV_DP_CLI_CONT("|PORT ID|Index|VNI     |<\n");
	MV_DP_CLI_CONT("|%7d|%5d|0x%06x|\n", event->params.vxlan_vni_cfg->port_id,
			event->params.vxlan_vni_cfg->index,
			event->params.vxlan_vni_cfg->vni);
	MV_DP_CLI_CONT("|VXLAN VNI CFG END|<\n");

}

static void mv_dp_vxlan_vni_cfg_del_cb(mv_nss_dp_event_t *event)
{

	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (event->status != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL_CB("Delete Port %d VXLAN VNI Status: %s\n",
				event, event->xid, mv_dp_err_name_get(event->status));
		return;
	}


	MV_DP_CLI_OK_CB("Delete Port ID: %d VXLAN VNI\n", event, event->xid);

}
