
/*******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
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
#include <linux/slab.h>
#include <linux/inet.h>

#include "mv_dp_defs.h"
#include "mv_dp_sysfs.h"
#include "mv_nss_dp.h"
#include "mv_dp_main.h"
#include "mv_dp_int_if.h"



#define MV_DP_SYSFS_FLOWS_MAX		(4*1024)
#define MV_DP_SYSFS_FLOWS_NUM_OK(n)	((n) >= 0 && (n) < allocated)

enum mv_dp_sysfs_param_type {
	param_type_cls = 0,
	param_type_act = 1,
	param_type_last
};

static char *param_names[param_type_last] = {
				"CLS",
				"ACT"
				};

enum mv_dp_sysfs_dir_type {
	dir_type_src = 0,
	dir_type_dst = 1,
	dir_type_param_last
};
static char *dir_names[dir_type_param_last] = {
				"SRC",
				"DST"
				};


enum mv_dp_sysfs_flow_status {
	flow_status_inactive = 0,
	flow_status_active = 1,
	flow_status_last
};

static char *l4_proto_names[MV_NSS_DP_PROTO_TCP + 1] = {
				"UDP",
				"UDP LITE",
				"TCP"
				};
#define MV_DP_SYS_L4_PROTO_IS_OK(v) ((v) >= MV_NSS_DP_PROTO_UDP && (v) <= MV_NSS_DP_PROTO_TCP)

#define MV_DP_SYS_FLOW_FIELD_LAST  (MV_NSS_DP_PRIO_ACTION_BIT+1)
static char *flow_field_names[MV_DP_SYS_FLOW_FIELD_LAST] = {
				"PORT_DST",
				"PORT_SRC",
				"L2_DST",
				"L2_SRC",
				"ETH_TYPE",
				"LLC_SSAP",
				"LLC_DSAP",
				"SNAP",
				"IP_DST",
				"IP_SRC",
				"L4_PORT_DST",
				"L4_PORT_SRC",
				"L4_PROTOCOL",
				"FLOW_LABEL",
				"PRIORITY",
				"OPAQUE",
				"PRIO ACTION"
				};
#define MV_DP_SYS_FLOW_FIELD_IS_OK(v) ((v) >= MV_NSS_DP_PORT_DST_BIT && (v) < MV_DP_SYS_FLOW_FIELD_LAST)

static char *prio_act_names[MV_DP_NSS_PRIO_ACTION_QOS_PLCY + 1] = {
				"NONE",
				"FIXED",
				"QOS_POLICY"
				};

#define MV_DP_SYS_PRI_ACT_IS_OK(v) ((v) >= MV_DP_NSS_PRIO_ACTION_NONE && (v) <= MV_DP_NSS_PRIO_ACTION_QOS_PLCY)


static  mv_nss_dp_flow_t	*flows;
static  int			allocated; /*number of allocated entries*/

static void flow_present_bit_help(void);
static void flow_present_bit_show(u32 val);
static void mv_dp_flow_get(int flow_id);
static void mv_dp_flow_stats_bulk_get(int i, int cnt, u32 options);
static void mv_dp_flow_read(int ind, int flow_id);

static void mv_dp_flow_commit(int ind);
static void mv_dp_flow_show(int ind);
static void mv_dp_flow_show_single(mv_nss_dp_flow_t *flow);
static void mv_dp_param_show_single(mv_nss_dp_params_t *param);

static void mv_dp_flow_alloc(int i);
static void mv_dp_flow_release(void);
static void mv_dp_flow_clear(int i);
static void mv_dp_flow_delete(int flow_id);
static void mv_dp_flow_delete_all(void);
static void mv_dp_flow_delete_multiple(void);


/*set flow params: index, type (cls or action), value*/
static void mv_dp_flow_set_mac(int ind,
				 enum mv_dp_sysfs_param_type p_type,
				 enum mv_dp_sysfs_dir_type m_type,
				 unsigned char *mac);
static void mv_dp_flow_set_ip(int ind,
				 enum mv_dp_sysfs_param_type p_type,
				 enum mv_dp_sysfs_dir_type ip_type,
				 int af, unsigned char *ip);

static void mv_dp_flow_set_flow_id(int ind, int flow_id);
static void mv_dp_flow_set_idle(int ind, u32 timeout);
static void mv_dp_flow_set_status(int ind, int status);
static void mv_dp_flow_set_od(int i, enum mv_dp_sysfs_param_type p_type, u32 od);
static void mv_dp_flow_set_present(int ind, enum mv_dp_sysfs_param_type p_type, u32 present);
static void mv_dp_flow_rst_present(int i, enum mv_dp_sysfs_param_type p_type, int b);
static void mv_dp_flow_set_flbl(int ind, enum mv_dp_sysfs_param_type p_type, u32 flbl);
static void mv_dp_flow_set_etht(int ind, enum mv_dp_sysfs_param_type p_type, u16 etht);
static void mv_dp_flow_set_llc_ssap(int ind, enum mv_dp_sysfs_param_type p_type, u16 snap);
static void mv_dp_flow_set_llc_dsap(int ind, enum mv_dp_sysfs_param_type p_type, u16 snap);
static void mv_dp_flow_set_l4_dst_prt(int ind, enum mv_dp_sysfs_param_type p_type, u16 prt);
static void mv_dp_flow_set_l4_src_prt(int ind, enum mv_dp_sysfs_param_type p_type, u16 prt);
static void mv_dp_flow_set_dst_prt(int ind, enum mv_dp_sysfs_param_type p_type, u8 prt);
static void mv_dp_flow_set_src_prt(int ind, enum mv_dp_sysfs_param_type p_type, u8 prt);
static void mv_dp_flow_set_snap(int ind, enum mv_dp_sysfs_param_type p_type, u8 *snap);
static void mv_dp_flow_set_l4_proto(int ind, enum mv_dp_sysfs_param_type p_type, u8 proto);
static void mv_dp_flow_set_prio_act(int ind, enum mv_dp_sysfs_param_type p_type, u8 act);
static void mv_dp_flow_set_prio_val(int ind, enum mv_dp_sysfs_param_type p_type, u8 val);

/*Status only manipualtion*/
static void mv_dp_flow_set_fw_status(int ind, int status);
static void mv_dp_flow_get_fw_status(int ind);
static void mv_dp_flow_fw_status_set_multiple(void);

static void mv_dp_flow_get_stats(int flow_id);
static void mv_dp_flow_reset_stats(int flow_id);
static void mv_dp_flow_show_stats_single(mv_nss_dp_flow_stats_t *stats);

static void mv_dp_flow_get_count(void);

/*cb*/
static void mv_dp_flow_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_flow_read_cb(mv_nss_dp_event_t *event);
static void mv_dp_flow_delete_cb(mv_nss_dp_event_t *event);
static void mv_dp_flow_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_flow_delete_all_cb(mv_nss_dp_event_t *event);
static void mv_dp_flow_set_flow_status_cb(mv_nss_dp_event_t *event);
static void mv_dp_flow_get_flow_status_cb(mv_nss_dp_event_t *event);
static void mv_dp_flow_get_flow_stats_cb(mv_nss_dp_event_t *event);
static void mv_dp_flow_reset_flow_stats_cb(mv_nss_dp_event_t *event);
static void mv_dp_flow_get_count_cb(mv_nss_dp_event_t *event);
static void mv_dp_flow_stats_bulk_get_cb(mv_nss_dp_event_t *event);



static ssize_t mv_dp_flow_sysfs_help(char *b)
{
	int o = 0;

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                    help                - Show Help\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                    flow_release        - Release Flows Cache\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                    flow_delete_all     - Delete ALL Flows from FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                    flow_count_get      - Get Flow COunt from FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                    flow_present_bit_help - Show bit values help for present field\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                    flow_delete_multi   - Delete Flows from FW for ALL IDs stored in cache)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		      "cat                     flow_fw_status_set_multi - Set Status in FW for ALL IDs stored in cache\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]               > flow_clear          - Clear Flowss Cache [i] to 0 (no release)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [ID]          > flow_read           - Read Flow Record by [ID] from FW to cache index [i]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]              > flow_get            - Print Flow Record by its ID [ID] (not saved to cache)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]               > flow_commit         - Save Flow stored at index [i] to FW, -1 for all\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]               > flow_alloc          - Allocate Flow cache of [i] entries\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]               > flow_show           - Show Flow cache entry [i] or -1 for ALL\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]              > flow_delete         - Delete Flow Record from FW by its [ID]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID] [ST]         > flow_fw_status_set  - Set Flow[id] status in the FW to[st]: 0 or 1\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]              > flow_fw_status_get  - Get Flow[id] status from the FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [IND] [CNT] [OPT] > flow_stats_bulk_get - Get [CNT] Flows stats from Flow index[IND] with OPTS\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]              > flow_stats_get      - Get Flow[id] statistics from the FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [id]              > flow_stats_rst      - Reset Flow[id] statistics in the FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [ID]          > flow_id_set         - Set Flow ID [id] to cache index[i]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [sec]         > flow_idle_set       - Set Flow idle timeout[sec] to cache index[i]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [ST]          > flow_status_set     - Set Cached Flow[i] status to [st] 0 or 1\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [PR]     > flow_present_set    - Set Cached Flow[i] present field to [PR] (hex 32)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [b]      > flow_present_rst    - Reset a bit in  Cached Flow[i] present field\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [DT] [MAC] > flow_mac_set      - Set Cached Flow[i]'s [DT] mac[MAC] for [PT]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [DT] [AF] [IP] >flow_ip_set    - Set Cached Flow[i]'s [DT] IP[IP] AF[AF] for [PT]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [OD]     > flow_od_set         - Set Cached Flow[i]'s p_type opaque data [OD](hex 32)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [LBL]    > flow_flbl_set       - Set Cached Flow[i] flow label to [LBL](hex 32)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [ETH]    > flow_etht_set       - Set Cached Flow[i] eth type to [ETH] (hex 16)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [AP]     > flow_llc_ssap_set   - Set Cached Flow[i] llc ssap to [AP] (hex 16)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [AP]     > flow_llc_dsap_set   - Set Cached Flow[i] llc dsap to [AP] (hex 16)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [L4PRT]  > flow_l4_dst_prt_set - Set Cached Flow[i] L4 DST port to[L4PRT] (dec 16)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [L4PRT]  > flow_l4_src_prt_set - Set Cached Flow[i] L4 SRC port to[L4PRT] (dec 16)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [PRT]    > flow_dst_prt_set    - Set Cached Flow[i] DST port to[PRT] (dec 8)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [PRT]    > flow_src_prt_set    - Set Cached Flow[i] SRC port to[PRT] (dec 8)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [SNP]    > flow_snap_set       - Set Cached Flow[i] SNAP to[SNP] (00:00:00:00:00)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [L4PROTO]> flow_l4_proto_set   - Set Cached Flow[i] L4 PROTO to[L4PROTO]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [PRI_ACT]> flow_prio_act_set   - Set Cached Flow[i] Prio Action to[PRI_ACT]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PT] [PRI_VAL]> flow_prio_val_set   - Set Cached Flow[i] Prio VALUE to[PRI_VAL](dec 8)\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [MAC]= MAC address as 00:00:00:00:00:00");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   dec - decimal 123; hex - hexadecimal (0x000)");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [PT]= Param type: CLS[%d] or ACT[%d]", param_type_cls, param_type_act);
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [DT]= IP/MAC addr type: DST[%d] or SRC[%d]", dir_type_dst, dir_type_src);
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [AF]= IP addr family: 4 or 6");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [PRI_ACT]= 0 - NONE, 1 - FIXED, 2 - QOS_POLICY");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [L4PROTO]= %d-%s,%d-%s,%d-%s",
		       MV_NSS_DP_PROTO_UDP, l4_proto_names[MV_NSS_DP_PROTO_UDP],
		       MV_NSS_DP_PROTO_UDP_LITE, l4_proto_names[MV_NSS_DP_PROTO_UDP_LITE],
		       MV_NSS_DP_PROTO_TCP, l4_proto_names[MV_NSS_DP_PROTO_TCP]
		       );
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [IP]  = IP address as 0.0.0.0 or 0:0:0:0:0:0:0:0");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [sec]= Flow Idle timeout in sec");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [OPT] = Get Flow stats options: 0x0-ALL; 0x1-Aged only\n");


	return o;
}

static ssize_t mv_dp_flow_sysfs_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int off = 0;
	const char  *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "help")) {
		return mv_dp_flow_sysfs_help(buf);
	} else if (!strcmp(name, "flow_release")) {
			mv_dp_flow_release();
	} else if (!strcmp(name, "flow_delete_all")) {
			mv_dp_flow_delete_all();
	} else if (!strcmp(name, "flow_count_get")) {
			mv_dp_flow_get_count();
	} else if (!strcmp(name, "flow_present_bit_help")) {
			flow_present_bit_help();
	} else if (!strcmp(name, "flow_delete_multi")) {
			mv_dp_flow_delete_multiple();
	} else if (!strcmp(name, "flow_fw_status_set_multi")) {
			mv_dp_flow_fw_status_set_multiple();

	} else {
		MV_DP_CLI_FAIL("Parse Error CMD: <%s>\n", MV_NSS_DP_INVALID_PARAM, name);
		off = mv_dp_flow_sysfs_help(buf);
	}
	return off;
}

static ssize_t mv_dp_flow_sysfs_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t len)
{
	const char *name = attr->attr.name;
	unsigned int  a, b, c, d;
	unsigned char mac[6];
	unsigned char ip[INET6_ADDRSTRLEN];


	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = b = c = 0;

	if (!strcmp(name, "flow_read")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_flow_read(a, b);
	} else	if (!strcmp(name, "flow_get")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_flow_get(a);
	} else if (!strcmp(name, "flow_commit")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_flow_commit(a);
	} else if (!strcmp(name, "flow_clear")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_flow_clear(a);
	} else if (!strcmp(name, "flow_show")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_flow_show(a);
	} else if (!strcmp(name, "flow_alloc")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_flow_alloc(a);
	} else if (!strcmp(name, "flow_delete")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_flow_delete(a);
	} else if (!strcmp(name, "flow_id_set")) {
		if (2 != sscanf(buf, "%d %i", &a, &b))
			goto err;
		mv_dp_flow_set_flow_id(a, b);
	} else if (!strcmp(name, "flow_status_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_flow_set_status(a, b);
	} else if (!strcmp(name, "flow_idle_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_flow_set_idle(a, b);
	} else if (!strcmp(name, "flow_fw_status_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_flow_set_fw_status(a, b);
	} else if (!strcmp(name, "flow_fw_status_get")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_flow_get_fw_status(a);
	} else if (!strcmp(name, "flow_stats_get")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_flow_get_stats(a);
	} else if (!strcmp(name, "flow_stats_bulk_get")) {
		if (3 != sscanf(buf, "%d %d %i", &a, &b, &c))
			goto err;
		mv_dp_flow_stats_bulk_get(a, b, c);
	} else if (!strcmp(name, "flow_stats_rst")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_flow_reset_stats(a);
	} else if (!strcmp(name, "flow_mac_set")) {
		if (9 != sscanf(buf, "%d %d %d %hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		       &a, &b, &c, &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
			goto err;
		mv_dp_flow_set_mac(a, b, c, mac);
	} else if (!strcmp(name, "flow_ip_set")) {
		if (5 != sscanf(buf, "%d %d %d %d %s", &a, &b, &c, &d, ip))
			goto err;
		mv_dp_flow_set_ip(a, b, c, d, ip);
	} else if (!strcmp(name, "flow_od_set")) {
		if (3 != sscanf(buf, "%d %d 0x%8X", &a, &b, &c))
			goto err;
		mv_dp_flow_set_od(a, b, c);
	} else if (!strcmp(name, "flow_present_set")) {
		if (3 != sscanf(buf, "%d %d 0x%8X", &a, &b, &c))
			goto err;
		mv_dp_flow_set_present(a, b, c);
	} else if (!strcmp(name, "flow_present_rst")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_flow_rst_present(a, b, c);
	} else if (!strcmp(name, "flow_flbl_set")) {
		if (3 != sscanf(buf, "%d %d 0x%8X", &a, &b, &c))
			goto err;
		mv_dp_flow_set_flbl(a, b, c);
	} else if (!strcmp(name, "flow_etht_set")) {
		if (3 != sscanf(buf, "%d %d 0x%4X", &a, &b, &c))
			goto err;
		mv_dp_flow_set_etht(a, b, c);
	} else if (!strcmp(name, "flow_llc_ssap_set")) {
		if (3 != sscanf(buf, "%d %d 0x%4X", &a, &b, &c))
			goto err;
		mv_dp_flow_set_llc_ssap(a, b, c);
	} else if (!strcmp(name, "flow_llc_dsap_set")) {
		if (3 != sscanf(buf, "%d %d 0x%4X", &a, &b, &c))
			goto err;
		mv_dp_flow_set_llc_dsap(a, b, c);
	} else if (!strcmp(name, "flow_l4_dst_prt_set")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_flow_set_l4_dst_prt(a, b, c);
	} else if (!strcmp(name, "flow_l4_src_prt_set")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_flow_set_l4_src_prt(a, b, c);
	} else if (!strcmp(name, "flow_src_prt_set")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_flow_set_src_prt(a, b, c);
	} else if (!strcmp(name, "flow_dst_prt_set")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_flow_set_dst_prt(a, b, c);
	} else if (!strcmp(name, "flow_snap_set")) {
		if (7 != sscanf(buf, "%d %d %hhx:%hhx:%hhx:%hhx:%hhx",
		       &a, &b, &mac[0], &mac[1], &mac[2], &mac[3], &mac[4]))
			goto err;
		mv_dp_flow_set_snap(a, b, mac);
	} else if (!strcmp(name, "flow_l4_proto_set")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_flow_set_l4_proto(a, b, c);
	} else if (!strcmp(name, "flow_prio_act_set")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_flow_set_prio_act(a, b, c);
	} else if (!strcmp(name, "flow_prio_val_set")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_flow_set_prio_val(a, b, c);

	} else {
		goto err;
	}

	return len;
err:
	MV_DP_LOG_INF("Parse Error CMD:<%s>\n", attr->attr.name);
	return -EINVAL;
}

static DEVICE_ATTR(help,			S_IRUSR, mv_dp_flow_sysfs_show, NULL);
static DEVICE_ATTR(flow_release,		S_IRUSR, mv_dp_flow_sysfs_show, NULL);
static DEVICE_ATTR(flow_delete_all,		S_IRUSR, mv_dp_flow_sysfs_show, NULL);
static DEVICE_ATTR(flow_present_bit_help,	S_IRUSR, mv_dp_flow_sysfs_show, NULL);
static DEVICE_ATTR(flow_count_get,		S_IRUSR, mv_dp_flow_sysfs_show, NULL);
static DEVICE_ATTR(flow_delete_mult,		S_IRUSR, mv_dp_flow_sysfs_show, NULL);
static DEVICE_ATTR(flow_fw_status_set_mult,	S_IRUSR, mv_dp_flow_sysfs_show, NULL);

static DEVICE_ATTR(flow_clear,			S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_read,			S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_commit,			S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_show,			S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_get,			S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_alloc,			S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_delete,			S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_mac_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_ip_set,			S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_od_set,			S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_id_set,			S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_status_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_present_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_flbl_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_etht_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_llc_ssap_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_llc_dsap_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_l4_dst_prt_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_l4_src_prt_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_src_prt_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_dst_prt_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_snap_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_l4_proto_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_prio_act_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_prio_val_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_fw_status_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_fw_status_get,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_stats_get,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_stats_bulk_get,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_stats_rst,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_present_rst,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);
static DEVICE_ATTR(flow_idle_set,		S_IWUSR, NULL, mv_dp_flow_sysfs_store);



static struct attribute *mv_dp_flow_sysfs_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_flow_present_bit_help.attr,
	&dev_attr_flow_clear.attr,
	&dev_attr_flow_delete_all.attr,
	&dev_attr_flow_delete_mult.attr,
	&dev_attr_flow_fw_status_set_mult.attr,
	&dev_attr_flow_release.attr,
	&dev_attr_flow_read.attr,
	&dev_attr_flow_commit.attr,
	&dev_attr_flow_show.attr,
	&dev_attr_flow_get.attr,
	&dev_attr_flow_alloc.attr,
	&dev_attr_flow_delete.attr,
	&dev_attr_flow_mac_set.attr,
	&dev_attr_flow_ip_set.attr,
	&dev_attr_flow_od_set.attr,
	&dev_attr_flow_id_set.attr,
	&dev_attr_flow_status_set.attr,
	&dev_attr_flow_present_set.attr,
	&dev_attr_flow_present_rst.attr,
	&dev_attr_flow_flbl_set.attr,
	&dev_attr_flow_etht_set.attr,
	&dev_attr_flow_llc_ssap_set.attr,
	&dev_attr_flow_llc_dsap_set.attr,
	&dev_attr_flow_l4_dst_prt_set.attr,
	&dev_attr_flow_l4_src_prt_set.attr,
	&dev_attr_flow_src_prt_set.attr,
	&dev_attr_flow_dst_prt_set.attr,
	&dev_attr_flow_snap_set.attr,
	&dev_attr_flow_l4_proto_set.attr,
	&dev_attr_flow_prio_act_set.attr,
	&dev_attr_flow_prio_val_set.attr,
	&dev_attr_flow_fw_status_set.attr,
	&dev_attr_flow_fw_status_get.attr,
	&dev_attr_flow_stats_get.attr,
	&dev_attr_flow_stats_bulk_get.attr,
	&dev_attr_flow_stats_rst.attr,
	&dev_attr_flow_count_get.attr,
	&dev_attr_flow_idle_set.attr,



	NULL
};


static struct attribute_group mv_dp_flow_sysfs_group = {
	.name = "flow",
	.attrs = mv_dp_flow_sysfs_attrs,
};



int mv_dp_flow_sysfs_init(struct kobject *ko)
{
	int err;
	err = sysfs_create_group(ko, &mv_dp_flow_sysfs_group);

	if (err) {
		MV_DP_LOG_INF("Flow sysFS group init failed %d\n", err);
		return err;
	}

	MV_DP_LOG_DBG1("Flow sysFS INITALIZED\n");
	return err;
}

int mv_dp_flow_sysfs_exit(struct kobject *ko)
{
	sysfs_remove_group(ko, &mv_dp_flow_sysfs_group);
	if (allocated) {
		kfree(flows);
		allocated = 0;
	}

	return 0;
}


static void mv_dp_flow_read(int dest, int flow_id)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(dest)) {
		MV_DP_CLI_FAIL("Illegal Cache Flow index: %d\n", MV_NSS_DP_INVALID_PARAM, dest);
		return;
	}

	/*the index is stored in xid*/
	res.cb = mv_dp_flow_read_cb;
	res.xid = dest;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed Status: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
				       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	rc = mv_nss_dp_flow_get(flow_id, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Read Flow_id:%d to Index: %d Status: %s\n", rc, flow_id, dest, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get Flow to Index: %d\n", dest);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Read MAC Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);

}

static void mv_dp_flow_set_flow_id(int i, int flow_id)
{
	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	flows[i].flow_id = flow_id;

	MV_DP_CLI_OK("Set Flow index: %d FlowID:%d\n", i, flows[i].flow_id);
}

static void mv_dp_flow_set_idle(int i, u32 timeout)
{
	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	flows[i].idle_timeout = timeout;

	MV_DP_CLI_OK("Set Flow index: %d Idle Timeout:%d\n", i, flows[i].idle_timeout);
}


static void mv_dp_flow_set_status(int i, int status)
{
	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}
	if (!(flow_status_active == status || flow_status_inactive == status)) {
		MV_DP_CLI_FAIL("Illegal Flow status: %d\n", MV_NSS_DP_INVALID_PARAM, status);
		return;
	}

	flows[i].status = status;

	MV_DP_CLI_OK("Set Flow index: %d Flow status:%d\n", i, flows[i].status);
}


static void mv_dp_flow_get(int flow_id)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_flow_get_cb;
	res.xid = flow_id;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed Status: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
				       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	rc = mv_nss_dp_flow_get(flow_id, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Flow_id: %d Status: %s\n", rc, flow_id, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get Flow:%d\n", flow_id);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Flow:%d Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       flow_id, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);
}

static void mv_dp_flow_set_fw_status(int flow_id, int status)
{
	mv_nss_dp_result_spec_t	res;
	mv_nss_dp_flow_status_t	fs;
	mv_nss_dp_status_t	rc;
	struct completion	*compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_flow_set_flow_status_cb;
	res.xid = flow_id;

	if (!(flow_status_active == status || flow_status_inactive == status)) {
		MV_DP_CLI_FAIL("Illegal Flow status: %d\n", MV_NSS_DP_INVALID_PARAM, status);
		return;
	}


	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed Status: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
				       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	fs.flow_id = flow_id;
	fs.status = status;
	rc = mv_nss_dp_flow_status_set(&fs, 1, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Set Flow_id:%d Status: %s\n", rc, flow_id, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Set Flow status\n");
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Set Flow_id:%d Status:%s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       flow_id, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);
}

static void mv_dp_flow_fw_status_set_multiple(void)
{
	mv_nss_dp_result_spec_t	res;
	mv_nss_dp_status_t	rc;
	struct completion	*compl_ptr;
	mv_nss_dp_flow_status_t *fs = NULL;
	int			i;

	/*the index is stored in xid*/
	res.cb = mv_dp_flow_set_flow_status_cb;
	res.xid = -1;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed Status: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
				       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	/*delete all cached*/

	if (allocated <= 0) {
		MV_DP_CLI_FAIL("No Flows Allocated\n", MV_NSS_DP_INVALID_PARAM);
		goto err_free;
	}

	fs = kzalloc(allocated * sizeof(mv_nss_dp_flow_status_t), GFP_KERNEL);
	if (!fs) {
		MV_DP_CLI_FAIL("Flow status allocation Failed: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
			       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
		goto err_free;
	}

	for (i = 0; i < allocated; i++) {
		fs[i].flow_id = flows[i].flow_id;
		fs[i].status = flows[i].status;
	}

	rc = mv_nss_dp_flow_status_set(fs, allocated, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Flow Status Set Count:%d Status:%s\n", rc, allocated, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Flow Status Set Count:%d\n", allocated);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Flow Status Set Count:%d Status:%s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       allocated, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);
	if (fs)
		kfree(fs);
}



static void mv_dp_flow_get_fw_status(int flow_id)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_flow_get_flow_status_cb;
	res.xid = flow_id;


	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed Status: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
				       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	rc = mv_nss_dp_flow_status_get(flow_id, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Flow_id:%d Status: %s\n", rc, flow_id, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get Flow status\n");
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Flow_id:%d Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       flow_id, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);

}


static void flow_present_bit_help(void)
{
	int i;
	MV_DP_CLI_OK("FLOW PRESENT BITS:\n");
	for (i = 0; i < MV_DP_SYS_FLOW_FIELD_LAST; i++)
		MV_DP_CLI_CONT("%12s: %d\n", flow_field_names[i], i);
	MV_DP_CLI_OK("\n");
}

static void flow_present_bit_show(u32 val)
{
	int i;
	u32 mask = 1l;

	MV_DP_CLI_CONT("|PRESENT :  0x%08X|\n", val);
	MV_DP_CLI_CONT("|PTD|PTS|L2D|L2S|ETH|LLS|LLD|OUI|IPD|IPS|L4D|L4S|L4P|LBL|PRA| OD|PRV|\n|");

	for (i = 0; i < MV_DP_SYS_FLOW_FIELD_LAST; i++)
		MV_DP_CLI_CONT("%3s|", (val & (mask << i)) ? "SET" : "RST");
	MV_DP_CLI_CONT("\n");

}



static void mv_dp_flow_get_count(void)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_flow_get_count_cb;
	res.xid = 0;


	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed Status: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
				       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	rc = mv_nss_dp_flow_count_get(&res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Flow Count Status: %s\n", rc, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get Flow Count\n");
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Flow Count Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}
err_free:
	if (res.cookie)
		kfree(res.cookie);

}



static void mv_dp_flow_get_stats(int flow_id)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_flow_get_flow_stats_cb;
	res.xid = flow_id;


	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed Status: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
				       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	rc = mv_nss_dp_flow_stats_get(flow_id, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Flow_id:%d Stats Status: %s\n", rc, flow_id, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get Flow status\n");
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Flow_id:%d stats Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       flow_id, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);

}

static void mv_dp_flow_stats_bulk_get(int i, int cnt, u32 options)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;
	mv_nss_dp_flow_stats_t *stats_ptr;


	stats_ptr = kzalloc(sizeof(mv_nss_dp_flow_stats_t)*cnt, GFP_KERNEL);
	if (!stats_ptr) {
		MV_DP_CLI_FAIL("Stats buffer alloc failed for %d entries\n", MV_NSS_DP_OUT_OF_RESOURCES, cnt);
		return;
	}

	/*the index is stored in xid*/
	res.cb = mv_dp_flow_stats_bulk_get_cb;
	res.xid = i;

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

	if (MV_NSS_DP_OK != mv_nss_dp_flow_bulk_stats_get(i, cnt, options, stats_ptr, &res)) {
		MV_DP_CLI_FAIL("Get Bulk Statistics for %d entries\n", MV_NSS_DP_FAILED, cnt);
		if (res.cookie)
			kfree(res.cookie);
		kfree(stats_ptr);
		return;
	}

	MV_DP_CLI_TMSG("Get Bulk Statistics for %d entries from index: %d\n", cnt, i);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Bulk Flow Statistics Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}




static void mv_dp_flow_reset_stats(int flow_id)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_flow_reset_flow_stats_cb;
	res.xid = flow_id;


	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed Status:%s\n", MV_NSS_DP_OUT_OF_RESOURCES,
				       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	rc = mv_nss_dp_flow_stats_reset(flow_id, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Reset Flow_id:%d Stats Status:%s\n", rc, flow_id, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get Flow status\n");
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Reset Flow_id:%d stats Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       flow_id, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);
}



static void mv_dp_flow_delete_all(void)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_flow_delete_all_cb;
	res.xid = 0;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed Status:%s\n", MV_NSS_DP_OUT_OF_RESOURCES,
				       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}


	rc = mv_nss_dp_flow_delete_all(&res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Flow Delete ALL Status: %s\n", rc, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Flow Delete ALL\n");

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Flow Delete All Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}
err_free:
	if (res.cookie)
		kfree(res.cookie);

}

static void mv_dp_flow_delete_multiple(void)
{
	mv_nss_dp_result_spec_t	res;
	mv_nss_dp_status_t	rc;
	struct completion	*compl_ptr;
	mv_nss_dp_flow_id_t	*flow_ids = NULL;
	int			i;

	/*the index is stored in xid*/
	res.cb = mv_dp_flow_delete_cb;
	res.xid = -1;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
				       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	/*delete all cached*/

	if (allocated <= 0) {
		MV_DP_CLI_FAIL("No Flows Allocated\n", MV_NSS_DP_INVALID_PARAM);
		goto err_free;
	}

	flow_ids = kzalloc(allocated * sizeof(mv_nss_dp_flow_id_t), GFP_KERNEL);
	if (!flow_ids) {
		MV_DP_CLI_FAIL("Flow_ids allocation Failed: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
			       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
		goto err_free;
	}

	for (i = 0; i < allocated; i++)
		flow_ids[i] = flows[i].flow_id;

	rc = mv_nss_dp_flow_delete(flow_ids, allocated, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Flow Delete Count:%d Status:%s\n", rc, allocated, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Delete Flow Count:%d\n", allocated);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Flow Delete Count:%d Status:%s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       allocated, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);
	if (flow_ids)
		kfree(flow_ids);
}

static void mv_dp_flow_delete(int flow_id)
{
	mv_nss_dp_result_spec_t	res;
	mv_nss_dp_status_t	rc;
	struct completion	*compl_ptr;


	/*the index is stored in xid*/
	res.cb = mv_dp_flow_delete_cb;
	res.xid = flow_id;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
				       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	rc = mv_nss_dp_flow_delete(&flow_id, 1, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Flow Delete:%d Status:%s\n", rc, flow_id, mv_dp_err_name_get(rc));
		goto err;
	}

	MV_DP_CLI_TMSG("Delete Flow_id:%d\n", flow_id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Flow_id:%d Delete Status:%s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       flow_id, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}
err:
	if (res.cookie)
		kfree(res.cookie);
}


static void mv_dp_flow_get_cb(mv_nss_dp_event_t *event)
{
	int i;
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (!(MV_NSS_DP_OK == event->status || MV_NSS_DP_END_OF_LIST == event->status)) {
		MV_DP_CLI_FAIL_CB("Get Flow Status: %s\n",
				  event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.flow) {
		MV_DP_CLI_FAIL_CB("Get Flow: Empty Flow param, Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	/*loop on count*/
	MV_DP_CLI_OK_CB("|FLOWS Get Count: %d\n", event, event->count);
	for (i = 0; i < event->count; i++)
		mv_dp_flow_show_single(&event->params.flow[i]);

	MV_DP_CLI_CONT("|FLOWS GET END|<\n");

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_flow_get_flow_stats_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get Flow Stats Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.flow_stats) {
		MV_DP_CLI_FAIL_CB("Get Flow Stats Empty flow stats param, Status: %s\n",
				  event, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("|GOT FLOW_ID STATS: %u|<\n", event, event->params.flow_stats->flow_id);
	MV_DP_CLI_CONT("|FLOW  ID|LAST PKT  TS|PKTS    RX|OCTETS  RX|<\n");
	mv_dp_flow_show_stats_single(event->params.flow_stats);
	MV_DP_CLI_CONT("|Flow Stats END|<\n");

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

void mv_dp_flow_stats_bulk_get_cb(mv_nss_dp_event_t *event)
{
	mv_nss_dp_flow_stats_t		*stats_ptr;
	int				i;


	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	stats_ptr = event->params.flow_stats;

	if (!stats_ptr) {
		MV_DP_CLI_FAIL_CB("null stats buff ptr\n", event);
		goto err;
	}

	if ((MV_NSS_DP_OK != event->status) && (MV_NSS_DP_END_OF_LIST != event->status)) {

		MV_DP_CLI_FAIL_CB("GetBulk Statistics for %d entries Status: %s\n",
				  event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("|GOT BULK STATS COUNT: %u|<\n", event, event->count);
	MV_DP_CLI_CONT("|IDX|FLOW  ID|LAST PKT  TS|PKTS    RX|OCTETS  RX|<\n");
	for (i = 0; i < event->count; i++) {
		MV_DP_CLI_CONT("|%3d", i);
		mv_dp_flow_show_stats_single(stats_ptr + i);
	}
	MV_DP_CLI_CONT("|Flow Stats END|<\n");

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);
	if (stats_ptr)
		kfree(stats_ptr);

}


static void mv_dp_flow_reset_flow_stats_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Reset Flow Stats Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("|FLOW_ID %d STATS RESET|<\n", event, event->xid);

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}




static void mv_dp_flow_get_flow_status_cb(mv_nss_dp_event_t *event)
{
	int i;
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get Flow Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.flow_status) {
		MV_DP_CLI_FAIL_CB("Get Flow Status: Empty flow status param, Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}


	/*TO do: loop on count*/
	MV_DP_CLI_OK_CB("|Get Flow Status| count: %d|<\n", event, event->count);
	MV_DP_CLI_CONT("| N  |Flow    ID|STATUS|<\n");

	for (i = 0; i < event->count; i++) {
		MV_DP_CLI_CONT("|%4d|%10d|%6d|<\n", i,
				event->params.flow_status->flow_id,
				event->params.flow_status->status);
	}
	MV_DP_CLI_CONT("|Flow Status END|<\n");

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_flow_get_count_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get Flow Count Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.flow_count) {
		MV_DP_CLI_FAIL_CB("Get Flow Count: Empty flow count param, Status: %s\n",
				  event, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("Flow Count:%d\n", event, *(event->params.flow_count));

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}



static void mv_dp_flow_set_flow_status_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (!(MV_NSS_DP_OK == event->status || MV_NSS_DP_END_OF_LIST == event->status)) {
		MV_DP_CLI_FAIL_CB("Get Flow Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Flow status set. First Flow_id:%d Count:%d\n", event, event->xid, event->count);

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}


static void mv_dp_flow_read_cb(mv_nss_dp_event_t *event)
{
	/*save response in the client buffer and return OK;*/
	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Read Flow Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.flow) {
		MV_DP_CLI_FAIL_CB("Read Flow: Empty Flow struct\n", event);
		goto err;
	}


	if (!MV_DP_SYSFS_FLOWS_NUM_OK(event->xid)) {
		MV_DP_CLI_FAIL_CB("Illegal Cache Flow index: %d\n", event, event->xid);
		goto err;
	}

	memcpy(&flows[event->xid], event->params.flow, sizeof(mv_nss_dp_flow_t));

	MV_DP_CLI_OK_CB("Read Flow_id:%d to Index: %d\n", event, event->params.flow->flow_id, event->xid);

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_flow_delete_cb(mv_nss_dp_event_t *event)
{
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (!(MV_NSS_DP_OK == event->status || MV_NSS_DP_END_OF_LIST == event->status)) {
		MV_DP_CLI_FAIL_CB("Delete Flow Status: %s\n",
				  event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Deleted First Flow_id:%d count:%d\n", event, event->xid, event->count);
err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_flow_delete_all_cb(mv_nss_dp_event_t *event)
{
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Flow Delete ALL count:%d Status:%s\n",
				  event, event->count, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("Flow Delete All deleted:%d\n", event, event->count);
err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}


static void mv_dp_flow_set_cb(mv_nss_dp_event_t *event)
{
	int i;
	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (!event->params.flow) {
		MV_DP_CLI_FAIL_CB("Get Flow: Empty Flow param, Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!(MV_NSS_DP_OK == event->status || MV_NSS_DP_END_OF_LIST == event->status)) {
		MV_DP_CLI_FAIL_CB("Set Index:%d Status: %s\n",
				  event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	/*the flows ptr is passed in and is populated inside*/

	for (i = 0; i < event->count; i++)
		MV_DP_CLI_OK_CB("Set Index:%d count:%d Flow_id:%d\n",
				event, event->xid, event->count, event->params.flow->flow_id);

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);

}


static void mv_dp_flow_commit(int cache_ind)
{
	mv_nss_dp_result_spec_t	res;
	int			tmp_count;
	mv_nss_dp_flow_t	*ptr_flows;
	struct completion	*compl_ptr;
	mv_nss_dp_status_t	rc;


	if (cache_ind == -1) {
		/*commit all*/
		tmp_count = allocated;
		ptr_flows = flows;
		res.xid = 0;
	} else if (!MV_DP_SYSFS_FLOWS_NUM_OK(cache_ind)) {
		MV_DP_CLI_FAIL("Illegal Flow index: %d\n", MV_NSS_DP_INVALID_PARAM, cache_ind);
		return;
	} else	{
		tmp_count = 1;
		res.xid = cache_ind;
		ptr_flows = &flows[cache_ind];
	}

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed Status: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
				       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
			return;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	res.cb = mv_dp_flow_set_cb;

	rc = mv_nss_dp_flow_set(ptr_flows, tmp_count, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Commit Flow Index: %d Status: %s\n", rc, cache_ind, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Commit Flow Index:%d, Count:%d\n", cache_ind, tmp_count);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Flow Commit: %d Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       res.xid, mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);

}


static void mv_dp_flow_release(void)
{

	if (!allocated) {
		MV_DP_CLI_FAIL("Not Allocated\n", MV_NSS_DP_FAILED);
		return;
	}

	kfree(flows);
	MV_DP_CLI_OK("Flows Cache Released\n");
	allocated = 0;

}

static void mv_dp_flow_clear(int cache_ind)
{
	if (!allocated) {
		MV_DP_CLI_FAIL("Not allocated\n", MV_NSS_DP_FAILED);
		return;
	}

	if (cache_ind == -1) {
		memset(flows, 0, sizeof(mv_nss_dp_flow_t) * allocated);
		MV_DP_CLI_OK("Flows Cache Cleared: %d\n", allocated);
	} else if (!MV_DP_SYSFS_FLOWS_NUM_OK(cache_ind)) {
		MV_DP_CLI_FAIL("Illegal Cache Flow index: %d\n", MV_NSS_DP_INVALID_PARAM, cache_ind);
		return;
	} else	{
		memset(&flows[cache_ind], 0, sizeof(mv_nss_dp_flow_t));
		MV_DP_CLI_OK("Flow Cache %d Cleared\n", cache_ind);
	}
}


static void mv_dp_flow_alloc(int num)
{
	if (allocated) {
		MV_DP_CLI_FAIL("Already allocated: %d\n", MV_NSS_DP_FAILED, allocated);
		return;
	}
	if (num < 0 || num > MV_DP_SYSFS_FLOWS_MAX) {
		MV_DP_CLI_FAIL("Illegal Flow Records number: %d\n", MV_NSS_DP_INVALID_PARAM, num);
		return;
	}

	flows = kzalloc(sizeof(mv_nss_dp_flow_t) * num, GFP_KERNEL);
	if (!flows) {
		MV_DP_CLI_FAIL("Cache alloc failed for Flows number:%d\n", MV_NSS_DP_OUT_OF_RESOURCES, num);
		return;
	}

	allocated = num;
	MV_DP_CLI_OK("Allocated:%d\n", allocated);
}


static void mv_dp_flow_show(int index)
{
	int i;

	if (index == -1) {
		/*show all*/
		MV_DP_CLI_OK("|FLOWS CACHE|TOTAL:%d|<\n", allocated);
		for (i = 0; i < allocated; i++) {
			MV_DP_CLI_CONT("|%5d", i);
			mv_dp_flow_show_single(&flows[i]);
		}

		MV_DP_CLI_CONT("|FLOWS CAHCE END|<\n");
		return;
	} else if (!MV_DP_SYSFS_FLOWS_NUM_OK(index)) {
			MV_DP_CLI_FAIL("Illegal Flow Records number:%d", MV_NSS_DP_INVALID_PARAM, index);
			return;
	}

	/*show single*/

	MV_DP_CLI_OK("|FLOWS CACHE:%d|<\n", index);
	MV_DP_CLI_CONT("|%5d", index);
	mv_dp_flow_show_single(&flows[index]);
	MV_DP_CLI_CONT("|FLOWS CACHE END|<\n");
}

static void mv_dp_flow_show_single(mv_nss_dp_flow_t *flow)
{
	if (!flow) {
		MV_DP_CLI_FAIL("Null flow\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("|FLOWID:%u|STATUS:%7s|IDLE:%d|\n",
		       flow->flow_id,
		       flow->status ? "ACT" : "NOT ACT",
		       flow->idle_timeout);
	MV_DP_CLI_CONT("|CLS:.......................................|\n");
	mv_dp_param_show_single(&flow->cls);
	MV_DP_CLI_CONT("|ACT:.......................................|\n");
	mv_dp_param_show_single(&flow->act);

}

static void mv_dp_param_show_single(mv_nss_dp_params_t *param)
{
	unsigned char	dst_ip[INET6_ADDRSTRLEN];
	unsigned char	src_ip[INET6_ADDRSTRLEN];

	if (!param) {
		MV_DP_CLI_FAIL("Null params\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	mv_dp_sysfs_ip_to_str(dst_ip, &(param->ip_addr_dst));
	mv_dp_sysfs_ip_to_str(src_ip, &(param->ip_addr_src));


	/*MV_DP_CLI_CONT("|PRESENT :0x%08X|\n", param->present);*/
	flow_present_bit_show(param->present);
	MV_DP_CLI_CONT("|PORT DST:%17d|\n", param->port_dst);
	MV_DP_CLI_CONT("|PORT SRC:%17d|\n", param->port_src);

	MV_DP_CLI_CONT("|DEST MAC:%02X%02X%02X%02X%02X%02X|\n",
		       param->l2_addr_dst.addr[0],
		       param->l2_addr_dst.addr[1],
		       param->l2_addr_dst.addr[2],
		       param->l2_addr_dst.addr[3],
		       param->l2_addr_dst.addr[4],
		       param->l2_addr_dst.addr[5]);
	MV_DP_CLI_CONT("|SRC  MAC:%02X%02X%02X%02X%02X%02X|\n",
		       param->l2_addr_src.addr[0],
		       param->l2_addr_src.addr[1],
		       param->l2_addr_src.addr[2],
		       param->l2_addr_src.addr[3],
		       param->l2_addr_src.addr[4],
		       param->l2_addr_src.addr[5]);
	MV_DP_CLI_CONT("|ETH TYPE:           0x%04X|\n", param->eth_type);
	MV_DP_CLI_CONT("|LLC SSAP:           0x%04X|\n", param->llc_ssap);
	MV_DP_CLI_CONT("|LLC DSAP:           0x%04X|\n", param->llc_dsap);
	MV_DP_CLI_CONT("|SNAP    :           %02X%02X%02X%02X%02X|\n",
		       param->snap[0], param->snap[1], param->snap[2], param->snap[3], param->snap[4]);
	MV_DP_CLI_CONT("|DEST  IP:%s|\n", dst_ip);
	MV_DP_CLI_CONT("|SRC   IP:%s|\n", src_ip);
	MV_DP_CLI_CONT("|FLW  LBL:       0x%08X|\n", param->flbl);
	MV_DP_CLI_CONT("|L4 PROTO:%17d(%s)|\n", param->l4_proto, l4_proto_names[param->l4_proto]);
	MV_DP_CLI_CONT("|L4 DST P:%17d|\n", param->l4_port_dst);
	MV_DP_CLI_CONT("|L4 SRC P:%17d|\n", param->l4_port_src);
	MV_DP_CLI_CONT("|OPAQUE  :       0x%08X|\n", param->opaque);
	MV_DP_CLI_CONT("|PRIO ACT:%17d(%s)|\n", param->prio_action, prio_act_names[param->prio_action]);
	MV_DP_CLI_CONT("|PRIO VAL:%17d|\n", param->prio_val);
}



static void mv_dp_flow_set_mac(int i,
				 enum mv_dp_sysfs_param_type p_type,
				 enum mv_dp_sysfs_dir_type m_type,
				 unsigned char *mac)
{

	mv_nss_dp_params_t *target_params;
	mv_nss_dp_l2_addr_t *target_mac;
	u32 mask = 0;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (!mac) {
		MV_DP_CLI_FAIL("Null MAC ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (param_type_cls == p_type) {
		target_params = &flows[i].cls;
	} else if (param_type_act == p_type) {
		target_params = &flows[i].act;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM Type: %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	if (dir_type_dst == m_type) {
		target_mac = &target_params->l2_addr_dst;
		mask = MV_NSS_DP_L2_ADDR_DST;
	} else if (dir_type_src == m_type) {
		target_mac = &target_params->l2_addr_src;
		mask = MV_NSS_DP_L2_ADDR_SRC;
	} else {
		MV_DP_CLI_FAIL("Illegal MAC PARAM Type: %d\n", MV_NSS_DP_INVALID_PARAM, m_type);
		return;
	}

	memcpy(target_mac->addr, mac, sizeof(mv_nss_dp_l2_addr_t));
	target_params->present |= mask;

	MV_DP_CLI_OK("Set ind:%d %s mac:%s to:%02X:%02X:%02X:%02X:%02X:%02X\n", i,
		     param_names[p_type], dir_names[m_type],
		     target_mac->addr[0],
		     target_mac->addr[1],
		     target_mac->addr[2],
		     target_mac->addr[3],
		     target_mac->addr[4],
		     target_mac->addr[5]);
}

static void mv_dp_flow_set_snap(int i,
				 enum mv_dp_sysfs_param_type p_type,
				 unsigned char *snap)
{

	mv_nss_dp_params_t *target_params;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_cls == p_type) {
		target_params = &flows[i].cls;
	} else if (param_type_act == p_type) {
		target_params = &flows[i].act;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM Type: %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}


	memcpy(&target_params->snap, snap, sizeof(target_params->snap));
	target_params->present |= MV_NSS_DP_SNAP;

	MV_DP_CLI_OK("Set Flow ind:%d %s  oui to:%02X:%02X:%02X:%02X:%02X\n", i, param_names[p_type],
		     target_params->snap[0],
		     target_params->snap[1],
		     target_params->snap[2],
		     target_params->snap[3],
		     target_params->snap[4]);
}


static void mv_dp_flow_set_ip(int i,
				 enum mv_dp_sysfs_param_type p_type,
				 enum mv_dp_sysfs_dir_type ip_type,
				 int af, unsigned char *ip)
{

	mv_nss_dp_params_t *target_params;
	mv_nss_dp_ip_addr_t *target_ip;
	const char	*end;
	unsigned char	ip_str[INET6_ADDRSTRLEN];
	u32 mask = 0;


	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (!ip) {
		MV_DP_CLI_FAIL("Null IP ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (param_type_cls == p_type) {
		target_params = &flows[i].cls;
	} else if (param_type_act == p_type) {
		target_params = &flows[i].act;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM Type: %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}


	if (dir_type_dst == ip_type) {
		target_ip = &target_params->ip_addr_dst;
		mask = MV_NSS_DP_IP_DST;
	} else if (dir_type_src == ip_type) {
		target_ip = &target_params->ip_addr_src;
		mask = MV_NSS_DP_IP_SRC;
	} else {
		MV_DP_CLI_FAIL("Illegal IP PARAM Type: %d\n", MV_NSS_DP_INVALID_PARAM, ip_type);
		return;
	}


	if (af == 4) {
		if (in4_pton(ip, -1, (u8 *)&target_ip->ip, -1, &end)) {
			target_ip->ver = 4;
			mv_dp_sysfs_ip_to_str(ip_str, target_ip);
			MV_DP_CLI_OK("Set Flow Index %d %s  to %s IPv4: %s\n", i,
				     param_names[p_type], dir_names[ip_type], ip_str);
		} else {
			MV_DP_CLI_FAIL("Invalid IPv4 format: %s\n", MV_NSS_DP_INVALID_PARAM, ip);
		}
	} else if (af == 6) {
		if (in6_pton(ip, -1, (u8 *)&target_ip->ip, -1, &end)) {
			target_ip->ver = 6;
			mv_dp_sysfs_ip_to_str(ip_str, target_ip);
			MV_DP_CLI_OK("Set Flow Index %d %s to %s IPv6: %s\n", i,
				     param_names[p_type], dir_names[ip_type], ip_str);
		} else {
			MV_DP_CLI_FAIL("Invalid IPv6 format: %s\n", MV_NSS_DP_INVALID_PARAM, ip);
		}
	} else {
		MV_DP_CLI_FAIL("Invalid AF %d - shold be 4 or 6\n", MV_NSS_DP_INVALID_PARAM, af);
		return;
	}

	target_params->present |= mask;

}

static void mv_dp_flow_set_od(int i, enum mv_dp_sysfs_param_type p_type, u32 od)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	target->opaque = od;
	target->present |= MV_NSS_DP_OPAQUE;

	MV_DP_CLI_OK("Set Flow index:%d %s OD=0x%08X\n", i, param_names[p_type], target->opaque);
}

static void mv_dp_flow_set_present(int i, enum mv_dp_sysfs_param_type p_type, u32 present)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	target->present = present;


	MV_DP_CLI_OK("Set Flow index:%d %s present=0x%08X\n", i, param_names[p_type], target->present);
}

static void mv_dp_flow_rst_present(int i, enum mv_dp_sysfs_param_type p_type, int b)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (!MV_DP_SYS_FLOW_FIELD_IS_OK(b)) {
		MV_DP_CLI_FAIL("Illegal Flow bit: %d\n", MV_NSS_DP_INVALID_PARAM, b);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	target->present &= ~(1l << b);

	MV_DP_CLI_OK("Flow index:%d %s present=0x%08X\n", i, param_names[p_type], target->present);
}


static void mv_dp_flow_set_flbl(int i, enum mv_dp_sysfs_param_type p_type, u32 lbl)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	target->flbl = lbl;
	target->present |= MV_NSS_DP_FLBL;


	MV_DP_CLI_OK("Set Flow index:%d %s flow lablel=0x%08X\n", i, param_names[p_type], target->flbl);
}

static void mv_dp_flow_set_etht(int i, enum mv_dp_sysfs_param_type p_type, u16 etht)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	target->eth_type = etht;
	target->present |= MV_NSS_DP_ETH_TYPE;

	MV_DP_CLI_OK("Set Flow index:%d %s ETH Type=0x%04X\n", i, param_names[p_type], target->eth_type);
}

static void mv_dp_flow_set_llc_ssap(int i, enum mv_dp_sysfs_param_type p_type, u16 snap)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	target->llc_ssap = snap;
	target->present |= MV_NSS_DP_LLC_SSAP;


	MV_DP_CLI_OK("Set Flow index:%d %s LLC SSAP=0x%04X\n", i, param_names[p_type], target->llc_ssap);
}

static void mv_dp_flow_set_llc_dsap(int i, enum mv_dp_sysfs_param_type p_type, u16 snap)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	target->llc_dsap = snap;
	target->present |= MV_NSS_DP_LLC_DSAP;

	MV_DP_CLI_OK("Set Flow index:%d %s LLC DSAP=0x%04X\n", i, param_names[p_type], target->llc_ssap);
}

static void mv_dp_flow_set_l4_dst_prt(int i, enum mv_dp_sysfs_param_type p_type, u16 prt)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	target->l4_port_dst = prt;
	target->present |= MV_NSS_DP_L4_PORT_DST;

	MV_DP_CLI_OK("Set Flow index:%d %s L4 DST PORT = %5d\n", i, param_names[p_type], target->l4_port_dst);
}

static void mv_dp_flow_set_l4_src_prt(int i, enum mv_dp_sysfs_param_type p_type, u16 prt)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	target->l4_port_src = prt;
	target->present |= MV_NSS_DP_L4_PORT_SRC;

	MV_DP_CLI_OK("Set Flow index:%d %s L4 SRC PORT = %5d\n", i, param_names[p_type], target->l4_port_src);
}

static void mv_dp_flow_set_src_prt(int i, enum mv_dp_sysfs_param_type p_type, u8 prt)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	target->port_src = prt;
	target->present |= MV_NSS_DP_PORT_SRC;

	MV_DP_CLI_OK("Set Flow index:%d %s SRC PORT = %5d\n", i, param_names[p_type], target->port_src);
}

static void mv_dp_flow_set_dst_prt(int i, enum mv_dp_sysfs_param_type p_type, u8 prt)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	target->port_dst = prt;
	target->present |= MV_NSS_DP_PORT_DST;

	MV_DP_CLI_OK("Set Flow index:%d %s DST PORT = %5d\n", i, param_names[p_type], target->port_dst);
}


static void mv_dp_flow_set_l4_proto(int i, enum mv_dp_sysfs_param_type p_type, u8 proto)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	if (!MV_DP_SYS_L4_PROTO_IS_OK(proto)) {
		MV_DP_CLI_FAIL("Illegal PROTO TYPE %d\n", MV_NSS_DP_INVALID_PARAM, proto);
		return;
	}

	target->l4_proto = proto;
	target->present |= MV_NSS_DP_L4_PROTO;

	MV_DP_CLI_OK("Set Flow index:%d %s L4 PROTOCOL = %5d(%s)\n", i, param_names[p_type],
		     target->l4_proto, l4_proto_names[target->l4_proto]);
}

static void mv_dp_flow_set_prio_act(int i, enum mv_dp_sysfs_param_type p_type, u8 act)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	if (!MV_DP_SYS_PRI_ACT_IS_OK(act)) {
		MV_DP_CLI_FAIL("Illegal PRIORITY ACT %d\n", MV_NSS_DP_INVALID_PARAM, act);
		return;
	}

	target->prio_action = act;
	target->present |= MV_NSS_DP_PRIO_ACTION;

	MV_DP_CLI_OK("Set Flow index:%d %s PRIOTITY ACTION = %5d(%s)\n", i, param_names[p_type],
		     target->prio_action, prio_act_names[target->prio_action]);
}

static void mv_dp_flow_set_prio_val(int i, enum mv_dp_sysfs_param_type p_type, u8 val)
{
	mv_nss_dp_params_t *target;

	if (!MV_DP_SYSFS_FLOWS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache FLOW index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (param_type_act == p_type) {
		target = &flows[i].act;
	} else if (param_type_cls == p_type) {
		target = &flows[i].cls;
	} else {
		MV_DP_CLI_FAIL("Illegal PARAM TYPE %d\n", MV_NSS_DP_INVALID_PARAM, p_type);
		return;
	}

	target->prio_val = val;
	target->present |= MV_NSS_DP_PRIO;

	MV_DP_CLI_OK("Set Flow index:%d %s PRIOTITY VALUE = %5d\n", i, param_names[p_type], target->prio_val);
}


static void mv_dp_flow_show_stats_single(mv_nss_dp_flow_stats_t *stats)
{

	if (!stats) {
		MV_DP_CLI_FAIL("Null Flow Statistics", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("|%8u|%12u|%10llu|%10llu|<\n",
		       (u32)stats->flow_id,
		       (u32)stats->rx_last_ts,
		       stats->rx_pkts,
		       stats->rx_octets
		       );

}

