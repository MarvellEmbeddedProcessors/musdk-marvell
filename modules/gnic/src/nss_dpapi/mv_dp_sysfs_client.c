
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
#include <linux/slab.h>


#include "mv_dp_defs.h"
#include "mv_dp_sysfs.h"
#include "mv_nss_dp.h"
#include "mv_dp_main.h"
#include "mv_dp_int_if.h"



#define MV_DP_SYSFS_CLIENTS_MAX		(2*1024)
#define MV_DP_SYSFS_CLIENTS_NUM_OK(n)	((n) >= 0 && (n) < allocated)


static  mv_nss_dp_client_t	*clients;
static  int			allocated; /*number of allocated entries*/

static void mv_dp_client_get(int i);
static void mv_dp_client_read(int d, int s);
static void mv_dp_client_read_mac(int d, unsigned char *mac);
static void mv_dp_client_get_mac(unsigned char *mac);
static void mv_dp_client_commit(int i);
static void mv_dp_client_show(int i);
static void mv_dp_client_show_single(mv_nss_dp_client_t *client);
static void mv_dp_client_alloc(int i);
static void mv_dp_client_release(void);
static void mv_dp_client_clear(int i);
static void mv_dp_client_delete(int id);

static void mv_dp_client_set_mac(int i, unsigned char *mac);
static void mv_dp_client_set_ck(int i, u32 ck);
static void mv_dp_client_set_policy(int i, int policy);
static void mv_dp_client_set_vlan(int i, int vlan);
static void mv_dp_client_set_dscp(int i, int dscp);
static void mv_dp_client_set_qcf(int i, int qcf);
static void mv_dp_client_set_bssid(int i, int bssid);
static void mv_dp_client_set_radioid(int i, int rad_id);
static void mv_dp_client_set_my_mac(int i, int my);
static void mv_dp_client_set_port_type(int i, int type);
static void mv_dp_client_set_port_id_idx(int i, int id);
static void mv_dp_client_set_client_id(int i, int id);

static void mv_dp_client_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_client_read_cb(mv_nss_dp_event_t *event);
static void mv_dp_client_delete_cb(mv_nss_dp_event_t *event);
static void mv_dp_client_set_cb(mv_nss_dp_event_t *event);



static ssize_t mv_dp_client_sysfs_help(char *b)
{
	int o = 0;

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  help                - Show Help\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  client_release      - Release Clients Cache\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]           > client_clear        - Clear Clients Cache [i] to 0 (no release)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [MAC]     > client_read_mac     - Read Client Record by [MAC]\n"
		       "                                           from FW to cache index [i]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [ID]      > client_read         - Read Client Record [ID]\n"
		       "                                           to cache index [i]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]          > client_get          - Print only Client Record by their\n"
		       "                                           [ID] (not saved to cache)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [MAC]         > client_get_mac      - Print only Client Record by their\n"
		       "                                           MAC (not saved to cache)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]           > client_commit       - Save Client stored at index [i]\n"
		       "                                           to FW, -1 for all\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [n]           > client_alloc        - Allocate Client cache of [n] entries\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]           > client_show         - Show Client cache entry [i] or -1 for ALL\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]          > client_delete       - Delete Client Record from FW by its [MAC]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [ID]      > client_id_set       - Set Cached Client[i]'s [ID]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [MAC]     > client_mac_set      - Set Cached Client[i]'s [MAC]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [ck]      > client_ck_set       - Set Cached Client[i]'s cookie data [ck]\n"
		       "                                           (hex 0x0 16 bit)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [p]       > client_policy_set   - Set Cached Client[i]'s priority\n"
		       "                                           policy[p] (dec 8 bit)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [v]       > client_vlan_set     - Set Cached Client[i]'s vlan[v]\n"
		       "                                           (dec 12 bit)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [m]       > client_my_mac_set   - Set Cached Client[i]'s is mymac[m] (0:1)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [q]       > client_qcf_set      - Set Cached Client[i]'s\n"
		       "                                           QCF needed[q] (0:1)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [q]       > client_dscp_set     - Set Cached Client[i]'s\n"
		       "                                           DSCP needed[q] (0:1)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [b]       > client_bssid_set    - Set Cached Client[i]'s BSSID[b]\n"
		       "                                           (dec 4 bit)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [r]       > client_radioid_set  - Set Cached Client[i]'s RadioID[r]\n"
		       "                                           (dec 2 bit)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [T]       > client_port_type_set   - Set Cached Client[i]'s port type[T]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [ID]      > client_port_id_idx_set - Set Cached Client[i]'s\n"
		       "                                              port id/index[ID] according to type\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [MAC]= MAC address as 00:00:00:00:00:00");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   dec - decimal 123; hex - hexadecimal (0x000)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "     [T]   = Virtual port type: 1-ETH, 4-CAPWAP, 5-VXLAN\n");
	return o;
}

static ssize_t mv_dp_client_sysfs_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int off = 0;
	const char  *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "help")) {
		return mv_dp_client_sysfs_help(buf);
	} else if (!strcmp(name, "client_release")) {
			mv_dp_client_release();
	} else {
		MV_DP_CLI_FAIL("Parse Error CMD: <%s>\n", MV_NSS_DP_INVALID_PARAM, name);
		off = mv_dp_client_sysfs_help(buf);
	}
	return off;
}

static ssize_t mv_dp_client_sysfs_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t len)
{
	const char *name = attr->attr.name;
	unsigned int    a, b, c;
	unsigned char mac[6];


	int err;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	err = a = b = c = 0;

	if (!strcmp(name, "client_read_mac")) {
		if (7 != sscanf(buf, "%d %hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		       &a, &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
			goto err;
		mv_dp_client_read_mac(a, mac);
	} else	if (!strcmp(name, "client_read")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_client_read(a, b);
	} else	if (!strcmp(name, "client_get")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_client_get(a);
	} else if (!strcmp(name, "client_commit")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_client_commit(a);
	} else if (!strcmp(name, "client_clear")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_client_clear(a);
	} else if (!strcmp(name, "client_show")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_client_show(a);
	} else if (!strcmp(name, "client_alloc")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_client_alloc(a);
	} else if (!strcmp(name, "client_delete")) {
		if (sscanf(buf, " %d", &a) != 1)
			goto err;
		mv_dp_client_delete(a);
	} else if (!strcmp(name, "client_get_mac")) {
		if (6 != sscanf(buf, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		       &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
			goto err;
		mv_dp_client_get_mac(mac);
	} else if (!strcmp(name, "client_mac_set")) {
		if (7 != sscanf(buf, "%d %hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		       &a, &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
			goto err;
		mv_dp_client_set_mac(a, mac);
	} else if (!strcmp(name, "client_ck_set")) {
		if (2 != sscanf(buf, "%d 0x%8X", &a, &b))
			goto err;
		mv_dp_client_set_ck(a, b);
	} else if (!strcmp(name, "client_policy_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_client_set_policy(a, b);
	} else if (!strcmp(name, "client_vlan_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_client_set_vlan(a, b);
	} else if (!strcmp(name, "client_dscp_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_client_set_dscp(a, b);
	} else if (!strcmp(name, "client_my_mac_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_client_set_my_mac(a, b);
	} else if (!strcmp(name, "client_qcf_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_client_set_qcf(a, b);
	} else if (!strcmp(name, "client_radio_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_client_set_radioid(a, b);
	} else if (!strcmp(name, "client_bssid_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_client_set_bssid(a, b);
	} else if (!strcmp(name, "client_port_type_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_client_set_port_type(a, b);
	} else if (!strcmp(name, "client_port_id_idx_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_client_set_port_id_idx(a, b);
	} else if (!strcmp(name, "client_id_set")) {
		if (sscanf(buf, "%d %d", &a, &b) != 2)
			goto err;
		mv_dp_client_set_client_id(a, b);
	} else {
		goto err;
	}

	return len;
err:
	MV_DP_LOG_INF("Parse Error CMD:<%s>\n", attr->attr.name);
	return -EINVAL;
}

static DEVICE_ATTR(help,			S_IRUSR, mv_dp_client_sysfs_show, NULL);
static DEVICE_ATTR(client_release,		S_IRUSR, mv_dp_client_sysfs_show, NULL);
static DEVICE_ATTR(client_delete_multi,		S_IRUSR, mv_dp_client_sysfs_show, NULL);
static DEVICE_ATTR(client_clear,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_read,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_read_mac,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_commit,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_show,			S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_get,			S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_get_mac,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_alloc,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_delete,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_mac_set,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_ck_set,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_vlan_set,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_policy_set,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_dscp_set,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_bridge_set,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_my_mac_set,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_qcf_set,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_radio_set,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_bssid_set,		S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_port_type_set,	S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_port_id_idx_set,	S_IWUSR, NULL, mv_dp_client_sysfs_store);
static DEVICE_ATTR(client_id_set,	S_IWUSR, NULL, mv_dp_client_sysfs_store);

static struct attribute *mv_dp_client_sysfs_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_client_clear.attr,
	&dev_attr_client_delete_multi.attr,
	&dev_attr_client_release.attr,
	&dev_attr_client_read.attr,
	&dev_attr_client_read_mac.attr,
	&dev_attr_client_commit.attr,
	&dev_attr_client_show.attr,
	&dev_attr_client_get.attr,
	&dev_attr_client_get_mac.attr,
	&dev_attr_client_alloc.attr,
	&dev_attr_client_delete.attr,
	&dev_attr_client_mac_set.attr,
	&dev_attr_client_ck_set.attr,
	&dev_attr_client_policy_set.attr,
	&dev_attr_client_vlan_set.attr,
	&dev_attr_client_qcf_set.attr,
	&dev_attr_client_radio_set.attr,
	&dev_attr_client_bssid_set.attr,
	&dev_attr_client_dscp_set.attr,
	&dev_attr_client_bridge_set.attr,
	&dev_attr_client_my_mac_set.attr,
	&dev_attr_client_port_type_set.attr,
	&dev_attr_client_port_id_idx_set.attr,
	&dev_attr_client_id_set.attr,

	NULL
};


static struct attribute_group mv_dp_client_sysfs_group = {
	.name = "client",
	.attrs = mv_dp_client_sysfs_attrs,
};



int mv_dp_client_sysfs_init(struct kobject *ko)
{
	int err;
	err = sysfs_create_group(ko, &mv_dp_client_sysfs_group);

	if (err) {
		MV_DP_LOG_INF("Client sysFS group init failed %d\n", err);
		return err;
	}

	MV_DP_LOG_DBG1("Client sysFS INITALIZED\n");
	return err;
}

int mv_dp_client_sysfs_exit(struct kobject *ko)
{
	sysfs_remove_group(ko, &mv_dp_client_sysfs_group);
	if (allocated) {
		kfree(clients);
		allocated = 0;
	}

	return 0;
}



static void mv_dp_client_read_mac(int dest, unsigned char *mac)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_l2_addr_t l2;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(dest)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, dest);
		return;
	}

	/*the index is stored in xid*/
	res.cb = mv_dp_client_read_cb;
	res.xid = dest;

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

	memcpy(l2.addr, mac, sizeof(l2.addr));

	rc = mv_nss_dp_client_get_mac(&l2, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Client by MAC to Index: %d Status: %d\n", MV_NSS_DP_FAILED, dest, rc);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Get Client by MAC to Index: %d\n", dest);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Read MAC Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}


static void mv_dp_client_get_mac(unsigned char *mac)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_l2_addr_t l2;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_client_get_cb;
	res.xid = 0;

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

	memcpy(l2.addr, mac, sizeof(l2.addr));

	rc = mv_nss_dp_client_get_mac(&l2, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Client by MAC Status: %d\n", MV_NSS_DP_FAILED, rc);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Get Client by MAC\n");
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get by MAC Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}



static void mv_dp_client_delete(int id)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.cb = mv_dp_client_delete_cb;
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


	rc = mv_nss_dp_client_delete(id, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Delete Client ID %d Status:%d\n", MV_NSS_DP_FAILED,
		       id, rc);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Delete Client ID %d Sent\n", id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Client Delete Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}

}

static void mv_dp_client_read(int dest, int src)
{
	mv_nss_dp_result_spec_t res;
	int rc;
	struct completion *compl_ptr;
	/*src index validation is to be performed in dpapi*/

	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(dest)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, dest);
		return;
	}

	/*the index is stored in xid*/
	res.cb = mv_dp_client_read_cb;
	res.xid = dest;

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

	rc = mv_nss_dp_client_get(src, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Client Index: %d Status: %d\n", MV_NSS_DP_FAILED, src, rc);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Get Client Index: %d\n", src);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Client Read Ind: %d Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT, res.xid);
		kfree(res.cookie);
	}
}


/*dest is the FW client index*/
static void mv_dp_client_get(int id)
{
	mv_nss_dp_result_spec_t res;
	int rc;
	struct completion *compl_ptr;

	res.cb = mv_dp_client_get_cb;
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

	rc = mv_nss_dp_client_get(id, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Client ID: %d, status: %d\n", MV_NSS_DP_FAILED, id, rc);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Get Client ID: %d\n", id);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Client Get: %d Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT, res.xid);
		kfree(res.cookie);
	}

}


static void mv_dp_client_get_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get Client Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.client) {
		MV_DP_CLI_FAIL_CB("Get Client: Empty Client Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Get Client\n", event);
	MV_DP_CLI_CONT("|ID  |TYPE|ID/IDX|MAC         |Cookie  |PRI POL|VLAN|QCF|DSCP|BSS|RADID|MYMAC|<\n");
	mv_dp_client_show_single(event->params.client);
	MV_DP_CLI_CONT("|Client END|<\n");

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);

}



static void mv_dp_client_read_cb(mv_nss_dp_event_t *event)
{
	/*save response in the client buffer and return OK;*/
	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Read Client Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.client) {
		MV_DP_CLI_FAIL_CB("Read Client: Empty Client\n", event);
		goto err;
	}


	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(event->xid)) {
		MV_DP_CLI_FAIL_CB("Illegal Cache Client index: %d\n", event, event->xid);
		goto err;
	}

	memcpy(&clients[event->xid], event->params.client, sizeof(mv_nss_dp_client_t));

	MV_DP_CLI_OK_CB("Get Client Index: %d\n", event, event->xid);

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_client_delete_cb(mv_nss_dp_event_t *event)
{
	/*save response in the client buffer and return OK;*/
	/*4 LSB is in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (!(MV_NSS_DP_OK == event->status || MV_NSS_DP_OK == event->status)) {
		MV_DP_CLI_FAIL_CB("Delete Client Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Delete Client ID: %d\n", event, event->xid);
err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_client_set_cb(mv_nss_dp_event_t *event)
{

	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (!(MV_NSS_DP_OK == event->status || MV_NSS_DP_OK == event->status)) {
		MV_DP_CLI_FAIL_CB("Set Client Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Set Client ID:%d\n", event, event->params.client->client_id);

err:
	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);

}


static void mv_dp_client_commit(int cache_ind)
{
	mv_nss_dp_result_spec_t	res;
	int			tmp_count;
	mv_nss_dp_client_t	*ptr_clients;
	struct completion	*compl_ptr;

	if (cache_ind == -1) {
		/*commit all*/
		tmp_count = allocated;
		ptr_clients = clients;
	} else if (!MV_DP_SYSFS_CLIENTS_NUM_OK(cache_ind)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, cache_ind);
		return;
	} else	{
		tmp_count = 1;
		ptr_clients = &clients[cache_ind];
	}

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

	/*the index is stored in xid*/
	res.cb = mv_dp_client_set_cb;
	res.xid = cache_ind;

	if (MV_NSS_DP_OK != mv_nss_dp_client_set(ptr_clients, tmp_count, &res)) {
		MV_DP_CLI_FAIL("Get Client Index: %d\n", MV_NSS_DP_FAILED, cache_ind);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Commit Client Index: %d, Count: %d\n", cache_ind, tmp_count);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Client Commit: %d Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT, res.xid);
		kfree(res.cookie);
	}
}


static void mv_dp_client_release(void)
{

	if (!allocated) {
		MV_DP_CLI_FAIL("Not Allocated\n", MV_NSS_DP_FAILED);
		return;
	}

	kfree(clients);
	MV_DP_CLI_OK("Clients Cache Released\n");
	allocated = 0;

}

static void mv_dp_client_clear(int cache_ind)
{
	if (!allocated) {
		MV_DP_CLI_FAIL("Not allocated\n", MV_NSS_DP_FAILED);
		return;
	}

	if (cache_ind == -1) {
		memset(clients, 0, sizeof(mv_nss_dp_client_t) * allocated);
		MV_DP_CLI_OK("Clients Cache Cleared: %d\n", allocated);
	} else if (!MV_DP_SYSFS_CLIENTS_NUM_OK(cache_ind)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, cache_ind);
		return;
	} else	{
		memset(&clients[cache_ind], 0, sizeof(mv_nss_dp_client_t));
		MV_DP_CLI_OK("Client Cache %d Cleared\n", cache_ind);
	}
}


static void mv_dp_client_alloc(int num)
{
	if (allocated) {
		MV_DP_CLI_FAIL("Already allocated: %d\n", MV_NSS_DP_FAILED, allocated);
		return;
	}
	if (num < 0 || num > MV_DP_SYSFS_CLIENTS_MAX) {
		MV_DP_CLI_FAIL("Illegal Client Records number: %d\n", MV_NSS_DP_INVALID_PARAM, num);
		return;
	}

	clients = kzalloc(sizeof(mv_nss_dp_client_t) * num, GFP_KERNEL);
	if (!clients) {
		MV_DP_CLI_FAIL("Cache alloc failed for Clients:%d\n", MV_NSS_DP_OUT_OF_RESOURCES, num);
		return;
	}

	allocated = num;
	MV_DP_CLI_OK("Allocated:%d\n", allocated);
}


static void mv_dp_client_show(int index)
{
	int i;

	if (index == -1) {
		/*show all*/
		MV_DP_CLI_OK("|CLIENTS CACHE|TOTAL:%d|<\n", allocated);
		MV_DP_CLI_CONT("|ID  |TYPE|ID/IDX|MAC         |Cookie  |PRI POL|VLAN|QCF|DSCP|BSS|RADID|MYMAC|<\n");
		for (i = 0; i < allocated; i++) {
			MV_DP_CLI_CONT("|%3d", i);
			mv_dp_client_show_single(&clients[i]);
		}

		MV_DP_CLI_CONT("|CLIENTS CAHCE END|<\n");
		return;
	} else if (!MV_DP_SYSFS_CLIENTS_NUM_OK(index)) {
			MV_DP_CLI_FAIL("Illegal Client Records number:%d\n", MV_NSS_DP_INVALID_PARAM, index);
			return;
	}

	/*show single*/
	MV_DP_CLI_OK("|CLIENT CACHE|INDEX:%d|<\n", index);
	MV_DP_CLI_CONT("|ID  |TYPE|ID/IDX|MAC         |Cookie  |PRI POL|VLAN|QCF|DSCP|BSS|RADID|MYMAC|<\n");
	mv_dp_client_show_single(&clients[index]);
	MV_DP_CLI_CONT("|CLIENTS CACHE END|<\n");
}

static void mv_dp_client_show_single(mv_nss_dp_client_t *client)
{

	if (!client) {
		MV_DP_CLI_FAIL("Null client", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("|%4d|%4d|%6d|%02X%02X%02X%02X%02X%02X|%08X|%7d|%4d|%3d|%4d|%3d|%5d|%5d|<\n",
			client->client_id,
			client->type,
			client->u.port_id,
			client->l2_addr.addr[0],
			client->l2_addr.addr[1],
			client->l2_addr.addr[2],
			client->l2_addr.addr[3],
			client->l2_addr.addr[4],
			client->l2_addr.addr[5],
			client->cookie,
			client->prio_policy,
			client->vlan,
			client->qcf_needed,
			client->dscp_needed,
			client->bssid,
			client->radio_id,
			client->is_my_mac);

}


static void mv_dp_client_set_mac(int i, unsigned char *mac)
{
	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (!mac) {
		MV_DP_CLI_FAIL("Null MAC ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	memcpy(clients[i].l2_addr.addr, mac, sizeof(clients[i].l2_addr.addr));
	MV_DP_CLI_OK("Set Client %d MAC:%02X:%02X:%02X:%02X:%02X:%02X\n", i,
		     clients[i].l2_addr.addr[0],
		     clients[i].l2_addr.addr[1],
		     clients[i].l2_addr.addr[2],
		     clients[i].l2_addr.addr[3],
		     clients[i].l2_addr.addr[4],
		     clients[i].l2_addr.addr[5]);
}

static void mv_dp_client_set_ck(int i, u32 ck)
{
	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	clients[i].cookie = ck;
	MV_DP_CLI_OK("Set Client %d COOKIE=0x%08X\n", i, ck);

}

static void mv_dp_client_set_policy(int i, int policy)
{
	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	clients[i].prio_policy = policy;
	MV_DP_CLI_OK("Set Client %d Prio Policy=%d\n", i, clients[i].prio_policy);
}

static void mv_dp_client_set_vlan(int i, int vlan)
{
	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	clients[i].vlan = vlan;
	MV_DP_CLI_OK("Set Client %d Vlan=%d\n", i, clients[i].vlan);
}

static void mv_dp_client_set_qcf(int i, int qcf)
{
	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	clients[i].qcf_needed = qcf;
	MV_DP_CLI_OK("Set Client %d QCF Needed=%d\n", i, clients[i].qcf_needed);
}

static void mv_dp_client_set_bssid(int i, int bssid)
{
	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	clients[i].bssid = bssid;
	MV_DP_CLI_OK("Set Client %d BSSID=%d\n", i, clients[i].bssid);
}

static void mv_dp_client_set_radioid(int i, int rad_id)
{
	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	clients[i].radio_id = rad_id;
	MV_DP_CLI_OK("Set Client %d RaidioID=%d\n", i, clients[i].radio_id);
}


static void mv_dp_client_set_my_mac(int i, int my)
{
	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	clients[i].is_my_mac = my;
	MV_DP_CLI_OK("Set Client %d MyMAC=%d\n", i, clients[i].is_my_mac);
}

static void mv_dp_client_set_dscp(int i, int dscp)
{
	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	clients[i].dscp_needed = dscp;
	MV_DP_CLI_OK("Set Client %d DSCP Needed=%d\n", i, clients[i].dscp_needed);
}

static void mv_dp_client_set_port_type(int i, int type)
{
	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	clients[i].type = type;
	MV_DP_CLI_OK("Set Client %d port type=%d\n", i, clients[i].type);
}

static void mv_dp_client_set_port_id_idx(int i, int id)
{
	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	clients[i].u.port_id = id;
	MV_DP_CLI_OK("Set Client %d port ID/IDX=%d\n", i, clients[i].u.port_id);
}

static void mv_dp_client_set_client_id(int i, int id)
{
	if (!MV_DP_SYSFS_CLIENTS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Client index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	clients[i].client_id = id;
	MV_DP_CLI_OK("Set Client %d ID=%d\n", i, clients[i].client_id);
}
