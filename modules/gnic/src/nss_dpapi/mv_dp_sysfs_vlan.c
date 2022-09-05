
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


#define MV_DP_SYSFS_VLANS_MAX		(48)
#define MV_DP_SYSFS_VLANS_NUM_OK(n)	((n) >= 0 && (n) < allocated)


static  mv_nss_dp_vlan_cfg_t	*vlans;
static  int			allocated; /*number of allocated entries*/


/* VLAN update FW commands */
static void mv_dp_vlan_commit(int i);
static void mv_dp_vlan_read(int i, int index);
static void mv_dp_vlan_read_id(int i, int vlan_id);
static void mv_dp_vlan_get_id(int vlan_id);

/* VLAN update Cache commands */
static void mv_dp_vlan_alloc(int n);
static void mv_dp_vlan_release(void);
static void mv_dp_vlan_clear(int i);
static void mv_dp_vlan_show(int i);
static void mv_dp_vlan_set_id(int i, int id);

static void mv_dp_vlan_set_uc_qos_policy(int i, int policy);
static void mv_dp_vlan_set_mc_qos_policy(int i, int policy);
static void mv_dp_vlan_set_ports_mask(int i, int ports_mask);

/* VLAN get FW commands */
static void mv_dp_vlan_get(int index);

/* VLAN delete FW commands */
static void mv_dp_vlan_delete(int id);
static void mv_dp_vlan_delete_multiple(void);

/* VLAN callbacks */
static void mv_dp_vlan_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_vlan_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_vlan_delete_cb(mv_nss_dp_event_t *event);
static void mv_dp_vlan_read_cb(mv_nss_dp_event_t *event);
static void mv_dp_vlan_get_id_cb(mv_nss_dp_event_t *event);



static ssize_t mv_dp_vlan_sysfs_help(char *b)
{
	int o = 0;

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  help                   - Show Help\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  vlan_release           - Release VLANs Cache\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  vlan_delete_multi      - Delete VLANs from FW pointed by ALL VLAN_IDs\n"
		       "                                               in the cache\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]           > vlan_clear             - Clear VLANs Cache entry [i] or -1 for ALL to 0)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [n]           > vlan_alloc             - Allocate VLAN cache of [n] entries\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [IDX]     > vlan_read              - Read VLAN Record with index [IDX]\n"
		       "                                              from FW to cache index [i]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [IDX]         > vlan_get               - Print only VLAN Record with index [IDX]\n"
		       "                                              read from FW (not saved))\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][VID]      > vlan_read_id           - Read VLAN Record with VLAN[VID]\n"
		       "                                              from FW to cache index [i]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]           > vlan_commit            - Save VLAN stored at index [i] to FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i]           > vlan_show              - Show VLAN cache entry [i] or -1 for ALL\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]          > vlan_delete            - Delete VLAN Record with [ID] from FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [VID]     > vlan_id_set            - Set VLAN [i]'s VLAN ID [VID]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [UP]      > vlan_uc_qos_policy_set - Set VLAN [i]'s Unicast ingress QoS policy [UP]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [MP]      > vlan_mc_qos_policy_set - Set VLAN [i]'s Multicast ingress QoS policy [MP]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [PM]      > vlan_ports_mask_set    - Set VLAN [i]'s Bit mask of phys ETH ports [PM]\n"
		       "                                              (hex 0..0xFF)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [VID]         > vlan_get_id            - Show VLAN Record with VLAN[VID] from FW\n");
	return o;
}

static ssize_t mv_dp_vlan_sysfs_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int off = 0;
	const char  *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "help")) {
		return mv_dp_vlan_sysfs_help(buf);
	} else if (!strcmp(name, "vlan_release")) {
			mv_dp_vlan_release();
	} else if (!strcmp(name, "vlan_delete_multi")) {
			mv_dp_vlan_delete_multiple();
	} else {
		MV_DP_CLI_FAIL("Parse Error CMD: <%s>\n", MV_NSS_DP_INVALID_PARAM, name);
		off = mv_dp_vlan_sysfs_help(buf);
	}
	return off;
}

static ssize_t mv_dp_vlan_sysfs_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t len)
{
	const char *name = attr->attr.name;
	unsigned int    a, b;


	int err;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	err = a = b = 0;

	if (!strcmp(name, "vlan_read")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_vlan_read(a, b);
	} else if (!strcmp(name, "vlan_get")) {
		if (1 != sscanf(buf, "%i", &a))
			goto err;
		mv_dp_vlan_get(a);
	} else if (!strcmp(name, "vlan_read_id")) {
		if (2 != sscanf(buf, "%d %i", &a, &b))
			goto err;
		mv_dp_vlan_read_id(a, b);
	} else if (!strcmp(name, "vlan_get_id")) {
		if (1 != sscanf(buf, "%i", &a))
			goto err;
		mv_dp_vlan_get_id(a);
	} else if (!strcmp(name, "vlan_clear")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_vlan_clear(a);
	} else if (!strcmp(name, "vlan_commit")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_vlan_commit(a);
	} else if (!strcmp(name, "vlan_show")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_vlan_show(a);
	} else if (!strcmp(name, "vlan_alloc")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_vlan_alloc(a);
	} else if (!strcmp(name, "vlan_delete")) {
		if (1 != sscanf(buf, "%i", &a))
			goto err;
		mv_dp_vlan_delete(a);
	} else if (!strcmp(name, "vlan_id_set")) {
		if (2 != sscanf(buf, "%d %i", &a, &b))
			goto err;
		mv_dp_vlan_set_id(a, b);
	} else if (!strcmp(name, "vlan_uc_qos_policy_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_vlan_set_uc_qos_policy(a, b);
	} else if (!strcmp(name, "vlan_mc_qos_policy_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_vlan_set_mc_qos_policy(a, b);
	} else if (!strcmp(name, "vlan_ports_mask_set")) {
		if (2 != sscanf(buf, "%d 0x%X", &a, &b))
			goto err;
		mv_dp_vlan_set_ports_mask(a, b);
	} else {
		goto err;
	}

	return len;
err:
	MV_DP_LOG_INF("Parse Error CMD:<%s>\n", attr->attr.name);
	return -EINVAL;
}

static DEVICE_ATTR(help,				S_IRUSR, mv_dp_vlan_sysfs_show, NULL);
static DEVICE_ATTR(vlan_release,			S_IRUSR, mv_dp_vlan_sysfs_show, NULL);
static DEVICE_ATTR(vlan_delete_multi,			S_IRUSR, mv_dp_vlan_sysfs_show, NULL);
static DEVICE_ATTR(vlan_clear,				S_IWUSR, NULL, mv_dp_vlan_sysfs_store);
static DEVICE_ATTR(vlan_read,				S_IWUSR, NULL, mv_dp_vlan_sysfs_store);
static DEVICE_ATTR(vlan_read_id,			S_IWUSR, NULL, mv_dp_vlan_sysfs_store);
static DEVICE_ATTR(vlan_get,				S_IWUSR, NULL, mv_dp_vlan_sysfs_store);
static DEVICE_ATTR(vlan_commit,				S_IWUSR, NULL, mv_dp_vlan_sysfs_store);
static DEVICE_ATTR(vlan_show,				S_IWUSR, NULL, mv_dp_vlan_sysfs_store);
static DEVICE_ATTR(vlan_alloc,				S_IWUSR, NULL, mv_dp_vlan_sysfs_store);
static DEVICE_ATTR(vlan_delete,				S_IWUSR, NULL, mv_dp_vlan_sysfs_store);
static DEVICE_ATTR(vlan_id_set,				S_IWUSR, NULL, mv_dp_vlan_sysfs_store);
static DEVICE_ATTR(vlan_uc_qos_policy_set,		S_IWUSR, NULL, mv_dp_vlan_sysfs_store);
static DEVICE_ATTR(vlan_mc_qos_policy_set,		S_IWUSR, NULL, mv_dp_vlan_sysfs_store);
static DEVICE_ATTR(vlan_ports_mask_set,			S_IWUSR, NULL, mv_dp_vlan_sysfs_store);
static DEVICE_ATTR(vlan_get_id,				S_IWUSR, NULL, mv_dp_vlan_sysfs_store);


static struct attribute *mv_dp_vlan_sysfs_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_vlan_release.attr,
	&dev_attr_vlan_delete_multi.attr,
	&dev_attr_vlan_clear.attr,
	&dev_attr_vlan_read.attr,
	&dev_attr_vlan_read_id.attr,
	&dev_attr_vlan_get.attr,
	&dev_attr_vlan_commit.attr,
	&dev_attr_vlan_show.attr,
	&dev_attr_vlan_alloc.attr,
	&dev_attr_vlan_delete.attr,
	&dev_attr_vlan_id_set.attr,
	&dev_attr_vlan_uc_qos_policy_set.attr,
	&dev_attr_vlan_mc_qos_policy_set.attr,
	&dev_attr_vlan_ports_mask_set.attr,
	&dev_attr_vlan_get_id.attr,

	NULL
};


static struct attribute_group mv_dp_vlan_sysfs_group = {
	.name = "vlan",
	.attrs = mv_dp_vlan_sysfs_attrs,
};



int mv_dp_vlan_sysfs_init(struct kobject *ko)
{
	int err;

	err = sysfs_create_group(ko, &mv_dp_vlan_sysfs_group);

	if (err) {
		MV_DP_LOG_INF("VLAN sysFS group init failed %d\n", err);
		return err;
	}
	allocated = 0;
	MV_DP_LOG_DBG1("VLAN sysFS INITALIZED\n");
	return err;
}

int mv_dp_vlan_sysfs_exit(struct kobject *ko)
{
	sysfs_remove_group(ko, &mv_dp_vlan_sysfs_group);
	if (allocated) {
		kfree(vlans);
		allocated = 0;
	}
	return 0;
}



static void mv_dp_vlan_show_single(mv_nss_dp_vlan_cfg_t *vlan)
{


	if (!vlan) {
		MV_DP_CLI_FAIL("Null VLAN", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("|%7d|%10d|%10d|0x%07X|<\n",
			vlan->vlan_id,
			vlan->uc_qos_policy,
			vlan->mc_qos_policy,
			vlan->ports_mask);

}


static void mv_dp_vlan_commit(int i)
{
	mv_nss_dp_result_spec_t	res;
	int			tmp_count;
	mv_nss_dp_vlan_cfg_t	*vlans_ptr;
	struct completion	*compl_ptr;
	mv_nss_dp_status_t	rc;

	if (i == -1) {
		/*commit all*/
		tmp_count = allocated;
		vlans_ptr = vlans;
	} else {
		if (!MV_DP_SYSFS_VLANS_NUM_OK(i)) {
			MV_DP_CLI_FAIL("Illegal Cache VLAN index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
			return;
		}
		tmp_count = 1;
		vlans_ptr = &vlans[i];
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
	res.cb = mv_dp_vlan_set_cb;
	res.xid = i;

	rc = mv_nss_dp_vlan_cfg_set(vlans_ptr, tmp_count, &res);
	if (rc != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL("Set VLAN Index: %d Status: %s\n", rc, i, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Commit VLAN Index: %d, Count: %d\n", i, tmp_count);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("VLAN Commit: %d Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       res.xid, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
		kfree(res.cookie);

}


static void mv_dp_vlan_release(void)
{

	if (!allocated) {
		MV_DP_CLI_FAIL("Not Allocated\n", MV_NSS_DP_FAILED);
		return;
	}

	kfree(vlans);
	MV_DP_CLI_OK("VLANs Cache Released\n");
	allocated = 0;

}

static void mv_dp_vlan_clear(int i)
{
	if (!allocated) {
		MV_DP_CLI_FAIL("Not allocated\n", MV_NSS_DP_FAILED);
		return;
	}

	if (i == -1) {
		/*clear all*/
		memset(vlans, 0, sizeof(mv_nss_dp_vlan_cfg_t)*allocated);
		MV_DP_CLI_OK("VLANs Cache Cleared: %d\n", allocated);
	} else if (!MV_DP_SYSFS_VLANS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache VLAN index:%d\n", MV_NSS_DP_INVALID_PARAM, i);
	} else {
		memset(&vlans[i], 0, sizeof(mv_nss_dp_vlan_cfg_t));
		MV_DP_CLI_OK("VLANs Cache entry %d cleared\n", i);
	}
}

static void mv_dp_vlan_read(int i, int index)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;
	mv_nss_dp_status_t	rc;

	/*VLAN index validation is to be performed in dpapi*/

	if (!MV_DP_SYSFS_VLANS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache VLAN index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	/*the index is stored in xid*/
	res.cb = mv_dp_vlan_read_cb;
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

	rc = mv_nss_dp_vlan_cfg_get((uint16_t)index, &res);
	if (rc != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL("Get VLAN idx %d to Index: %d Status: %s\n", rc, index, i, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get VLAN idx %d to Index: %d\n", index, i);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get VLAN Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}
err_free:
		kfree(res.cookie);
}

static void mv_dp_vlan_read_id(int i, int vlan_id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;
	mv_nss_dp_status_t	rc;

	/*VLAN index validation is to be performed in dpapi*/

	if (!MV_DP_SYSFS_VLANS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache VLAN index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	/*the index is stored in xid*/
	res.cb = mv_dp_vlan_read_cb;
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

	rc = mv_nss_dp_vlan_cfg_get_by_id(vlan_id, &res);
	if (rc != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL("Get VLAN ID %d to Index: %d Status: %s\n", rc, vlan_id, i, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get VLAN ID %d to Index: %d\n", vlan_id, i);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get VLAN ID Status:%s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
		kfree(res.cookie);
}

static void mv_dp_vlan_get(int index)
{

	mv_nss_dp_result_spec_t	res;
	struct completion	*compl_ptr;
	mv_nss_dp_status_t	rc;

	/*the index is stored in xid*/
	res.cb = mv_dp_vlan_get_cb;
	res.xid = index;

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

	rc = mv_nss_dp_vlan_cfg_get((uint16_t)index, &res);
	if (rc != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL("Get VLAN idx: %d Status: %s\n", rc, index, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get VLAN idx %d\n", index);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get VLAN Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
		kfree(res.cookie);

}

static void mv_dp_vlan_get_id(int vlan_id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;
	mv_nss_dp_status_t	rc;


	/*vlan_id is stored in xid*/
	res.cb = mv_dp_vlan_get_id_cb;
	res.xid = vlan_id;

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

	rc = mv_nss_dp_vlan_cfg_get_by_id(vlan_id, &res);
	if (rc != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL("Get VLAN ID %d Status: %s\n", rc, vlan_id, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get VLAN ID %d\n", vlan_id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get VLAN ID Status:%s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	kfree(res.cookie);
}

static void mv_dp_vlan_show(int index)
{
	int i;

	if (index == -1) {
		/*show all*/
		MV_DP_CLI_OK("|VLAN CACHE|TOTAL:%d|<\n", allocated);
		MV_DP_CLI_CONT("|IND|VLAN ID|UC QOS POL|MC QOS POL|PORT MASK|<\n");
		for (i = 0; i < allocated; i++) {
			MV_DP_CLI_CONT("|%3d", i);
			mv_dp_vlan_show_single(&vlans[i]);
		}

		MV_DP_CLI_CONT("|VLAN CAHCE END|<\n");
		return;
	} else if (!MV_DP_SYSFS_VLANS_NUM_OK(index)) {
		MV_DP_CLI_FAIL("Illegal VLAN Record number:%d\n", MV_NSS_DP_INVALID_PARAM, index);
		return;
	}

	/*show single*/
	MV_DP_CLI_OK("|VLAN CACHE|INDEX:%d|<\n", index);
	MV_DP_CLI_CONT("|VLAN ID|UC QOS POL|MC QOS POL|PORT MASK|<\n");
	mv_dp_vlan_show_single(&vlans[index]);
	MV_DP_CLI_CONT("|VLANS CACHE END|<\n");
}

static void mv_dp_vlan_alloc(int n)
{
	if (allocated) {
		MV_DP_CLI_FAIL("Already allocated: %d\n", MV_NSS_DP_FAILED, allocated);
		return;
	}
	if (n < 0 || n > MV_DP_SYSFS_VLANS_MAX) {
		MV_DP_CLI_FAIL("Illegal VLAN Record number: %d\n", MV_NSS_DP_INVALID_PARAM, n);
		return;
	}

	vlans = kcalloc(n, sizeof(mv_nss_dp_vlan_cfg_t), GFP_KERNEL);
	if (!vlans) {
		MV_DP_CLI_FAIL("Cache alloc failed for VLANs: %d\n", MV_NSS_DP_OUT_OF_RESOURCES, n);
		return;
	}

	allocated = n;
	MV_DP_CLI_OK("Allocated: %d\n", allocated);
}

static void mv_dp_vlan_delete(int id)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;
	mv_nss_dp_status_t	rc;

	/*VLAN ID validation is to be performed in dpapi*/
	/*the VLAN ID is stored in xid*/
	res.cb = mv_dp_vlan_delete_cb;
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

	rc = mv_nss_dp_vlan_cfg_delete((uint16_t *)&id, 1, &res);
	if (rc != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL("Delete VLAN %d Status: %s", rc, id, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Deleting VLAN: %d\n", id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Deleting VLAN:%d Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT, id,
				       mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
		kfree(res.cookie);
}


static void mv_dp_vlan_delete_multiple(void)
{
	mv_nss_dp_result_spec_t	res;
	mv_nss_dp_status_t	rc;
	struct completion	*compl_ptr;
	uint16_t		*vids;
	int			i;

	/*the index is stored in xid*/
	res.cb = mv_dp_vlan_delete_cb;
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
		MV_DP_CLI_FAIL("No VLANS Allocated\n", MV_NSS_DP_INVALID_PARAM);
		goto err_free;
	}

	vids = kcalloc(allocated, sizeof(uint16_t), GFP_KERNEL);
	if (!vids) {
		MV_DP_CLI_FAIL("vlan_ids allocation Failed: %s\n", MV_NSS_DP_OUT_OF_RESOURCES,
			       mv_dp_err_name_get(MV_NSS_DP_OUT_OF_RESOURCES));
		goto err_free;
	}

	for (i = 0; i < allocated; i++)
		vids[i] = vlans[i].vlan_id;

	rc = mv_nss_dp_vlan_cfg_delete(vids, allocated, &res);
	if (rc != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL("VLAN Delete Count:%d Status:%s\n", rc, allocated, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Delete VLANs Count:%d\n", allocated);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("VLAN Delete Count:%d Status:%s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       allocated, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
		kfree(res.cookie);
		kfree(vids);
}




static void mv_dp_vlan_set_id(int i, int id)
{
	if (!MV_DP_SYSFS_VLANS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache VLAN index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	vlans[i].vlan_id = (uint16_t)id;
	MV_DP_CLI_OK("Set VLAN Index %d ID = %d\n", i, vlans[i].vlan_id);

}

static void mv_dp_vlan_set_uc_qos_policy(int i, int policy)
{
	if (!MV_DP_SYSFS_VLANS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache VLAN index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	vlans[i].uc_qos_policy = (uint8_t)policy;
	MV_DP_CLI_OK("Set VLAN Index %d UC QOS Policy = %d\n", i, vlans[i].uc_qos_policy);

}

static void mv_dp_vlan_set_mc_qos_policy(int i, int policy)
{
	if (!MV_DP_SYSFS_VLANS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache VLAN index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	vlans[i].mc_qos_policy = (uint8_t)policy;
	MV_DP_CLI_OK("Set VLAN Index %d MC QOS Policy = %d\n", i, vlans[i].mc_qos_policy);

}

static void mv_dp_vlan_set_ports_mask(int i, int ports_mask)
{
	if (!MV_DP_SYSFS_VLANS_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache VLAN index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	vlans[i].ports_mask = (uint8_t)ports_mask;
	MV_DP_CLI_OK("Set VLAN Index %d Ports mask = 0x%02X\n", i, vlans[i].ports_mask);

}

void mv_dp_vlan_set_cb(mv_nss_dp_event_t *event)
{

	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (!(MV_NSS_DP_OK == event->status || MV_NSS_DP_END_OF_LIST == event->status)) {
		MV_DP_CLI_FAIL_CB("Set VLAN Index: %d Count: %d Status: %s\n", event, event->xid, event->count,
				  mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("Set VLAN Index: %d Count:%d\n", event, event->xid, event->count);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

void mv_dp_vlan_get_cb(mv_nss_dp_event_t *event)
{

	/*Print response and return OK;*/
	/*VLAN Index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (event->status != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL_CB("Get VLAN index: %d Status: %s\n",
				event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.port) {
		MV_DP_CLI_FAIL_CB("Get VLAN index: %d Empty VLAN\n", event, event->xid);
		goto err;
	}

	MV_DP_CLI_OK_CB("Get VLAN Index: %d\n", event, event->xid);
	MV_DP_CLI_CONT("|VLAN ID|UC QOS POL|UC QOS POL|PORT MASK|<\n");
	mv_dp_vlan_show_single(event->params.vlan_cfg);
	MV_DP_CLI_CONT("|VLAN END|<\n");

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

void mv_dp_vlan_delete_cb(mv_nss_dp_event_t *event)
{

	/*save response in the port buffer and return OK;*/
	/*Port ID is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (!(MV_NSS_DP_OK == event->status || MV_NSS_DP_END_OF_LIST == event->status)) {
		MV_DP_CLI_FAIL_CB("Delete VLAN ID %d Count: %d Status: %s\n", event,
				event->xid, event->count, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Deleted VLAN ID %d Count:%d\n", event, event->xid, event->count);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

void mv_dp_vlan_read_cb(mv_nss_dp_event_t *event)
{

	/*save response in the vlan buffer and return OK;*/
	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (event->status != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL_CB("Get VLAN to Cache Index: %d Status: %s\n",
				event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.vlan_cfg) {
		MV_DP_CLI_FAIL_CB("Get VLAN to Cache Index: %d Empty VLAN\n", event, event->xid);
		goto err;
	}

	if (!MV_DP_SYSFS_VLANS_NUM_OK(event->xid)) {
		MV_DP_CLI_FAIL_CB("Illegal Cache VLAN index: %d\n", event, event->xid);
		goto err;
	}

	memcpy(&vlans[event->xid], event->params.vlan_cfg, sizeof(mv_nss_dp_vlan_cfg_t));

	MV_DP_CLI_OK_CB("Get VLAN to Cache Index: %d\n", event, event->xid);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

void mv_dp_vlan_get_id_cb(mv_nss_dp_event_t *event)
{

	/*print out the vlan configuration and return OK;*/
	/*the vlan_id is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (event->status != MV_NSS_DP_OK) {
		MV_DP_CLI_FAIL_CB("Get VLAN ID : %d Status: %s\n",
				event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.vlan_cfg) {
		MV_DP_CLI_FAIL_CB("Get VLAN ID: %d Empty VLAN\n", event, event->xid);
		goto err;
	}

	if (event->xid != event->params.vlan_cfg->vlan_id) {
		MV_DP_CLI_FAIL_CB("FW VLAN ID: %d doesnt equal to event's VLAN ID %d\n",
				   event, event->xid, event->params.vlan_cfg->vlan_id);
		goto err;
	}

	MV_DP_CLI_OK_CB("Get VLAN ID: %d\n", event, event->xid);

	MV_DP_CLI_CONT("|VLAN ID|UC QOS POL|UC QOS POL|PORT MASK|<\n");
	mv_dp_vlan_show_single(event->params.vlan_cfg);
	MV_DP_CLI_CONT("|VLAN END|<\n");

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

