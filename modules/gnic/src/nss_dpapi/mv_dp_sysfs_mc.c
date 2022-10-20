/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

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

enum mv_dp_sysfs_mc_type {
	MV_DP_SYSFS_MC_BRIDGED = 0,
	MV_DP_SYSFS_MC_TUNNELED = 1,
	MV_DP_SYSFS_MC_LAST
};

#define MV_DP_SYSFS_MC_MAX		(16)
#define MV_DP_SYSFS_MC_MAX_NUM_OK(n)	((n) >= 0 && (n) < MV_DP_SYSFS_MC_MAX)
#define MV_DP_SYSFS_MC_BR_NUM_OK(n)	((n) >= 0 && (n) < br_allocated)
#define MV_DP_SYSFS_MC_TN_NUM_OK(n)	((n) >= 0 && (n) < tn_allocated)


static  mv_nss_dp_mc_tunneled_cfg_t	*tn_mc;
static  int				tn_allocated; /*number of allocated entries*/

static  mv_nss_dp_mc_bridged_cfg_t	*br_mc;
static  int				br_allocated; /*number of allocated entries*/


/* MC update FW commands */
static void mv_dp_mc_commit(int i, enum mv_dp_sysfs_mc_type type);
static void mv_dp_mc_tn_commit(int i);
static void mv_dp_mc_br_commit(int i);
static void mv_dp_mc_delete(int i, enum mv_dp_sysfs_mc_type type);
static void mv_dp_mc_br_delete(int i);
static void mv_dp_mc_tn_delete(int i);
static void mv_dp_mc_get(int index, enum mv_dp_sysfs_mc_type type);
static void mv_dp_mc_br_get(int index);
static void mv_dp_mc_tn_get(int index);


/* MC update Cache commands */
static void mv_dp_mc_alloc(int n, enum mv_dp_sysfs_mc_type type);
static void mv_dp_mc_release(enum mv_dp_sysfs_mc_type type);
static void mv_dp_mc_clear(int i, enum mv_dp_sysfs_mc_type type);
static void mv_dp_mc_br_clear(int i);
static void mv_dp_mc_tn_clear(int i);
static void mv_dp_mc_show(int i, enum mv_dp_sysfs_mc_type type);
static void mv_dp_mc_read(int i, int index, enum mv_dp_sysfs_mc_type type);
static void mv_dp_mc_br_read(int i, int index);
static void mv_dp_mc_tn_read(int i, int index);

/*MC fields set*/
static void mv_dp_mc_br_l2_set(int i, unsigned char *mac);
static void mv_dp_mc_br_vid_set(int i, u16 vid);
static void mv_dp_mc_br_od_set(int i, u32 od);

static void mv_dp_mc_tn_mgid_set(int i, u16 mgid);
static void mv_dp_mc_tn_ingr_policy_id_set(int i, int id);


/*callbacks */
static void mv_dp_mc_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_mc_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_mc_delete_cb(mv_nss_dp_event_t *event);
static void mv_dp_mc_read_cb(mv_nss_dp_event_t *event);



static ssize_t mv_dp_mc_sysfs_help(char *b)
{
	int o = 0;

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                  help                 - Show Help\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [t]           > mc_release           - Release MC Cache Table of type [t]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [t]       > mc_clear             - Clear MC Cache entry [i] of type [t]\n"
		       "                                            or -1 for ALL to 0)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [n] [t]       > mc_alloc             - Allocate MC cache of type [t] of [n] entries\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [ind] [t] > mc_read              - Read MC Record of type [t] with index [ind]\n"
		       "                                            from FW to cache index [i]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ind] [t]     > mc_get               - Print only MC Record with index [ind] of type [t]\n"
		       "                                            from FW (not saved))\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [t]       > mc_commit            - Save MC stored at index [i] of type [t] to FW or\n"
		       "                                            -1 for ALL\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [t]       > mc_show              - Show MC cache entry [i] of type [t] or -1 for ALL\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [t]       > mc_delete            - Delete MC Record with:\n"
		       "                                            For Tunneled MC: [i] = mgid at cache[i]\n"
		       "                                            For Bridged MC: [i] = cache entry index\n"
		       "                                            -1 for all cached mgid or indecies.\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [MAC]     > mc_br_l2_set         - Set Bridged MC entry [i] L2 address to [MAC]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [OD]      > mc_br_od_set         - Set Bridged MC entry [i] Opaque (hex32) to [od]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [vid]     > mc_br_vid_set        - Set Bridged MC entry [i] vlan id (dec16) to [vid]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [mgid]    > mc_tn_mgid_set       - Set Tunneled MC entry [i] mgid (dec16) to [mgid]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i] [pol]     > mc_tn_policy_set     - Set Tunneled MC entry [i] ingress policy to [pol]\n");


	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n\n                   [t] - MC type: %d -- Bridged, %d -- Tunneled\n",
						MV_DP_SYSFS_MC_BRIDGED,
						MV_DP_SYSFS_MC_TUNNELED);
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [MAC]= MAC address as 00:00:00:00:00:00");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   dec - decimal (123); hex - hexadecimal (0x0)\n");


	return o;
}

static ssize_t mv_dp_mc_sysfs_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int off = 0;
	const char  *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "help")) {
		return mv_dp_mc_sysfs_help(buf);
	} else {
		MV_DP_CLI_FAIL("Parse Error CMD: <%s>\n", MV_NSS_DP_INVALID_PARAM, name);
		off = mv_dp_mc_sysfs_help(buf);
	}
	return off;
}

static ssize_t mv_dp_mc_sysfs_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t len)
{
	const char *name = attr->attr.name;
	unsigned int    a, b, c;
	unsigned char mac[6];
	u16		us;
	int err;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	err = a = b = c = 0;

	if (!strcmp(name, "mc_read")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_mc_read(a, b, c);
	} else if (!strcmp(name, "mc_get")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_mc_get(a, b);
	} else if (!strcmp(name, "mc_release")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_mc_release(a);
	} else if (!strcmp(name, "mc_clear")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_mc_clear(a, b);
	} else if (!strcmp(name, "mc_commit")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_mc_commit(a, b);
	} else if (!strcmp(name, "mc_show")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_mc_show(a, b);
	} else if (!strcmp(name, "mc_alloc")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_mc_alloc(a, b);
	} else if (!strcmp(name, "mc_delete")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_mc_delete(a, b);
	} else if (!strcmp(name, "mc_br_l2_set")) {
		if (7 != sscanf(buf, "%d %hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
		       &a, &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]))
			goto err;
		mv_dp_mc_br_l2_set(a, mac);
	} else if (!strcmp(name, "mc_br_od_set")) {
		if (2 != sscanf(buf, "%d %i", &a, &b))
			goto err;
		mv_dp_mc_br_od_set(a, b);
	} else if (!strcmp(name, "mc_br_vid_set")) {
		if (2 != sscanf(buf, "%d %hi", &a, &us))
			goto err;
		mv_dp_mc_br_vid_set(a, us);
	} else if (!strcmp(name, "mc_tn_mgid_set")) {
		if (2 != sscanf(buf, "%d %hi", &a, &us))
			goto err;
		mv_dp_mc_tn_mgid_set(a, us);
	} else if (!strcmp(name, "mc_tn_policy_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_mc_tn_ingr_policy_id_set(a, b);
	} else {
		goto err;
	}

	return len;
err:
	MV_DP_LOG_INF("Parse Error CMD:<%s>\n", attr->attr.name);
	return -EINVAL;
}

static DEVICE_ATTR(help,				S_IRUSR, mv_dp_mc_sysfs_show, NULL);
static DEVICE_ATTR(mc_release,				S_IWUSR, NULL, mv_dp_mc_sysfs_store);
static DEVICE_ATTR(mc_clear,				S_IWUSR, NULL, mv_dp_mc_sysfs_store);
static DEVICE_ATTR(mc_read,				S_IWUSR, NULL, mv_dp_mc_sysfs_store);
static DEVICE_ATTR(mc_get,				S_IWUSR, NULL, mv_dp_mc_sysfs_store);
static DEVICE_ATTR(mc_commit,				S_IWUSR, NULL, mv_dp_mc_sysfs_store);
static DEVICE_ATTR(mc_show,				S_IWUSR, NULL, mv_dp_mc_sysfs_store);
static DEVICE_ATTR(mc_alloc,				S_IWUSR, NULL, mv_dp_mc_sysfs_store);
static DEVICE_ATTR(mc_delete,				S_IWUSR, NULL, mv_dp_mc_sysfs_store);
static DEVICE_ATTR(mc_br_l2_set,			S_IWUSR, NULL, mv_dp_mc_sysfs_store);
static DEVICE_ATTR(mc_br_od_set,			S_IWUSR, NULL, mv_dp_mc_sysfs_store);
static DEVICE_ATTR(mc_br_vid_set,			S_IWUSR, NULL, mv_dp_mc_sysfs_store);
static DEVICE_ATTR(mc_tn_mgid_set,			S_IWUSR, NULL, mv_dp_mc_sysfs_store);
static DEVICE_ATTR(mc_tn_policy_set,			S_IWUSR, NULL, mv_dp_mc_sysfs_store);


static struct attribute *mv_dp_mc_sysfs_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_mc_release.attr,
	&dev_attr_mc_clear.attr,
	&dev_attr_mc_read.attr,
	&dev_attr_mc_get.attr,
	&dev_attr_mc_commit.attr,
	&dev_attr_mc_show.attr,
	&dev_attr_mc_alloc.attr,
	&dev_attr_mc_delete.attr,
	&dev_attr_mc_br_l2_set.attr,
	&dev_attr_mc_br_od_set.attr,
	&dev_attr_mc_br_vid_set.attr,
	&dev_attr_mc_tn_mgid_set.attr,
	&dev_attr_mc_tn_policy_set.attr,

	NULL
};


static struct attribute_group mv_dp_mc_sysfs_group = {
	.name = "mc",
	.attrs = mv_dp_mc_sysfs_attrs,
};



int mv_dp_mc_sysfs_init(struct kobject *ko)
{
	int err;
	err = sysfs_create_group(ko, &mv_dp_mc_sysfs_group);

	if (err) {
		MV_DP_LOG_INF("MC sysFS group init failed %d\n", err);
		return err;
	}
	tn_allocated = br_allocated = 0;
	MV_DP_LOG_DBG1("MC sysFS INITALIZED\n");
	return err;
}

int mv_dp_mc_sysfs_exit(struct kobject *ko)
{
	sysfs_remove_group(ko, &mv_dp_mc_sysfs_group);
	if (tn_allocated) {
		kfree(tn_mc);
		tn_allocated = 0;
	}

	if (br_allocated) {
		kfree(br_mc);
		br_allocated = 0;
	}

	return 0;
}



static void mv_dp_mc_br_show_single(mv_nss_dp_mc_bridged_cfg_t *br)
{

	if (!br) {
		MV_DP_CLI_FAIL("Null MC", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("|%02X%02X%02X%02X%02X%02X|%08X|%4d|<\n",
		       br->l2_addr.addr[0],
		       br->l2_addr.addr[1],
		       br->l2_addr.addr[2],
		       br->l2_addr.addr[3],
		       br->l2_addr.addr[4],
		       br->l2_addr.addr[5],
		       br->opaque,
		       br->vlan_id);
}

static void mv_dp_mc_tn_show_single(mv_nss_dp_mc_tunneled_cfg_t *tn)
{

	if (!tn) {
		MV_DP_CLI_FAIL("Null MC", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("|%4d|%6d|<\n",
		       tn->mgid,
		       tn->policy_id);
}

static void mv_dp_mc_br_show(int index)
{

	int i;

	if (index == -1) {
		/*show all*/
		MV_DP_CLI_OK("|BRIDGED MC|TOTAL:%d|<\n", br_allocated);
		MV_DP_CLI_CONT("|IND|MAC         | OPAQUE |VLAN|<\n");
		for (i = 0; i < br_allocated; i++) {
			MV_DP_CLI_CONT("|%3d", i);
			mv_dp_mc_br_show_single(&br_mc[i]);
		}

		MV_DP_CLI_CONT("|BRIDGED MC CACHE END|<\n");
		return;
	} else if (!MV_DP_SYSFS_MC_BR_NUM_OK(index)) {
			MV_DP_CLI_FAIL("Illegal BRIDGED MC Record cache index:%d\n", MV_NSS_DP_INVALID_PARAM, index);
			return;
	}

	/*show single*/
	MV_DP_CLI_OK("|BRIDGED MC|INDEX:%d|<\n", index);
	MV_DP_CLI_CONT("|MAC         | OPAQUE |VLAN|<\n");
	mv_dp_mc_br_show_single(&br_mc[index]);
	MV_DP_CLI_CONT("|BRIDGED MC CACHE END|<\n");
}

static void mv_dp_mc_tn_show(int index)
{

	int i;

	if (index == -1) {
		/*show all*/
		MV_DP_CLI_OK("|TUNNELED MC|TOTAL:%d|<\n", tn_allocated);
		MV_DP_CLI_CONT("|IND|MGID|POLICY|<\n");
		for (i = 0; i < tn_allocated; i++) {
			MV_DP_CLI_CONT("|%3d", i);
			mv_dp_mc_tn_show_single(&tn_mc[i]);
		}

		MV_DP_CLI_CONT("|TUNNELED MC CACHE END|<\n");
		return;
	} else if (!MV_DP_SYSFS_MC_TN_NUM_OK(index)) {
			MV_DP_CLI_FAIL("Illegal TUNNELED MC Record cache index:%d\n", MV_NSS_DP_INVALID_PARAM, index);
			return;
	}

	/*show single*/
	MV_DP_CLI_OK("|TUNNELED MC|INDEX:%d|<\n", index);
	MV_DP_CLI_CONT("|MGID|POLICY|<\n");
	mv_dp_mc_tn_show_single(&tn_mc[index]);
	MV_DP_CLI_CONT("|TUNNELED MC CACHE END|<\n");
}


static void mv_dp_mc_show(int i, enum mv_dp_sysfs_mc_type t)
{

	switch (t) {

	case MV_DP_SYSFS_MC_BRIDGED:
		mv_dp_mc_br_show(i);
		break;
	case MV_DP_SYSFS_MC_TUNNELED:
		mv_dp_mc_tn_show(i);
		break;

	default:
		MV_DP_CLI_FAIL("Illegal MC TYPE:%d\n", MV_NSS_DP_INVALID_PARAM, t);

	}
}



static void mv_dp_mc_commit(int i, enum mv_dp_sysfs_mc_type t)
{
	switch (t) {

	case MV_DP_SYSFS_MC_BRIDGED:
		mv_dp_mc_br_commit(i);
		break;
	case MV_DP_SYSFS_MC_TUNNELED:
		mv_dp_mc_tn_commit(i);
		break;

	default:
		MV_DP_CLI_FAIL("Illegal MC TYPE:%d\n", MV_NSS_DP_INVALID_PARAM, t);
	}
}

static void mv_dp_mc_tn_commit(int i)
{
	mv_nss_dp_result_spec_t		res;
	int				tmp_count;
	mv_nss_dp_mc_tunneled_cfg_t	*mc_ptr;
	struct completion		*compl_ptr;
	mv_nss_dp_status_t		rc;

	if (i == -1) {
		/*commit all*/
		tmp_count = tn_allocated;
		mc_ptr = tn_mc;
	} else if (!MV_DP_SYSFS_MC_TN_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Tunneled MC index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	} else	{
		tmp_count = 1;
		mc_ptr = &tn_mc[i];
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
	res.cb = mv_dp_mc_set_cb;
	res.xid = i;

	rc = mv_nss_dp_mc_tunneled_cfg_set(mc_ptr, tmp_count, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Set Tunneled MC Index: %d Status: %s\n", rc, i, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Commit Tunneled MC Index: %d, Count: %d\n", i, tmp_count);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Tunneled MC Commit: %d Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       res.xid, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);

}

static void mv_dp_mc_br_commit(int i)
{
	mv_nss_dp_result_spec_t		res;
	int				tmp_count;
	mv_nss_dp_mc_bridged_cfg_t	*br_ptr;
	struct completion		*compl_ptr;
	mv_nss_dp_status_t		rc;

	if (i == -1) {
		/*commit all*/
		tmp_count = br_allocated;
		br_ptr = br_mc;
	} else if (!MV_DP_SYSFS_MC_BR_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Bridged MC index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	} else	{
		tmp_count = 1;
		br_ptr = &br_mc[i];
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
	res.cb = mv_dp_mc_set_cb;
	res.xid = i;

	rc = mv_nss_dp_mc_bridged_cfg_set(br_ptr, tmp_count, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Set Bridged MC Index: %d Status: %s\n", rc, i, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Commit Bridged MC Index: %d, Count: %d\n", i, tmp_count);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Bridged MC Commit: %d Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       res.xid, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);

}


static void mv_dp_mc_release(enum mv_dp_sysfs_mc_type t)
{

	switch (t) {
	case MV_DP_SYSFS_MC_BRIDGED:
		if (!br_allocated) {
			MV_DP_CLI_FAIL("Not Allocated\n", MV_NSS_DP_FAILED);
			return;
		}

		kfree(br_mc);
		MV_DP_CLI_OK("Bridged MC Cache Released\n");
		br_allocated = 0;
		break;
	case MV_DP_SYSFS_MC_TUNNELED:
		if (!tn_allocated) {
			MV_DP_CLI_FAIL("Not Allocated\n", MV_NSS_DP_FAILED);
			return;
		}

		kfree(tn_mc);
		MV_DP_CLI_OK("Tunneled MC Cache Released\n");
		tn_allocated = 0;
		break;

	default:
		MV_DP_CLI_FAIL("Illegal MC TYPE:%d\n", MV_NSS_DP_INVALID_PARAM, t);
	}
}

static void mv_dp_mc_clear(int i, enum mv_dp_sysfs_mc_type t)
{
	switch (t) {

	case MV_DP_SYSFS_MC_BRIDGED:
		mv_dp_mc_br_clear(i);
		break;
	case MV_DP_SYSFS_MC_TUNNELED:
		mv_dp_mc_tn_clear(i);
		break;

	default:
		MV_DP_CLI_FAIL("Illegal MC TYPE:%d\n", MV_NSS_DP_INVALID_PARAM, t);
	}
}

static void mv_dp_mc_tn_clear(int i)
{

	if (!tn_allocated) {
		MV_DP_CLI_FAIL("Not allocated\n", MV_NSS_DP_FAILED);
		return;
	}

	if (i == -1) {
		/*clear all*/
		memset(tn_mc, 0, sizeof(mv_nss_dp_mc_tunneled_cfg_t) * tn_allocated);
		MV_DP_CLI_OK("Tunneled MC Cache Cleared: %d\n", tn_allocated);
	} else if (!MV_DP_SYSFS_MC_TN_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Tunneled MC index:%d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	} else {
		memset(&tn_mc[i], 0, sizeof(mv_nss_dp_mc_tunneled_cfg_t));
		MV_DP_CLI_OK("Tunneled MC Cache entry %d cleared\n", i);
	}
}

static void mv_dp_mc_br_clear(int i)
{

	if (!br_allocated) {
		MV_DP_CLI_FAIL("Not allocated\n", MV_NSS_DP_FAILED);
		return;
	}

	if (i == -1) {
		/*clear all*/
		memset(br_mc, 0, sizeof(mv_nss_dp_mc_bridged_cfg_t) * br_allocated);
		MV_DP_CLI_OK("Bridged MC Cache Cleared: %d\n", br_allocated);
	} else if (!MV_DP_SYSFS_MC_BR_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Bridged MC Cache index:%d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	} else {
		memset(&br_mc[i], 0, sizeof(mv_nss_dp_mc_bridged_cfg_t));
		MV_DP_CLI_OK("Bridged MC Cache entry %d cleared\n", i);
	}
}

static void mv_dp_mc_read(int i, int index, enum mv_dp_sysfs_mc_type t)
{
	switch (t) {

	case MV_DP_SYSFS_MC_BRIDGED:
		mv_dp_mc_br_read(i, index);
		break;
	case MV_DP_SYSFS_MC_TUNNELED:
		mv_dp_mc_tn_read(i, index);
		break;

	default:
		MV_DP_CLI_FAIL("Illegal MC TYPE:%d\n", MV_NSS_DP_INVALID_PARAM, t);
	}
}


static void mv_dp_mc_tn_read(int i, int index)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;
	mv_nss_dp_status_t	rc;

	/*index validation is to be performed in dpapi*/

	if (!MV_DP_SYSFS_MC_TN_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Tunneled MC index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	/*the index is stored in xid*/
	res.cb = mv_dp_mc_read_cb;
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

	rc = mv_nss_dp_mc_tunneled_cfg_get((uint16_t)index, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Tunneled MC index:%d to cache:%d Status:%s\n", rc, index, i, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get Tunneled MC index:%d to Cache:%d\n", index, i);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Tunneled Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
		kfree(res.cookie);
	}
err_free:
	if (res.cookie)
		kfree(res.cookie);
}


static void mv_dp_mc_br_read(int i, int index)
{

	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;
	mv_nss_dp_status_t	rc;

	/*index validation is to be performed in dpapi*/

	if (!MV_DP_SYSFS_MC_BR_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Bridged MC index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	/*the index is stored in xid*/
	res.cb = mv_dp_mc_read_cb;
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

	rc = mv_nss_dp_mc_bridged_cfg_get((uint16_t)index, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Bridged MC index:%d to cache:%d Status:%s\n", rc, index, i, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get Bridged MC index:%d to Cache:%d\n", index, i);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Bridged Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
		kfree(res.cookie);
	}
err_free:
	if (res.cookie)
		kfree(res.cookie);
}



static void mv_dp_mc_get(int index, enum mv_dp_sysfs_mc_type t)
{
	switch (t) {

	case MV_DP_SYSFS_MC_BRIDGED:
		mv_dp_mc_br_get(index);
		break;
	case MV_DP_SYSFS_MC_TUNNELED:
		mv_dp_mc_tn_get(index);
		break;

	default:
		MV_DP_CLI_FAIL("Illegal MC TYPE:%d\n", MV_NSS_DP_INVALID_PARAM, t);
	}
}


static void mv_dp_mc_tn_get(int index)
{

	mv_nss_dp_result_spec_t	res;
	struct completion	*compl_ptr;
	mv_nss_dp_status_t	rc;

	/*the index is stored in xid*/
	res.cb = mv_dp_mc_get_cb;
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

	rc = mv_nss_dp_mc_tunneled_cfg_get((uint16_t)index, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Tunneled MC index: %d Status: %s\n", rc, index, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get Tunneled MC index %d\n", index);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Tunneled MC Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
		kfree(res.cookie);
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);

}

static void mv_dp_mc_br_get(int index)
{

	mv_nss_dp_result_spec_t	res;
	struct completion	*compl_ptr;
	mv_nss_dp_status_t	rc;

	/*the index is stored in xid*/
	res.cb = mv_dp_mc_get_cb;
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

	rc = mv_nss_dp_mc_bridged_cfg_get((uint16_t)index, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Bridged MC index: %d Status: %s\n", rc, index, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Get Bridged MC index %d\n", index);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Bridged MC Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
		kfree(res.cookie);
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);

}



static void mv_dp_mc_alloc(int n, enum mv_dp_sysfs_mc_type t)
{
	switch (t) {

	case MV_DP_SYSFS_MC_BRIDGED:
		if (br_allocated) {
			MV_DP_CLI_FAIL("Already allocated: %d\n", MV_NSS_DP_FAILED, br_allocated);
			return;
		}
		if (!MV_DP_SYSFS_MC_MAX_NUM_OK(n)) {
			MV_DP_CLI_FAIL("Illegal Bridged MC Record number: %d\n", MV_NSS_DP_INVALID_PARAM, n);
			return;
		}
		br_mc = kzalloc(sizeof(mv_nss_dp_mc_bridged_cfg_t) * n, GFP_KERNEL);
		if (!br_mc) {
			MV_DP_CLI_FAIL("Cache alloc failed for size:%d\n", MV_NSS_DP_OUT_OF_RESOURCES, n);
			return;
		}
		br_allocated = n;
		MV_DP_CLI_OK("Allocated: %d\n", br_allocated);
		break;

	case MV_DP_SYSFS_MC_TUNNELED:
		if (tn_allocated) {
			MV_DP_CLI_FAIL("Already allocated: %d\n", MV_NSS_DP_FAILED, tn_allocated);
			return;
		}
		if (!MV_DP_SYSFS_MC_MAX_NUM_OK(n)) {
			MV_DP_CLI_FAIL("Illegal Tunneled MC Record number: %d\n", MV_NSS_DP_INVALID_PARAM, n);
			return;
		}
		tn_mc = kzalloc(sizeof(mv_nss_dp_mc_tunneled_cfg_t) * n, GFP_KERNEL);
		if (!tn_mc) {
			MV_DP_CLI_FAIL("Cache alloc failed for size:%d\n", MV_NSS_DP_OUT_OF_RESOURCES, n);
			return;
		}
		tn_allocated = n;
		MV_DP_CLI_OK("Allocated: %d\n", tn_allocated);
		break;

	default:
		MV_DP_CLI_FAIL("Illegal MC TYPE:%d\n", MV_NSS_DP_INVALID_PARAM, t);

	}
}

static void mv_dp_mc_delete(int i, enum mv_dp_sysfs_mc_type t)
{

	switch (t) {

	case MV_DP_SYSFS_MC_BRIDGED:
		mv_dp_mc_br_delete(i);
		break;
	case MV_DP_SYSFS_MC_TUNNELED:
		mv_dp_mc_tn_delete(i);
		break;

	default:
		MV_DP_CLI_FAIL("Illegal MC TYPE:%d\n", MV_NSS_DP_INVALID_PARAM, t);

	}
}

static void mv_dp_mc_br_delete(int i)
{

	mv_nss_dp_result_spec_t		res;
	int				tmp_count;
	mv_nss_dp_mc_bridged_cfg_t	*mc_ptr;
	struct completion		*compl_ptr;
	mv_nss_dp_status_t		rc;

	if (i == -1) {
		/*commit all*/
		tmp_count = br_allocated;
		mc_ptr = br_mc;
	} else if (!MV_DP_SYSFS_MC_BR_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Bridged MC index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	} else	{
		tmp_count = 1;
		mc_ptr = &br_mc[i];
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
	res.cb = mv_dp_mc_delete_cb;
	res.xid = i;

	rc = mv_nss_dp_mc_bridged_cfg_delete(mc_ptr, tmp_count, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Delete Bridged MC Index: %d Status: %s\n", rc, i, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Delete Tunneled MC Index: %d, Count: %d\n", i, tmp_count);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Bridged MC Delete: %d Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       res.xid, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);
}


static void mv_dp_mc_tn_delete(int i)
{

	mv_nss_dp_result_spec_t		res;
	int				tmp_count;
	uint16_t			*mgid;
	int				start, end;
	struct completion		*compl_ptr;
	mv_nss_dp_status_t		rc;

	if (i == -1) {
		/*commit all*/
		tmp_count = tn_allocated;
		start = 0;
		end = tn_allocated;

	} else if (!MV_DP_SYSFS_MC_TN_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Tunneled MC index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	} else	{
		tmp_count = 1;
		start = i;
		end = start + 1;
	}

	mgid = kmalloc(sizeof(uint16_t) * tmp_count, GFP_KERNEL);
	if (!mgid) {
		MV_DP_CLI_FAIL("MGIDs Alloc failed: %d\n", MV_NSS_DP_OUT_OF_RESOURCES, i);
		return;
	}

	for (i = start; i < end; i++)
		mgid[i] = tn_mc[i].mgid;

	if (mv_dp_sysfs_delay_get()) {
		compl_ptr = kmalloc(sizeof(struct completion), GFP_KERNEL);
		if (!compl_ptr) {
			MV_DP_CLI_FAIL("Completion allocation Failed\n", MV_NSS_DP_OUT_OF_RESOURCES);
			goto err_mgid;
		}
		init_completion(compl_ptr);
		res.cookie = compl_ptr;
	} else {
		res.cookie = 0;
	}

	/*the index is stored in xid*/
	res.cb = mv_dp_mc_delete_cb;
	res.xid = i;

	rc = mv_nss_dp_mc_tunneled_cfg_delete(mgid, tmp_count, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Delete Tunneled MC Index: %d Status: %s\n", rc, i, mv_dp_err_name_get(rc));
		goto err_free;
	}

	MV_DP_CLI_TMSG("Delete Tunneled MC Index: %d, Count: %d\n", i, tmp_count);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Tunneled MC Delete: %d Status: %s\n", MV_NSS_DP_API_EXEC_TIMEOUT,
				       res.xid, mv_dp_err_name_get(MV_NSS_DP_API_EXEC_TIMEOUT));
	}

err_free:
	if (res.cookie)
		kfree(res.cookie);
err_mgid:
	kfree(mgid);
}




static void mv_dp_mc_br_l2_set(int i, unsigned char *mac)
{
	if (!MV_DP_SYSFS_MC_BR_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Bridged MC index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (!mac) {
		MV_DP_CLI_FAIL("Null MAC ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	memcpy(br_mc[i].l2_addr.addr, mac, sizeof(br_mc[i].l2_addr.addr));
	MV_DP_CLI_OK("Set Bridged MC:%d MAC:%02X:%02X:%02X:%02X:%02X:%02X\n", i,
		     br_mc[i].l2_addr.addr[0],
		     br_mc[i].l2_addr.addr[1],
		     br_mc[i].l2_addr.addr[2],
		     br_mc[i].l2_addr.addr[3],
		     br_mc[i].l2_addr.addr[4],
		     br_mc[i].l2_addr.addr[5]);

}

static void mv_dp_mc_br_od_set(int i, u32 od)
{
	if (!MV_DP_SYSFS_MC_BR_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Bridged MC index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	br_mc[i].opaque = od;

	MV_DP_CLI_OK("Set Bridged MC:%d Opaque:0x%08X\n", i, br_mc[i].opaque);

}

static void mv_dp_mc_br_vid_set(int i, u16 vid)
{
	if (!MV_DP_SYSFS_MC_BR_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Bridged MC index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	br_mc[i].vlan_id = vid;

	MV_DP_CLI_OK("Set Bridged MC:%d vlan id:%d\n", i, br_mc[i].vlan_id);

}


static void mv_dp_mc_tn_mgid_set(int i, u16 mgid)
{
	if (!MV_DP_SYSFS_MC_TN_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Tunneled MC index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	tn_mc[i].mgid = mgid;

	MV_DP_CLI_OK("Set Tunneled MC:%d mgid:%d\n", i, tn_mc[i].mgid);

}

static void mv_dp_mc_tn_ingr_policy_id_set(int i, int pol)
{
	if (!MV_DP_SYSFS_MC_TN_NUM_OK(i)) {
		MV_DP_CLI_FAIL("Illegal Cache Tunneled MC index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	tn_mc[i].policy_id = pol;

	MV_DP_CLI_OK("Set Tunneled MC:%d ingress policy:%d\n", i, tn_mc[i].policy_id);

}


void mv_dp_mc_set_cb(mv_nss_dp_event_t *event)
{

	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (!(MV_NSS_DP_OK == event->status || MV_NSS_DP_END_OF_LIST == event->status)) {
		MV_DP_CLI_FAIL_CB("Set %s MC Index: %d Count: %d Status: %s\n", event,
				  event->type == MV_NSS_DP_EVT_MC_BRIDGED_CFG_SET ? "Bridged" : "Tunneled",
				  event->xid, event->count,
				  mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Set %s MC Index: %d Count:%d\n", event,
			event->type == MV_NSS_DP_EVT_MC_BRIDGED_CFG_SET ? "Bridged" : "Tunneled",
			event->xid, event->count);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

void mv_dp_mc_get_cb(mv_nss_dp_event_t *event)
{

	/*Print response and return OK;*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get %s MC Index: %d Status: %s\n", event,
				  (event->type == MV_NSS_DP_EVT_MC_BRIDGED_CFG_SET) ? "Bridged" : "Tunneled",
				  event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	switch (event->type) {
	case MV_NSS_DP_EVT_MC_BRIDGED_CFG_GET:
		if (!event->params.mc_bridged_cfg) {
			MV_DP_CLI_FAIL_CB("Get Bridged MC index: %d Empty VLAN\n", event, event->xid);
			goto err;
		}
		MV_DP_CLI_OK_CB("Get Bridged Index: %d\n", event, event->xid);
		MV_DP_CLI_CONT("|MAC         | OPAQUE |VLAN|<\n");
		mv_dp_mc_br_show_single(event->params.mc_bridged_cfg);
		MV_DP_CLI_CONT("|BRIDGED MC END|<\n");
		break;

	case MV_NSS_DP_EVT_MC_TUNNELED_CFG_GET:
		if (!event->params.mc_tunneled_cfg) {
			MV_DP_CLI_FAIL_CB("Get Tunneled MC index: %d Empty VLAN\n", event, event->xid);
			goto err;
		}
		MV_DP_CLI_OK_CB("Get Tunneled Index: %d\n", event, event->xid);
		MV_DP_CLI_CONT("|MGID|POLICY|<\n");
		mv_dp_mc_tn_show_single(event->params.mc_tunneled_cfg);
		MV_DP_CLI_CONT("|TUNNELED MC END|<\n");
		break;

	default:
		MV_DP_CLI_FAIL_CB("Get MC Index: %d Unexpected Type:d\n", event, event->type);
	}

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);
}

void mv_dp_mc_delete_cb(mv_nss_dp_event_t *event)
{

	/*save response in the port buffer and return OK;*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (!(MV_NSS_DP_OK == event->status || MV_NSS_DP_END_OF_LIST == event->status)) {
				MV_DP_CLI_FAIL_CB("Delete MC ID: %d Count: %d Status: %s\n", event,
				  event->xid, event->count, mv_dp_err_name_get(event->status));
		goto err;
	}


	switch (event->type) {
	case MV_NSS_DP_EVT_MC_BRIDGED_CFG_DELETE:
		MV_DP_CLI_OK_CB("Deleted Bridged MC Index:%d Count:%d\n", event, event->xid, event->count);
		break;

	case MV_NSS_DP_EVT_MC_TUNNELED_CFG_DELETE:
		MV_DP_CLI_OK_CB("Deleted Tunneled MC MGID:%d Count:%d\n", event, event->xid, event->count);
		break;
	default:
		MV_DP_CLI_FAIL_CB("Delete MC Index: %d Unexpected Type:d\n", event, event->type);
	}

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

void mv_dp_mc_read_cb(mv_nss_dp_event_t *event)
{

	/*the index is stored in xid*/
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Read MC to Cache Index: %d Status: %s\n", event,
				  event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	switch (event->type) {
	case MV_NSS_DP_EVT_MC_BRIDGED_CFG_GET:
		if (!event->params.mc_bridged_cfg) {
			MV_DP_CLI_FAIL_CB("Read Bridged MC to Cache Index: %d Empty MC\n", event, event->xid);
			goto err;
		}

		if (!MV_DP_SYSFS_MC_BR_NUM_OK(event->xid)) {
			MV_DP_CLI_FAIL_CB("Illegal Cache Bridged MC index: %d\n", event, event->xid);
			goto err;
		}

		memcpy(&br_mc[event->xid], event->params.mc_bridged_cfg, sizeof(mv_nss_dp_mc_bridged_cfg_t));
		MV_DP_CLI_OK_CB("Read Bridged MC to Cache Index: %d\n", event, event->xid);
		break;

	case MV_NSS_DP_EVT_MC_TUNNELED_CFG_GET:
		if (!event->params.mc_tunneled_cfg) {
			MV_DP_CLI_FAIL_CB("Read Tunneled MC to Cache Index: %d Empty MC\n", event, event->xid);
			goto err;
		}

		if (!MV_DP_SYSFS_MC_TN_NUM_OK(event->xid)) {
			MV_DP_CLI_FAIL_CB("Illegal Cache Tunneled MC index: %d\n", event, event->xid);
			goto err;
		}

		memcpy(&tn_mc[event->xid], event->params.mc_tunneled_cfg, sizeof(mv_nss_dp_mc_tunneled_cfg_t));
		MV_DP_CLI_OK_CB("Read Tunneled MC to Cache Index: %d\n", event, event->xid);
		break;
	default:
		MV_DP_CLI_FAIL_CB("Delete MC Index: %d Unexpected Type:d\n", event, event->type);
	}

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}

