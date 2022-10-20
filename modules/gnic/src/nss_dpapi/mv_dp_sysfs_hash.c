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
#include "mv_dp_int_if.h"
#include "mv_dp_main.h"



/* Hash Profile FW commands */
static void mv_dp_hash_prof_set(u32 *params);
static void mv_dp_hash_prof_del(u32 id);
static void mv_dp_hash_prof_get(u32 id);

/* Hash callbacks */
static void mv_dp_hash_prof_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_hash_prof_del_cb(mv_nss_dp_event_t *event);
void mv_dp_hash_prof_get_cb(mv_nss_dp_event_t *event);

static void mv_dp_hash_prof_show_entry(mv_nss_dp_hash_profile_t *entry);


static ssize_t mv_dp_hash_sysfs_help(char *b)
{
	int o = 0;

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                                           help          - Show Help\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID] [TU] [IP] [NIP] [0] [1] [2]       > hash_prof_set - Set Hash profile [ID]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]                                   > hash_prof_del - Delete Hash profile [ID]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [ID]                                   > hash_prof_get - Get Hash and show profile [ID]\n");


	o += scnprintf(b + o, PAGE_SIZE - o,
		     "\n         [ID]  = Hash profile ID (decimal 32b unsigned)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "         [TU]  = Fields Bitmask used for TCP/UDP/UDP-Lite packets Hash Calculation (hex 32b)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "         [IP]  = Fields Bitmask used for IP Hash packets Calculation (hex 32b)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "         [NIP] = Fields Bitmask used for non IP Hash packets Calculation (hex 32b)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "         [0]   = Output Value for Hash Value 0 (hex 32b)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "         [1]   = Output Value for Hash Value 1 (hex 32b)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "         [2]   = Output Value for Hash Value 2 (hex 32b)\n");

	return o;
}

static ssize_t mv_dp_hash_sysfs_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int off = 0;
	const char  *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "help")) {
		return mv_dp_hash_sysfs_help(buf);
	} else {
		MV_DP_CLI_FAIL("Parse Error CMD: <%s>\n", MV_NSS_DP_INVALID_PARAM, name);
		off = mv_dp_hash_sysfs_help(buf);
	}
	return off;
}

static ssize_t mv_dp_hash_sysfs_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t len)
{
	const char	*name = attr->attr.name;
	u32		arr[8] = {0};


	if (!capable(CAP_NET_ADMIN))
		return -EPERM;


	if (!strcmp(name, "hash_prof_set")) {
		if (7 != sscanf(buf, "%u 0x%X 0x%X 0x%X %i %i %i",
				&arr[0], &arr[1], &arr[2], &arr[3], &arr[4], &arr[5], &arr[6]))
			goto err;
		mv_dp_hash_prof_set(arr);
	} else if (!strcmp(name, "hash_prof_del")) {
		if (1 != sscanf(buf, "%u", &arr[0]))
			goto err;
		mv_dp_hash_prof_del(arr[0]);
	} else if (!strcmp(name, "hash_prof_get")) {
		if (1 != sscanf(buf, "%u", &arr[0]))
			goto err;
		mv_dp_hash_prof_get(arr[0]);
	} else {
		goto err;
	}

	return len;

err:
	MV_DP_CLI_FAIL("Parse Error CMD:<%s>\n", MV_NSS_DP_INVALID_PARAM, attr->attr.name);
	return -EINVAL;
}

static DEVICE_ATTR(help,				S_IRUSR, mv_dp_hash_sysfs_show, NULL);
static DEVICE_ATTR(hash_prof_set,			S_IWUSR, NULL, mv_dp_hash_sysfs_store);
static DEVICE_ATTR(hash_prof_del,			S_IWUSR, NULL, mv_dp_hash_sysfs_store);
static DEVICE_ATTR(hash_prof_get,			S_IWUSR, NULL, mv_dp_hash_sysfs_store);


static struct attribute *mv_dp_hash_sysfs_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_hash_prof_set.attr,
	&dev_attr_hash_prof_del.attr,
	&dev_attr_hash_prof_get.attr,

	NULL
};


static struct attribute_group mv_dp_hash_sysfs_group = {
	.name = "hash",
	.attrs = mv_dp_hash_sysfs_attrs,
};



int mv_dp_hash_sysfs_init(struct kobject *ko)
{
	int err;
	err = sysfs_create_group(ko, &mv_dp_hash_sysfs_group);

	if (err) {
		MV_DP_LOG_INF("Hash sysFS group init failed %d\n", err);
		return err;
	}
	MV_DP_LOG_DBG1("Hash sysFS INITALIZED\n");
	return err;
}

int mv_dp_hash_sysfs_exit(struct kobject *ko)
{
	sysfs_remove_group(ko, &mv_dp_hash_sysfs_group);
	return 0;
}


static void mv_dp_hash_prof_set(u32 *arr)
{
	mv_nss_dp_result_spec_t		res;
	mv_nss_dp_hash_profile_t	prof;
	struct completion		*compl_ptr;
	mv_nss_dp_status_t		rc;

	if (!arr) {
		MV_DP_CLI_FAIL("Null hash set parameters\n", MV_DP_RC_ERR_NULL_PTR);
		return;
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

	res.cb = mv_dp_hash_prof_set_cb;
	res.xid = arr[0];

	prof.profile_id = arr[0];
	prof.tcp_udp = arr[1];
	prof.ip = arr[2];
	prof.non_ip = arr[3];
	prof.result[0] = arr[4];
	prof.result[1] = arr[5];
	prof.result[2] = arr[6];

	rc = mv_nss_dp_hash_profile_set(&prof, &res);

	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Set Hash Profile\n", rc);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Set Hash profile id: %u\n", arr[0]);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Set Hash Profile - Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}

static void mv_dp_hash_prof_del(uint prof_id)
{

	mv_nss_dp_result_spec_t		res;
	struct completion		*compl_ptr;
	mv_nss_dp_status_t		rc;

	/*the index is stored in xid*/
	res.cb = mv_dp_hash_prof_del_cb;
	res.xid = prof_id;

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

	rc = mv_nss_dp_hash_profile_delete((mv_nss_dp_hash_profile_id_t)prof_id, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Delete Hash profile: %d\n", rc, prof_id);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Delete Hash profile: %d\n", prof_id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Delete Hash Profile: %d - Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT, prof_id);
		kfree(res.cookie);
	}
}

static void mv_dp_hash_prof_get(uint prof_id)
{

	mv_nss_dp_result_spec_t		res;
	struct completion		*compl_ptr;
	mv_nss_dp_status_t		rc;

	/*the index is stored in xid*/
	res.cb = mv_dp_hash_prof_get_cb;
	res.xid = prof_id;

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

	rc = mv_nss_dp_hash_profile_get((mv_nss_dp_hash_profile_id_t)prof_id, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Get Hash profile: %d\n", rc, prof_id);
		if (res.cookie)
			kfree(res.cookie);
		return;
	}

	MV_DP_CLI_TMSG("Get Hash profile: %d\n", prof_id);

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Hash Profile: %d - Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT, prof_id);
		kfree(res.cookie);
	}
}


static void mv_dp_hash_prof_show_entry(mv_nss_dp_hash_profile_t *entry)
{

	if (!entry) {
		MV_DP_LOG_ERR("Null Egress Q\n", MV_DP_RC_ERR_NULL_PTR);
		return;
	}

	MV_DP_LOG_CONT("|%8u|0x%08X|0x%08X|0x%08X|0x%08X|0x%08X|0x%08X|<\n",
		       entry->profile_id,
		       entry->tcp_udp,
		       entry->ip,
		       entry->non_ip,
		       entry->result[0],
		       entry->result[1],
		       entry->result[2]
		       );
}



void mv_dp_hash_prof_set_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Set Hash profile: %d Status: %s\n", event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("Set Hash profile: %d\n", event, event->xid);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);

}


void mv_dp_hash_prof_del_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Delete Hash profile: %d Status: %s\n", event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("Delete Hash profile: %d\n", event, event->xid);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);
}

void mv_dp_hash_prof_get_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get Hash profile: %d Status: %s\n", event, event->xid, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Get HASH Prof Index: %d\n", event, event->xid);
	MV_DP_CLI_CONT("|PROF  ID|TCP    UDP|    IP    |NON     IP| KNEE   0 | KNEE   1 | KNEE   2 |<\n");
	mv_dp_hash_prof_show_entry(event->params.hash_prof);
	MV_DP_CLI_CONT("|HASH Prof END|<\n");


err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);
}
