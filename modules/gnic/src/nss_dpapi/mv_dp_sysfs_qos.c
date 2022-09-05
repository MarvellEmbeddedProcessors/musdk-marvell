
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


#define MV_DP_SYSFS_QOS_POLICY_MAX	(64)

#define MV_DP_SYSFS_EGRESS_QOS_OK(n)	((n) >= 0 && (n) < egr_allocated)

static  mv_nss_dp_egress_qos_policy_t	*egr_policy;
static  int				egr_allocated; /*number of allocated entries*/

#define MV_DP_SYSFS_INGRESS_QOS_OK(n)	((n) >= 0 && (n) < ingr_allocated)

static  mv_nss_dp_ingress_qos_policy_t	*ingr_policy;
static  int				ingr_allocated; /*number of allocated entries*/



enum mv_dp_sys_egr_pol_type {
	EGR_POL_DSCP,
	EGR_POL_UP,
	EGR_POL_PCP,
	EGR_POL_LAST
};

#define MV_DP_SYS_EGR_POL_TYPE_IS_OK(v) ((v) >= EGR_POL_DSCP && (v) <= EGR_POL_LAST)
static char *egr_qos_names[EGR_POL_LAST] = {
	"DSCP",
	"UP",
	"PCP"
	};

#define MV_DP_SYS_INGR_QOS_POL_TYPE_IS_OK(v) ((v) >= MV_NSS_DP_INGRESS_QOS_POLICY_L2_L3 && \
					     (v) <= MV_NSS_DP_INGRESS_QOS_POLICY_L3)
static char *ingr_qos_names[MV_NSS_DP_INGRESS_QOS_POLICY_L3 + 1] = {
	"L2_L3",
	"L3_L2",
	"L2",
	"L3"
	};

static void mv_dp_qos_policy_get(enum mv_dp_sys_dir_type, int id);
static void mv_dp_qos_policy_read(int ind, enum mv_dp_sys_dir_type, int pol_id);

static void mv_dp_qos_policy_show(int ind, enum mv_dp_sys_dir_type);
static void mv_dp_egr_qos_policy_show(int ind);
static void mv_dp_ingr_qos_policy_show(int ind);
static void mv_dp_egr_qos_policy_show_sinlge(mv_nss_dp_egress_qos_policy_t *policy);
static void mv_dp_ingr_qos_policy_show_sinlge(mv_nss_dp_ingress_qos_policy_t *policy);

static void mv_dp_qos_policy_commit(int ind, enum mv_dp_sys_dir_type);
static void mv_dp_ingr_qos_policy_commit(int id);
static void mv_dp_egr_qos_policy_commit(int id);

static void mv_dp_qos_policy_alloc(int num, enum mv_dp_sys_dir_type);
static void mv_dp_qos_policy_release(enum mv_dp_sys_dir_type);
static void mv_dp_qos_policy_clear(int num, enum mv_dp_sys_dir_type);

static void mv_dp_qos_policy_id_set(int ind, enum mv_dp_sys_dir_type, u8 id);

static void mv_dp_ingr_qos_policy_type_set(int ind, int type);
static void mv_dp_ingr_qos_policy_prio_def_set(int ind, u8 prio);
static void mv_dp_ingr_qos_policy_l2_set(int ind, int *prio);
static void mv_dp_ingr_qos_policy_dscp_set(int ind, u8 x, int *prio); /*x is the 2 MSB or x16*/

static void mv_dp_egr_qos_policy_set(int ind, enum mv_dp_sys_egr_pol_type t, int *prio);


/**************CB************************************************/
static void mv_dp_ingr_qos_policy_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_ingr_qos_policy_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_ingr_qos_policy_read_cb(mv_nss_dp_event_t *event);

static void mv_dp_egr_qos_policy_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_egr_qos_policy_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_egr_qos_policy_read_cb(mv_nss_dp_event_t *event);



static ssize_t mv_dp_qos_sysfs_help(char *b)
{
	int o = 0;

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                   help     - Show Help\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [t]            > qos_policy_release - Release QOS Policy of type[t] Cache\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t]         > qos_policy_clear - Clear QOS policy type[t] Cache [i] to 0\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t][id]     > qos_policy_read  - Read FW QOS policy id[id] of type[t] to cache[i]\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [t][id]        > qos_policy_get   - Get FW QOS policy id[id] of type[t] and print it\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t]         > qos_policy_commit- Save QOS policy of type[t] stored in cache [i] to FW\n");


	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [n][t]         > qos_policy_alloc - Allocate QOS policy cache type[t] of [n] entries\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t]         > qos_policy_show  - Show QOS policy type[t] cache entry [i] or -1 for ALL\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t][id]     > qos_policy_id_set- Set Cached QOS Policy[i] of type[t] policy_id to[id]\n"
		       "                                        (dec 8 bit)\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][it]        > ingr_qos_policy_type_set- Set Cached Ingress QOS Policy[i] type to[it]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][pr]        > ingr_qos_policy_prio_def_set - Set Cached Ingress QOS Policy[i]\n"
		       "                       default fixed prio to[pr] (0 - 15)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][0]..[7]    > ingr_qos_policy_l2_set - Set Cached Ingress QOS Policy[i]'s L2 to Prio map\n"
		       "                      where [0]..[7] - 8 [pr]iority values for each of 3 bits L2 802.1Q bits\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][x16][0]..[15]>ingr_qos_policy_dscp_set- Set Cached Ingress QOS Policy[n]'s DSCP to Prio\n"
		       "                      where [0]..[15] - 16 pr values for each of 4 LSB DSCP bits, MSB are in[x16]\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][et][0]..[15]> egr_qos_policy_set - Set Cached Egress QOS Policy[i]'s Prio to CoS bits map\n"
		       "                      where [0]..[15] - 16 values(3 or 6 bits) for each of 16 Priorities\n");


	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [it]= ingress qos policy type: %d=%s;%d=%s;%d=%s;%d=%s",
		       MV_NSS_DP_INGRESS_QOS_POLICY_L2_L3, ingr_qos_names[MV_NSS_DP_INGRESS_QOS_POLICY_L2_L3],
		       MV_NSS_DP_INGRESS_QOS_POLICY_L3_L2, ingr_qos_names[MV_NSS_DP_INGRESS_QOS_POLICY_L3_L2],
		       MV_NSS_DP_INGRESS_QOS_POLICY_L2, ingr_qos_names[MV_NSS_DP_INGRESS_QOS_POLICY_L2],
		       MV_NSS_DP_INGRESS_QOS_POLICY_L3, ingr_qos_names[MV_NSS_DP_INGRESS_QOS_POLICY_L3]
		       );
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [et]= egress qos policy type:%d=%s;%d=%s;%d=%s",
						EGR_POL_DSCP, egr_qos_names[EGR_POL_DSCP],
						EGR_POL_UP, egr_qos_names[EGR_POL_UP],
						EGR_POL_PCP, egr_qos_names[EGR_POL_PCP]
						);

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [t]= qos/queue direction type:%d=INGRESS; %d=EGRESS",
		       QOS_DIR_TYPE_INGRESS, QOS_DIR_TYPE_EGRESS);

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   dec - decimal 123; hex - hexadecimal (0x000)\n");

	return o;
}

static ssize_t mv_dp_qos_sysfs_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int off = 0;
	const char  *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "help")) {
		return mv_dp_qos_sysfs_help(buf);
	} else {
		MV_DP_CLI_FAIL("Parse Error CMD: <%s>\n", MV_NSS_DP_INVALID_PARAM, name);
		off = mv_dp_qos_sysfs_help(buf);
	}
	return off;
}

static ssize_t mv_dp_qos_sysfs_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t len)
{
	const char *name = attr->attr.name;
	unsigned int    a, b, c, d;

	int arr[MV_NSS_DP_PRIO_NUM];


	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = b = c = d = 0;

	if (!strcmp(name, "egr_qos_policy_set")) {
		if (18 != sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
		       &a, &b, &arr[0], &arr[1], &arr[2], &arr[3], &arr[4], &arr[5], &arr[6], &arr[7],
		       &arr[8], &arr[9], &arr[10], &arr[11], &arr[12], &arr[13], &arr[14], &arr[15]))
			goto err;
		mv_dp_egr_qos_policy_set(a, b, arr);
	} else	if (!strcmp(name, "ingr_qos_policy_l2_set")) {
		if (9 != sscanf(buf, "%d %d %d %d %d %d %d %d %d",
		       &a, &arr[0], &arr[1], &arr[2], &arr[3], &arr[4], &arr[5], &arr[6], &arr[7]))
			goto err;
		mv_dp_ingr_qos_policy_l2_set(a, arr);
	} else	if (!strcmp(name, "ingr_qos_policy_dscp_set")) {
		if (18 != sscanf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
		       &a, &b, &arr[0], &arr[1], &arr[2], &arr[3], &arr[4], &arr[5], &arr[6], &arr[7],
		       &arr[8], &arr[9], &arr[10], &arr[11], &arr[12], &arr[13], &arr[14], &arr[15]))
			goto err;
		mv_dp_ingr_qos_policy_dscp_set(a, b, arr);
	} else	if (!strcmp(name, "qos_policy_alloc")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_qos_policy_alloc(a, b);
	} else	if (!strcmp(name, "qos_policy_release")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_qos_policy_release(a);
	} else	if (!strcmp(name, "qos_policy_get")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_qos_policy_get(a, b);
	} else	if (!strcmp(name, "qos_policy_read")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_qos_policy_read(a, b, c);
	} else	if (!strcmp(name, "qos_policy_commit")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_qos_policy_commit(a, b);
	} else	if (!strcmp(name, "qos_policy_clear")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_qos_policy_clear(a, b);
	} else	if (!strcmp(name, "ingr_qos_policy_type_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_ingr_qos_policy_type_set(a, b);
	} else	if (!strcmp(name, "ingr_qos_policy_prio_def_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_ingr_qos_policy_prio_def_set(a, b);
	} else	if (!strcmp(name, "qos_policy_id_set")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_qos_policy_id_set(a, b, c);
	} else	if (!strcmp(name, "qos_policy_show")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_qos_policy_show(a, b);

	} else {
		goto err;
	}

	return len;
err:
	MV_DP_LOG_INF("Parse Error CMD:<%s>\n", attr->attr.name);
	return -EINVAL;
}

static DEVICE_ATTR(help,			S_IRUSR, mv_dp_qos_sysfs_show, NULL);

static DEVICE_ATTR(qos_policy_alloc,		S_IWUSR, NULL, mv_dp_qos_sysfs_store);
static DEVICE_ATTR(qos_policy_release,		S_IWUSR, NULL, mv_dp_qos_sysfs_store);
static DEVICE_ATTR(qos_policy_get,		S_IWUSR, NULL, mv_dp_qos_sysfs_store);
static DEVICE_ATTR(qos_policy_read,		S_IWUSR, NULL, mv_dp_qos_sysfs_store);
static DEVICE_ATTR(qos_policy_clear,		S_IWUSR, NULL, mv_dp_qos_sysfs_store);
static DEVICE_ATTR(qos_policy_commit,		S_IWUSR, NULL, mv_dp_qos_sysfs_store);
static DEVICE_ATTR(ingr_qos_policy_l2_set,	S_IWUSR, NULL, mv_dp_qos_sysfs_store);
static DEVICE_ATTR(ingr_qos_policy_dscp_set,	S_IWUSR, NULL, mv_dp_qos_sysfs_store);
static DEVICE_ATTR(egr_qos_policy_set,		S_IWUSR, NULL, mv_dp_qos_sysfs_store);
static DEVICE_ATTR(ingr_qos_policy_type_set,	S_IWUSR, NULL, mv_dp_qos_sysfs_store);
static DEVICE_ATTR(ingr_qos_policy_prio_def_set, S_IWUSR, NULL, mv_dp_qos_sysfs_store);
static DEVICE_ATTR(qos_policy_id_set,		S_IWUSR, NULL, mv_dp_qos_sysfs_store);
static DEVICE_ATTR(qos_policy_show,		S_IWUSR, NULL, mv_dp_qos_sysfs_store);


static struct attribute *mv_dp_qos_sysfs_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_qos_policy_release.attr,
	&dev_attr_qos_policy_clear.attr,
	&dev_attr_qos_policy_read.attr,
	&dev_attr_qos_policy_get.attr,
	&dev_attr_qos_policy_show.attr,
	&dev_attr_qos_policy_commit.attr,
	&dev_attr_qos_policy_alloc.attr,
	&dev_attr_qos_policy_id_set.attr,
	&dev_attr_ingr_qos_policy_type_set.attr,
	&dev_attr_ingr_qos_policy_prio_def_set.attr,
	&dev_attr_ingr_qos_policy_l2_set.attr,
	&dev_attr_ingr_qos_policy_dscp_set.attr,
	&dev_attr_egr_qos_policy_set.attr,

	NULL
};


static struct attribute_group mv_dp_qos_sysfs_group = {
	.name = "qos",
	.attrs = mv_dp_qos_sysfs_attrs,
};



int mv_dp_qos_sysfs_init(struct kobject *ko)
{
	int err;
	err = sysfs_create_group(ko, &mv_dp_qos_sysfs_group);

	if (err) {
		MV_DP_LOG_INF("QOS sysFS group init failed %d\n", err);
		return err;
	}

	MV_DP_LOG_DBG1("QOS sysFS INITALIZED\n");
	return err;
}

int mv_dp_qos_sysfs_exit(struct kobject *ko)
{
	sysfs_remove_group(ko, &mv_dp_qos_sysfs_group);
	if (ingr_allocated) {
		kfree(ingr_policy);
		ingr_allocated = 0;
	}

	if (egr_allocated) {
		kfree(egr_policy);
		egr_allocated = 0;
	}

	return 0;
}


static void mv_dp_qos_policy_alloc(int num, enum mv_dp_sys_dir_type t)
{
	if (num < 0 || num > MV_DP_SYSFS_QOS_POLICY_MAX) {
			MV_DP_CLI_FAIL("Illegal QOS Policy Records number: %d\n", MV_NSS_DP_INVALID_PARAM, num);
			return;
	}

	if (t == QOS_DIR_TYPE_INGRESS) {
		if (ingr_allocated) {
			MV_DP_CLI_FAIL("INGR QOS Policy Already allocated: %d\n", MV_NSS_DP_FAILED, ingr_allocated);
			return;
		}

		ingr_policy = kzalloc(sizeof(mv_nss_dp_ingress_qos_policy_t) * num, GFP_KERNEL);
		if (!ingr_policy) {
			MV_DP_CLI_FAIL("Cache alloc failed for INGRESS QOS Policy:%d\n", MV_NSS_DP_OUT_OF_RESOURCES, num);
			return;
		}

		ingr_allocated = num;
		MV_DP_CLI_OK("INGRESS QOS Policy Allocated:%d\n", ingr_allocated);
		return;
	}
	if (t == QOS_DIR_TYPE_EGRESS) {
		if (egr_allocated) {
			MV_DP_CLI_FAIL("EGRESS QOS Policy Already allocated: %d\n", MV_NSS_DP_FAILED, egr_allocated);
			return;
		}

		egr_policy = kzalloc(sizeof(mv_nss_dp_egress_qos_policy_t) * num, GFP_KERNEL);
		if (!egr_policy) {
			MV_DP_CLI_FAIL("Cache alloc failed for EGRESS QOS Policy:%d\n", MV_NSS_DP_OUT_OF_RESOURCES, num);
			return;
		}

		egr_allocated = num;
		MV_DP_CLI_OK("EGRESS QOS Policy Allocated:%d\n", egr_allocated);
		return;
	}

	MV_DP_CLI_FAIL("ILLEGAL QOS Policy Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
}

static void mv_dp_qos_policy_release(enum mv_dp_sys_dir_type t)
{

	if (t == QOS_DIR_TYPE_INGRESS) {
		if (!ingr_allocated) {
			MV_DP_CLI_FAIL("INGRESS QOS Policy NOT allocated\n", MV_NSS_DP_FAILED);
			return;
		}
		kfree(ingr_policy);
		ingr_allocated = 0;
		MV_DP_CLI_OK("INGRESS QOS Policy Cache Released\n");
		return;
	}
	if (t == QOS_DIR_TYPE_EGRESS) {
		if (!egr_allocated) {
			MV_DP_CLI_FAIL("EGRESS QOS Policy NOT allocated\n", MV_NSS_DP_FAILED);
			return;
		}
		kfree(egr_policy);
		egr_allocated = 0;
		MV_DP_CLI_OK("EGRESS QOS Policy Cache Released\n");
		return;
	}

	MV_DP_CLI_FAIL("ILLEGAL QOS Policy Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
}


static void mv_dp_qos_policy_get(enum mv_dp_sys_dir_type t, int id)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
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

	if (t == QOS_DIR_TYPE_INGRESS) {
		res.cb = mv_dp_ingr_qos_policy_get_cb;
		rc = mv_nss_dp_ingress_qos_policy_get(id, &res);
	} else if (t == QOS_DIR_TYPE_EGRESS) {
		res.cb = mv_dp_egr_qos_policy_get_cb;
		rc = mv_nss_dp_egress_qos_policy_get(id, &res);
	} else {
		MV_DP_CLI_FAIL("INVALID POLICY Type:%d\n", MV_NSS_DP_FAILED, t);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto err;
	}

	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("READ QOS Policy_id:%d\n", rc, id);
		goto err;
	}

	MV_DP_CLI_TMSG("Get QOS POLICY:%d\n", id);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get QOS POLICY\n", MV_NSS_DP_API_EXEC_TIMEOUT);
	}
err:
	if (res.cookie)
		kfree(res.cookie);

}

static void mv_dp_qos_policy_read(int i, enum mv_dp_sys_dir_type t, int pol_id)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
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

	if (t == QOS_DIR_TYPE_INGRESS) {
		res.cb = mv_dp_ingr_qos_policy_read_cb;
		rc = mv_nss_dp_ingress_qos_policy_get(pol_id, &res);
	} else if (t == QOS_DIR_TYPE_EGRESS) {
		res.cb = mv_dp_egr_qos_policy_read_cb;
		rc = mv_nss_dp_egress_qos_policy_get(pol_id, &res);
	} else {
		MV_DP_CLI_FAIL("INVALID POLICY Type:%d\n", MV_NSS_DP_FAILED, t);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto err;
	}

	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("READ QOS Policy_id:%d\n", rc, pol_id);
		goto err;
	}

	MV_DP_CLI_TMSG("READ QOS POLICY_ID:%d\n", pol_id);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Read QOS POLICY\n", MV_NSS_DP_API_EXEC_TIMEOUT);
	}
err:
	if (res.cookie)
		kfree(res.cookie);
}


static void mv_dp_qos_policy_clear(int ind, enum mv_dp_sys_dir_type t)
{

	if (t == QOS_DIR_TYPE_INGRESS) {
		if (!ingr_allocated) {
			MV_DP_CLI_FAIL("INGRESS QOS Policy NOT allocated\n", MV_NSS_DP_FAILED);
			return;
		}
		if (ind == -1) {
			memset(ingr_policy, 0, sizeof(mv_nss_dp_ingress_qos_policy_t) * ingr_allocated);
			MV_DP_CLI_OK("INGRESS QOS POLICY Cache Cleared: %d\n", ingr_allocated);
		} else if (!MV_DP_SYSFS_INGRESS_QOS_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal INGRESS QOS POLICY index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		} else	{
			memset(&ingr_policy[ind], 0, sizeof(mv_nss_dp_ingress_qos_policy_t));
			MV_DP_CLI_OK("INGRESS QOS POLICY Cache: %d Cleared\n", ind);
			return;
		}
	}
	if (t == QOS_DIR_TYPE_EGRESS) {
		if (!egr_allocated) {
			MV_DP_CLI_FAIL("EGRESS QOS Policy NOT allocated\n", MV_NSS_DP_FAILED);
			return;
		}
		if (ind == -1) {
			memset(egr_policy, 0, sizeof(mv_nss_dp_egress_qos_policy_t) * egr_allocated);
			MV_DP_CLI_OK("EGRESS QOS POLICY Cache Cleared: %d\n", egr_allocated);
		} else if (!MV_DP_SYSFS_EGRESS_QOS_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal EGRESS QOS POLICY index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		} else	{
			memset(&egr_policy[ind], 0, sizeof(mv_nss_dp_egress_qos_policy_t));
			MV_DP_CLI_OK("EGRESS QOS POLICY Cache: %d Cleared\n", ind);
			return;
		}
	}

	MV_DP_CLI_FAIL("ILLEGAL QOS Policy Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
}


static void mv_dp_ingr_qos_policy_commit(int ind)
{
	mv_nss_dp_result_spec_t	res;
	int			tmp_count;
	mv_nss_dp_ingress_qos_policy_t *policy;
	struct completion	*compl_ptr;
	mv_nss_dp_status_t rc;

	if (ind == -1) {
		/*commit all*/
		tmp_count = ingr_allocated;
		policy = ingr_policy;
	} else if (!MV_DP_SYSFS_INGRESS_QOS_OK(ind)) {
		MV_DP_CLI_FAIL("Illegal INGRESS QOS Cache index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
		return;
	} else	{
		tmp_count = 1;
		policy = &ingr_policy[ind];
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
	res.cb = mv_dp_ingr_qos_policy_set_cb;
	res.xid = ind;

	rc = mv_nss_dp_ingress_qos_policy_set(policy, tmp_count, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Commit INGRESS QOS Policy Index: %d\n", rc, ind);
		goto err;
	}

	MV_DP_CLI_TMSG("Commit INGRESS QOS Policy Index:%d\n", ind);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Commit INGRESS QOS Policy Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
	}
err:
	if (res.cookie)
		kfree(res.cookie);
}


static void mv_dp_egr_qos_policy_commit(int ind)
{
	mv_nss_dp_result_spec_t	res;
	int			tmp_count;
	mv_nss_dp_egress_qos_policy_t *policy;
	struct completion	*compl_ptr;
	mv_nss_dp_status_t rc;

	if (ind == -1) {
		/*commit all*/
		tmp_count = egr_allocated;
		policy = egr_policy;
	} else if (!MV_DP_SYSFS_EGRESS_QOS_OK(ind)) {
		MV_DP_CLI_FAIL("Illegal EGRESS QOS Cache index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
		return;
	} else	{
		tmp_count = 1;
		policy = &egr_policy[ind];
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
	res.cb = mv_dp_egr_qos_policy_set_cb;
	res.xid = ind;

	rc = mv_nss_dp_egress_qos_policy_set(policy, tmp_count, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Commit EGRESS QOS Policy Index: %d\n", rc, ind);
		goto err;
	}

	MV_DP_CLI_TMSG("Commit EGRESS QOS Policy Index:%d\n", ind);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Commit INGRESS QOS Policy Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
	}
err:
	if (res.cookie)
		kfree(res.cookie);
}

static void mv_dp_qos_policy_commit(int i, enum mv_dp_sys_dir_type t)
{
	if (t == QOS_DIR_TYPE_INGRESS)
		return mv_dp_ingr_qos_policy_commit(i);
	if (t == QOS_DIR_TYPE_EGRESS)
		return mv_dp_egr_qos_policy_commit(i);

	MV_DP_CLI_FAIL("INVALID POLICY Type:%d\n", MV_NSS_DP_FAILED, t);
}

static void mv_dp_egr_qos_policy_set(int ind, enum mv_dp_sys_egr_pol_type t, int *prio)
{

	uint8_t *dest;
	int j;

	if (!MV_DP_SYSFS_EGRESS_QOS_OK(ind)) {
		MV_DP_CLI_FAIL("Illegal EGRESS QOS POLICY index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
		return;
	}

	if (t == EGR_POL_DSCP) {
		dest = egr_policy[ind].prio_to_dscp;
	} else if (t == EGR_POL_UP) {
		dest = egr_policy[ind].prio_to_up;
	} else if (t == EGR_POL_PCP) {
		dest = egr_policy[ind].prio_to_pcp;
	} else {
		MV_DP_CLI_FAIL("ILLEGAL EGRESS QOS Policy Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
		return;
	}

	for (j = 0; j < MV_NSS_DP_PRIO_NUM; j++)
		dest[j] = prio[j];

	MV_DP_CLI_OK("EGRESS %s QOS POLICY set for Cache index:%d\n", egr_qos_names[t], ind);
}

static void mv_dp_ingr_qos_policy_l2_set(int ind, int *prio)
{
	int j;

	if (!MV_DP_SYSFS_INGRESS_QOS_OK(ind)) {
		MV_DP_CLI_FAIL("Illegal INGRESS QOS POLICY index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
		return;
	}

	for (j = 0; j < 8; j++)
		ingr_policy[ind].l2_to_prio[j] = prio[j];

	MV_DP_CLI_OK("INGRESS L2 QOS POLICY set for Cache index:%d\n", ind);
}

static void mv_dp_ingr_qos_policy_dscp_set(int ind, u8 x, int *prio)
{

	u8 *dest;
	int j;

	if (!MV_DP_SYSFS_INGRESS_QOS_OK(ind)) {
		MV_DP_CLI_FAIL("Illegal INGRESS QOS POLICY index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
		return;
	}
	if (x < 0 || x > 3) {
		MV_DP_CLI_FAIL("Illegal INGRESS QOS POLICY for 2 MSB: %d\n", MV_NSS_DP_INVALID_PARAM, x);
		return;
	}

	dest = (u8 *)(ingr_policy[ind].dscp_to_prio) + (x << 4);
	for (j = 0; j < 16; j++)
		dest[j] = prio[j];

	MV_DP_CLI_OK("INGRESS DSCP QOS POLICY set for Cache index:%d MSB:%d\n", ind, x);
}

static void mv_dp_ingr_qos_policy_type_set(int i, int t)
{

	if (!MV_DP_SYSFS_INGRESS_QOS_OK(i)) {
		MV_DP_CLI_FAIL("Illegal INGRESS QOS POLICY index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}

	if (!MV_DP_SYS_INGR_QOS_POL_TYPE_IS_OK(t)) {
		MV_DP_CLI_FAIL("Illegal INGRESS QOS POLICY type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
		return;
	}

	ingr_policy[i].type = t;

	MV_DP_CLI_OK("INGRESS QOS POLICY type set for Cache index:%d to:%d(%s)\n", i, t, ingr_qos_names[t]);
}

static void mv_dp_ingr_qos_policy_prio_def_set(int i, u8 prio)
{

	if (!MV_DP_SYSFS_INGRESS_QOS_OK(i)) {
		MV_DP_CLI_FAIL("Illegal INGRESS QOS POLICY index: %d\n", MV_NSS_DP_INVALID_PARAM, i);
		return;
	}


	ingr_policy[i].prio_def = prio;

	MV_DP_CLI_OK("INGRESS QOS POLICY Default Fixed prio set for Cache index:%d to:%d\n", i, prio);
}

static void mv_dp_qos_policy_id_set(int ind, enum mv_dp_sys_dir_type t, u8 id)
{

	if (t == QOS_DIR_TYPE_INGRESS) {
		if (!MV_DP_SYSFS_INGRESS_QOS_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal INGRESS QOS POLICY index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}
		ingr_policy[ind].policy_id = id;
		MV_DP_CLI_OK("INGRESS QOS POLICY_ID set for Cache index:%d to:%d\n", ind, id);
	} else if (t == QOS_DIR_TYPE_EGRESS) {
		if (!MV_DP_SYSFS_EGRESS_QOS_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal EGRESS QOS POLICY index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}
		MV_DP_CLI_OK("EGRESS QOS POLICY_ID set for Cache index:%d to:%d\n", ind, id);
		egr_policy[ind].policy_id = id;

	} else {

		MV_DP_CLI_FAIL("ILLEGAL QOS Policy Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
	}
}

static void mv_dp_qos_policy_show(int i, enum mv_dp_sys_dir_type t)
{
	if (t == QOS_DIR_TYPE_INGRESS)
		return mv_dp_ingr_qos_policy_show(i);
	if (t == QOS_DIR_TYPE_EGRESS)
		return mv_dp_egr_qos_policy_show(i);

	MV_DP_CLI_FAIL("INVALID POLICY Type:%d\n", MV_NSS_DP_FAILED, t);
}


static void mv_dp_ingr_qos_policy_show_sinlge(mv_nss_dp_ingress_qos_policy_t *policy)
{

	int i;
	if (!policy) {
		MV_DP_CLI_FAIL("Null ingress policy ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}


	MV_DP_CLI_CONT("| ID |TYPE| NAME |Fixed Prio|<\n");
	MV_DP_CLI_CONT("|%4d|%4d|%6s|%10d|<\n",
		       policy->policy_id,
		       policy->type,
		       ingr_qos_names[policy->type],
		       policy->prio_def);
	MV_DP_CLI_CONT("|---------------------------|<\n");
	MV_DP_CLI_CONT("| L2 |PRIO|<\n");
	for (i = 0; i < 8; i++)
		MV_DP_CLI_CONT("|%4d|%4d|<\n", i, policy->l2_to_prio[i]);
	MV_DP_CLI_CONT("|---------|<\n");
	MV_DP_CLI_CONT("|DSCP|PRIO|<\n");
	for (i = 0; i < 64; i++)
		MV_DP_CLI_CONT("|%4d|%4d|<\n", i, policy->dscp_to_prio[i]);
}

static void mv_dp_egr_qos_policy_show_sinlge(mv_nss_dp_egress_qos_policy_t *policy)
{

	int i;
	if (!policy) {
		MV_DP_CLI_FAIL("Null egress policy ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("| ID |<\n");
	MV_DP_CLI_CONT("|%4d|<\n",
		       policy->policy_id);
	MV_DP_CLI_CONT("|---------|<\n");
	MV_DP_CLI_CONT("|PRIO| UP |<\n");
	for (i = 0; i < 16; i++)
		MV_DP_CLI_CONT("|%4d|%4d|<\n", i, policy->prio_to_up[i]);
	MV_DP_CLI_CONT("|---------|<\n");
	MV_DP_CLI_CONT("|PRIO|DSCP|<\n");
	for (i = 0; i < 16; i++)
		MV_DP_CLI_CONT("|%4d|%4d|<\n", i, policy->prio_to_dscp[i]);
	MV_DP_CLI_CONT("|---------|<\n");
	MV_DP_CLI_CONT("|PRIO| PCP|<\n");
	for (i = 0; i < 16; i++)
		MV_DP_CLI_CONT("|%4d|%4d|<\n", i, policy->prio_to_pcp[i]);

}

static void mv_dp_egr_qos_policy_show(int index)
{
	int i;
	if (index == -1) {
		/*show all*/
		MV_DP_CLI_OK("|EGRESS QOS POLICY|TOTAL:%d|<\n", egr_allocated);
		for (i = 0; i < egr_allocated; i++) {
			MV_DP_CLI_CONT("|EGRESS  QOS POLICY IND|\n|%22d|<\n", i);
			mv_dp_egr_qos_policy_show_sinlge(&egr_policy[i]);
		}

		MV_DP_CLI_CONT("|EGRESS QOS POLICY END|<\n");
		return;
	} else if (!MV_DP_SYSFS_EGRESS_QOS_OK(index)) {
			MV_DP_CLI_FAIL("Illegal EGRESS POLICY Cache index:%d", MV_NSS_DP_INVALID_PARAM, index);
			return;
	}

	/*show single*/

	MV_DP_CLI_OK("|EGRESS  QOS POLICY IND|%4d|<\n", index);
	mv_dp_egr_qos_policy_show_sinlge(&egr_policy[index]);
	MV_DP_CLI_CONT("|EGRESS QOS POLICY END|<\n");
}

static void mv_dp_ingr_qos_policy_show(int index)
{
	int i;
	if (index == -1) {
		/*show all*/
		MV_DP_CLI_OK("|INGRESS QOS POLICY|TOTAL:%d|<\n", ingr_allocated);
		for (i = 0; i < ingr_allocated; i++) {
			MV_DP_CLI_CONT("|INGRESS  QOS POLICY IND|\n|%22d|<\n", i);
			mv_dp_ingr_qos_policy_show_sinlge(&ingr_policy[i]);
		}

		MV_DP_CLI_CONT("|INGRESS QOS POLICY END|<\n");
		return;
	} else if (!MV_DP_SYSFS_INGRESS_QOS_OK(index)) {
			MV_DP_CLI_FAIL("Illegal INGRESS POLICY Cache index:%d", MV_NSS_DP_INVALID_PARAM, index);
			return;
	}

	/*show single*/

	MV_DP_CLI_OK("|INGRESS  QOS POLICY IND|%4d|<\n", index);
	mv_dp_ingr_qos_policy_show_sinlge(&ingr_policy[index]);
	MV_DP_CLI_CONT("|INGRESS QOS POLICY END|<\n");
}


/*************************CB************************************************************/
static void mv_dp_ingr_qos_policy_get_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get INGRESS QOS POLICY Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.ingress_qos_policy) {
		MV_DP_CLI_FAIL_CB("Get INGRESS QOS POLICY: Empty ptr Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Get INGRESS QOS POLICY\n", event);
	mv_dp_ingr_qos_policy_show_sinlge(event->params.ingress_qos_policy);
	MV_DP_CLI_CONT("|INGRESS QOS POLICY END|<\n");

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_egr_qos_policy_get_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get EGRESS QOS POLICY Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.egress_qos_policy) {
		MV_DP_CLI_FAIL_CB("Get EGRESS QOS POLICY: Empty ptr Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Get EGRESS QOS POLICY\n", event);
	mv_dp_egr_qos_policy_show_sinlge(event->params.egress_qos_policy);
	MV_DP_CLI_CONT("|EGRESS QOS POLICY END|<\n");

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}


static void mv_dp_ingr_qos_policy_read_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("READ INGRESS QOS POLICY Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.ingress_qos_policy) {
		MV_DP_CLI_FAIL_CB("READ INGRESS QOS POLICY: Empty ptr Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!MV_DP_SYSFS_INGRESS_QOS_OK(event->xid)) {
		MV_DP_CLI_FAIL_CB("Illegal INGRESS POLICY Cache index: %d\n", event, event->xid);
		goto err;
	}

	memcpy(&ingr_policy[event->xid], event->params.ingress_qos_policy, sizeof(mv_nss_dp_ingress_qos_policy_t));
	MV_DP_CLI_OK_CB("READ INGRESS QOS POLICY to Index:%d\n", event, event->xid);


err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}



static void mv_dp_egr_qos_policy_read_cb(mv_nss_dp_event_t *event)
{
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("READ EGRESS QOS POLICY Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.egress_qos_policy) {
		MV_DP_CLI_FAIL_CB("READ EGRESS QOS POLICY: Empty ptr Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!MV_DP_SYSFS_EGRESS_QOS_OK(event->xid)) {
		MV_DP_CLI_FAIL_CB("Illegal EGRESS POLICY Cache index: %d\n", event, event->xid);
		goto err;
	}

	memcpy(&egr_policy[event->xid], event->params.egress_qos_policy, sizeof(mv_nss_dp_egress_qos_policy_t));
	MV_DP_CLI_OK_CB("READ EGRESS QOS POLICY to Index:%d\n", event, event->xid);


err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_egr_qos_policy_set_cb(mv_nss_dp_event_t *event)
{
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("SET EGRESS QOS POLICY Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("SET EGRESS QOS POLICY Index:%d\n", event, event->xid);


err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_ingr_qos_policy_set_cb(mv_nss_dp_event_t *event)
{
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("SET INGRESS QOS POLICY Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("SET INGRESS QOS POLICY Index:%d\n", event, event->xid);


err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}
