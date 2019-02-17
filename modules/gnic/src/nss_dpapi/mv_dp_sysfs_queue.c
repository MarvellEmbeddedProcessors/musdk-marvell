
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


#include "mv_dp_defs.h"
#include "mv_dp_sysfs.h"
#include "mv_nss_dp.h"
#include "mv_dp_main.h"
#include "mv_dp_int_if.h"
#ifdef MV_DP_QOS_SHADOW
#include "mv_dp_qos.h"

#endif



#define MV_DP_SYSFS_QUEUE_ID_OK(n)	((n) >= 0 && (n) < (sizeof(u8)<<8))

#define MV_DP_SYSFS_QUEUE_CFG_MAX	(16)


#define MV_DP_SYSFS_EGRESS_QUEUE_OK(n)	((n) >= 0 && (n) < egr_q_allocated)

static  mv_nss_dp_egress_queue_cfg_t	*egr_queue;
static  int				egr_q_allocated; /*number of allocated entries*/

#define MV_DP_SYSFS_INGRESS_QUEUE_OK(n)	((n) >= 0 && (n) < ingr_q_allocated)

static  mv_nss_dp_ingress_queue_cfg_t	*ingr_queue;
static  int				ingr_q_allocated; /*number of allocated entries*/


/************************QUEUE CONFIG*****************************/

static void mv_dp_queue_cfg_show(int ind, enum mv_dp_sys_dir_type);
static void mv_dp_egr_queue_cfg_show(int ind);
static void mv_dp_ingr_queue_cfg_show(int ind);
static void mv_dp_egr_queue_cfg_show_sinlge(mv_nss_dp_egress_queue_cfg_t *q);
static void mv_dp_ingr_queue_cfg_show_sinlge(mv_nss_dp_ingress_queue_cfg_t *q);
static void mv_dp_queue_cfg_show_single(mv_nss_dp_queue_stats_t *stats);

static void mv_dp_queue_cfg_alloc(int n, enum mv_dp_sys_dir_type);
static void mv_dp_queue_cfg_release(enum mv_dp_sys_dir_type);
static void mv_dp_queue_cfg_clear(int ind, enum mv_dp_sys_dir_type);

static void mv_dp_queue_cfg_get(enum mv_dp_sys_dir_type, int id);
static void mv_dp_queue_cfg_read(int ind, enum mv_dp_sys_dir_type, int q_id);
static void mv_dp_queue_cfg_commit(int iind, enum mv_dp_sys_dir_type);
static void mv_dp_ingr_queue_cfg_commit(int ind);
static void mv_dp_egr_queue_cfg_commit(int ind);
static void mv_dp_queue_stats_get(enum mv_dp_sys_dir_type t, int q_id);
static void mv_dp_queue_stats_reset(enum mv_dp_sys_dir_type t, int q_id);

/***********INTERNAL DEBUG ONLY****************************************/
#ifdef MV_DP_QOS_SHADOW
static void mv_dp_queue_shadow_show(int ind, enum mv_dp_sys_dir_type);
static void mv_dp_egr_queue_shadow_show(int index);
static void mv_dp_ingr_queue_shadow_show(int index);
#endif
/************fields*******************************************/

static void mv_dp_queue_cfg_id_set(int ind, enum mv_dp_sys_dir_type t, uint id);
static void mv_dp_queue_cfg_length_set(int ind, enum mv_dp_sys_dir_type t, u16 length);
static void mv_dp_queue_cfg_weight_set(int ind, enum mv_dp_sys_dir_type t, u16 value);
static void mv_dp_queue_cfg_prio_set(int ind, enum mv_dp_sys_dir_type t, int type);
static void mv_dp_queue_cfg_policer_set(int ind, enum mv_dp_sys_dir_type t, u32 cir, u32 eir, u32 cbs, u32 ebs);
static void mv_dp_queue_cfg_shaper_set(int ind, enum mv_dp_sys_dir_type t, u32 cir, u32 eir, u32 cbs, u32 ebs);

static void mv_dp_ingr_queue_cfg_red_set(int ind, u16 red);

/*******************QUEUE CFG END*********************************/

static void mv_dp_ingr_queue_cfg_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_egr_queue_cfg_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_ingr_queue_cfg_read_cb(mv_nss_dp_event_t *event);
static void mv_dp_egr_queue_cfg_read_cb(mv_nss_dp_event_t *event);
static void mv_dp_ingr_queue_cfg_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_egr_queue_cfg_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_ingr_queue_stats_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_ingr_queue_stats_reset_cb(mv_nss_dp_event_t *event);
static void mv_dp_egr_queue_stats_get_cb(mv_nss_dp_event_t *event);
static void mv_dp_egr_queue_stats_reset_cb(mv_nss_dp_event_t *event);

static ssize_t mv_dp_queue_sysfs_help(char *b)
{
	int o = 0;

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat                             help     - Show Help\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [n][t]                     > queue_cfg_alloc - Allocate Queue cache type[t] of [n] entries\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [t]                        > queue_cfg_release - Release Queue CFG cache of type[t]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t]                     > queue_cfg_clear - Clear Queue CFG cache type[t] entry [i] to 0\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t]                     > queue_cfg_show - Show Queue type[t] cache entry[i] or -1: ALL)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [t][qid]                   > queue_cfg_get - Get Queue id[qid] from FW of type[t] and print\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t][qid]                > queue_cfg_read - Get Queue id[qid] from FW of type[t]\n"
		       "                                  and save to cache[i]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t]                     > queue_cfg_commit - Save Queue CFG of type[t]\n"
		       "                                  stored in cache [i] to FW\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t][qid]                > queue_cfg_id_set - Set Cached Queue [i] of type[t]\n"
		       "                                  queueid to[qid]\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t][l]                  > queue_cfg_length_set - Set Cached Queue [i] of type[t]\n"
		       "                                  length to[l](Ingress in KBytes, Egress in packets)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t][pr]                 > queue_cfg_prio_set - Set Cached Queue [i] of type[t]\n"
		       "                                  priority to [pr]\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t][w]                  > queue_cfg_weight_set - Set Cached Queue [i] of type[t]\n"
		       "                                  weight to[w](dec)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][t][cir][eir][cbs][ebs] > queue_cfg_policer_set - Set Cached Queue [i] of type[t]\n"
		       "                                  policer with [cir][eir][cbs][ebs](dec)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][cir][eir][cbs][ebs]    > egr_queue_cfg_shaper_set - Set Cached egress Queue [i]\n"
		       "                                  shaper with [cir][eir][cbs][ebs](dec)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [i][red]                   > ingr_queue_cfg_red_set - Set Ingress Cached Queue [i]\n"
		       "                                  red threshold to [red] (dec)\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [t][qid]                   > queue_stats_get - Get Queue id[qid] Statistics from FW of type[t] and print\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [t][qid]                   > queue_stats_reset - Reset Queue id[qid] Statistics from FW of type[t]\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [t]= qos/queue direction type:%d=INGRESS; %d=EGRESS",
		       QOS_DIR_TYPE_INGRESS, QOS_DIR_TYPE_EGRESS);
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [pr]= priority");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   [q]= queue number 8 bit dec\n");

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "\n                   dec - decimal 123; hex - hexadecimal (0x000)\n");

	return o;
}

static ssize_t mv_dp_queue_sysfs_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int off = 0;
	const char  *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "help")) {
		return mv_dp_queue_sysfs_help(buf);
	} else {
		MV_DP_CLI_FAIL("Parse Error CMD: <%s>\n", MV_NSS_DP_INVALID_PARAM, name);
		off = mv_dp_queue_sysfs_help(buf);
	}
	return off;
}

static ssize_t mv_dp_queue_sysfs_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t len)
{
	const char *name = attr->attr.name;
	unsigned int    a, b, c, d, e, f;


	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	a = b = c = d = e = f = 0;

	if (!strcmp(name, "queue_cfg_alloc")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_queue_cfg_alloc(a, b);
	} else	if (!strcmp(name, "queue_cfg_release")) {
		if (1 != sscanf(buf, "%d", &a))
			goto err;
		mv_dp_queue_cfg_release(a);
	} else	if (!strcmp(name, "queue_cfg_clear")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_queue_cfg_clear(a, b);
	} else	if (!strcmp(name, "queue_cfg_show")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_queue_cfg_show(a, b);
	} else	if (!strcmp(name, "queue_cfg_get")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_queue_cfg_get(a, b);
	} else	if (!strcmp(name, "queue_cfg_read")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_queue_cfg_read(a, b, c);
	} else	if (!strcmp(name, "queue_cfg_commit")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_queue_cfg_commit(a, b);
	} else	if (!strcmp(name, "queue_cfg_id_set")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_queue_cfg_id_set(a, b, c);
	} else	if (!strcmp(name, "queue_cfg_length_set")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_queue_cfg_length_set(a, b, c);
	} else	if (!strcmp(name, "queue_cfg_weight_set")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_queue_cfg_weight_set(a, b, c);
	} else	if (!strcmp(name, "queue_cfg_prio_set")) {
		if (3 != sscanf(buf, "%d %d %d", &a, &b, &c))
			goto err;
		mv_dp_queue_cfg_prio_set(a, b, c);
	} else	if (!strcmp(name, "ingr_queue_cfg_red_set")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_ingr_queue_cfg_red_set(a, b);
	} else	if (!strcmp(name, "queue_cfg_policer_set")) {
		if (6 != sscanf(buf, "%d %d %d %d %d %d", &a, &b, &c, &d, &e, &f))
			goto err;
		mv_dp_queue_cfg_policer_set(a, b, c, d, e, f);
	} else	if (!strcmp(name, "egr_queue_cfg_shaper_set")) {
		if (5 != sscanf(buf, "%d %d %d %d %d", &a, &b, &c, &e, &f))
			goto err;
		mv_dp_queue_cfg_shaper_set(a, QOS_DIR_TYPE_EGRESS, b, c, e, f);
	} else	if (!strcmp(name, "queue_stats_get")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_queue_stats_get(a, b);
	} else	if (!strcmp(name, "queue_stats_reset")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_queue_stats_reset(a, b);
#ifdef MV_DP_QOS_SHADOW
	} else	if (!strcmp(name, "queue_shadow_show")) {
		if (2 != sscanf(buf, "%d %d", &a, &b))
			goto err;
		mv_dp_queue_shadow_show(a, b);
#endif
	} else {
		goto err;
	}

	return len;
err:
	MV_DP_LOG_INF("Parse Error CMD:<%s>\n", attr->attr.name);
	return -EINVAL;
}


static DEVICE_ATTR(help,			S_IRUSR, mv_dp_queue_sysfs_show, NULL);

static DEVICE_ATTR(queue_cfg_alloc,		S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_cfg_release,		S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_cfg_clear,		S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_cfg_show,		S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_cfg_get,		S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_cfg_read,		S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_cfg_commit,		S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_cfg_id_set,		S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_cfg_length_set,	S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_cfg_weight_set,	S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_cfg_prio_set,		S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_cfg_policer_set,	S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(egr_queue_cfg_shaper_set,	S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(ingr_queue_cfg_red_set,	S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_stats_get,		S_IWUSR, NULL, mv_dp_queue_sysfs_store);
static DEVICE_ATTR(queue_stats_reset,		S_IWUSR, NULL, mv_dp_queue_sysfs_store);
#ifdef MV_DP_QOS_SHADOW
static DEVICE_ATTR(queue_shadow_show,		S_IWUSR, NULL, mv_dp_queue_sysfs_store);
#endif

static struct attribute *mv_dp_queue_sysfs_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_queue_cfg_alloc.attr,
	&dev_attr_queue_cfg_release.attr,
	&dev_attr_queue_cfg_clear.attr,
	&dev_attr_queue_cfg_show.attr,
	&dev_attr_queue_cfg_get.attr,
	&dev_attr_queue_cfg_read.attr,
	&dev_attr_queue_cfg_commit.attr,
	&dev_attr_queue_cfg_id_set.attr,
	&dev_attr_queue_cfg_length_set.attr,
	&dev_attr_queue_cfg_weight_set.attr,
	&dev_attr_queue_cfg_prio_set.attr,
	&dev_attr_queue_cfg_policer_set.attr,
	&dev_attr_egr_queue_cfg_shaper_set.attr,
	&dev_attr_ingr_queue_cfg_red_set.attr,
	&dev_attr_queue_stats_get.attr,
	&dev_attr_queue_stats_reset.attr,
#ifdef MV_DP_QOS_SHADOW
	&dev_attr_queue_shadow_show.attr,
#endif

	NULL
};


static struct attribute_group mv_dp_queue_sysfs_group = {
	.name = "queue",
	.attrs = mv_dp_queue_sysfs_attrs,
};



int mv_dp_queue_sysfs_init(struct kobject *ko)
{
	int err;
	err = sysfs_create_group(ko, &mv_dp_queue_sysfs_group);

	if (err) {
		MV_DP_LOG_INF("Queue CFG sysFS group init failed %d\n", err);
		return err;
	}

	MV_DP_LOG_DBG1("Queue CFG sysFS INITALIZED\n");
	return err;
}

int mv_dp_queue_sysfs_exit(struct kobject *ko)
{
	sysfs_remove_group(ko, &mv_dp_queue_sysfs_group);

	if (ingr_q_allocated) {
		kfree(ingr_queue);
		ingr_q_allocated = 0;
	}

	if (egr_q_allocated) {
		kfree(egr_queue);
		egr_q_allocated = 0;
	}

	return 0;
}


static void mv_dp_queue_cfg_alloc(int num, enum mv_dp_sys_dir_type t)
{
	if (num < 0 || num > MV_DP_SYSFS_QUEUE_CFG_MAX) {
			MV_DP_CLI_FAIL("Illegal Queue CFG Records number: %d\n", MV_NSS_DP_INVALID_PARAM, num);
			return;
	}

	if (t == QOS_DIR_TYPE_INGRESS) {
		if (ingr_q_allocated) {
			MV_DP_CLI_FAIL("INGR Queue CFG Already allocated: %d\n", MV_NSS_DP_FAILED, ingr_q_allocated);
			return;
		}

		ingr_queue = kzalloc(sizeof(mv_nss_dp_ingress_queue_cfg_t) * num, GFP_KERNEL);
		if (!ingr_queue) {
			MV_DP_CLI_FAIL("Cache alloc failed for INGRESS Queue size:%d\n", MV_NSS_DP_OUT_OF_RESOURCES, num);
			return;
		}

		ingr_q_allocated = num;
		MV_DP_CLI_OK("INGRESS Queue CFG Allocated:%d\n", ingr_q_allocated);
		return;
	}
	if (t == QOS_DIR_TYPE_EGRESS) {
		if (egr_q_allocated) {
			MV_DP_CLI_FAIL("EGRESS Queue CFG Already allocated: %d\n", MV_NSS_DP_FAILED, egr_q_allocated);
			return;
		}

		egr_queue = kzalloc(sizeof(mv_nss_dp_egress_queue_cfg_t) * num, GFP_KERNEL);
		if (!egr_queue) {
			MV_DP_CLI_FAIL("Cache alloc failed for EGRESS Queue CFG:%d\n", MV_NSS_DP_OUT_OF_RESOURCES, num);
			return;
		}

		egr_q_allocated = num;
		MV_DP_CLI_OK("EGRESS Queue CFG Allocated:%d\n", egr_q_allocated);
		return;
	}

	MV_DP_CLI_FAIL("ILLEGAL Queue Direction Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
}

static void mv_dp_queue_cfg_release(enum mv_dp_sys_dir_type t)
{

	if (t == QOS_DIR_TYPE_INGRESS) {
		if (!ingr_q_allocated) {
			MV_DP_CLI_FAIL("INGRESS Queue CFG NOT allocated\n", MV_NSS_DP_FAILED);
			return;
		}
		kfree(ingr_queue);
		ingr_q_allocated = 0;
		MV_DP_CLI_OK("INGRESS Queue CFG Cache Released\n");
		return;
	}
	if (t == QOS_DIR_TYPE_EGRESS) {
		if (!egr_q_allocated) {
			MV_DP_CLI_FAIL("EGRESS Queue CFG NOT allocated\n", MV_NSS_DP_FAILED);
			return;
		}
		kfree(egr_queue);
		egr_q_allocated = 0;
		MV_DP_CLI_OK("EGRESS Queue CFG Cache Released\n");
		return;
	}

	MV_DP_CLI_FAIL("ILLEGAL Queue Direction Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
}

static void mv_dp_queue_cfg_clear(int ind, enum mv_dp_sys_dir_type t)
{

	if (t == QOS_DIR_TYPE_INGRESS) {
		if (!ingr_q_allocated) {
			MV_DP_CLI_FAIL("INGRESS Queue CFG NOT allocated\n", MV_NSS_DP_FAILED);
			return;
		}
		if (ind == -1) {
			memset(ingr_queue, 0, sizeof(mv_nss_dp_ingress_queue_cfg_t) * ingr_q_allocated);
			MV_DP_CLI_OK("INGRESS Queue CFG Cache Cleared: %d\n", ingr_q_allocated);
		} else if (!MV_DP_SYSFS_INGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal INGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		} else	{
			memset(&ingr_queue[ind], 0, sizeof(mv_nss_dp_ingress_queue_cfg_t));
			MV_DP_CLI_OK("INGRESS Queue CFG Cache: %d Cleared\n", ind);
			return;
		}
	}
	if (t == QOS_DIR_TYPE_EGRESS) {
		if (!egr_q_allocated) {
			MV_DP_CLI_FAIL("EGRESS QOS Policy NOT allocated\n", MV_NSS_DP_FAILED);
			return;
		}
		if (ind == -1) {
			memset(egr_queue, 0, sizeof(mv_nss_dp_egress_queue_cfg_t) * egr_q_allocated);
			MV_DP_CLI_OK("EGRESS Queue CFG Cache Cleared: %d\n", egr_q_allocated);
		} else if (!MV_DP_SYSFS_EGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal EGRESS QOS POLICY index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		} else	{
			memset(&egr_queue[ind], 0, sizeof(mv_nss_dp_egress_queue_cfg_t));
			MV_DP_CLI_OK("EGRESS Queue CFG Cache: %d Cleared\n", ind);
			return;
		}
	}

	MV_DP_CLI_FAIL("ILLEGAL Queue Direction Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
}


static void mv_dp_queue_cfg_show(int i, enum mv_dp_sys_dir_type t)
{
	if (t == QOS_DIR_TYPE_INGRESS)
		return mv_dp_ingr_queue_cfg_show(i);
	if (t == QOS_DIR_TYPE_EGRESS)
		return mv_dp_egr_queue_cfg_show(i);

	MV_DP_CLI_FAIL("ILLEGAL Queue Direction Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
}

static void mv_dp_egr_queue_cfg_show(int index)
{
	int i;
	if (index == -1) {
		/*show all*/
		MV_DP_CLI_OK("|EGRESS Queue CFG|TOTAL:%d|<\n", egr_q_allocated);
		MV_DP_CLI_CONT("|IND |QUEUE|PRIO |WEIGHT|LENGTH| POLICER (CIR-EIR:CBS-EBS) | SHAPER  (CIR-EIR:CBS-EBS) |<\n");
		for (i = 0; i < egr_q_allocated; i++) {
			MV_DP_CLI_CONT("|%4u", i);
			mv_dp_egr_queue_cfg_show_sinlge(&egr_queue[i]);
		}

		MV_DP_CLI_CONT("|EGRESS Queue CFG END|<\n");
		return;
	} else if (!MV_DP_SYSFS_EGRESS_QUEUE_OK(index)) {
			MV_DP_CLI_FAIL("Illegal EGRESS Queue Cache index:%d", MV_NSS_DP_INVALID_PARAM, index);
			return;
	}

	/*show single*/

	MV_DP_CLI_OK("|EGRESS Queue CFG IND:%4u|<\n", index);
	MV_DP_CLI_CONT("|IND |QUEUE|PRIO |WEIGHT|LENGTH| POLICER (CIR-EIR:CBS-EBS) | SHAPER  (CIR-EIR:CBS-EBS) |<\n");
	MV_DP_CLI_CONT("|%4d", index);
	mv_dp_egr_queue_cfg_show_sinlge(&egr_queue[index]);
	MV_DP_CLI_CONT("|EGRESS Queue CFG END|<\n");
}


/*move to mv_dp_qos.c*/
static void mv_dp_egr_queue_cfg_show_sinlge(mv_nss_dp_egress_queue_cfg_t *q)
{

	if (!q) {
		MV_DP_CLI_FAIL("Null egress queue ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("|%5u|%5d|%6u|%6u|%6u-%6u:%6u-%6u|%6u-%6u:%6u-%6u|<\n",
		       q->queue,
		       q->sched_priority,
		       q->sched_weight,
		       q->tail_drop_thresh,
		       q->policer.cir, q->policer.eir,
		       q->policer.cbs, q->policer.ebs,
		       q->shaper.cir, q->shaper.eir,
		       q->shaper.cbs, q->shaper.ebs);
}


static void mv_dp_ingr_queue_cfg_show(int index)
{
	int i;
	if (index == -1) {
		/*show all*/
		MV_DP_CLI_OK("|INGRESS Queue CFG|TOTAL:%d|<\n", ingr_q_allocated);
		MV_DP_CLI_CONT("|IND |QUEUE|PRIO |WEIGHT|LENGTH|RED TRSH| POLICER (CIR-EIR:CBS-EBS) |<\n");
		for (i = 0; i < ingr_q_allocated; i++) {
			MV_DP_CLI_CONT("|%4u", i);
			mv_dp_ingr_queue_cfg_show_sinlge(&ingr_queue[i]);
		}

		MV_DP_CLI_CONT("|INGRESS Queue CFG END|<\n");
		return;
	} else if (!MV_DP_SYSFS_INGRESS_QUEUE_OK(index)) {
			MV_DP_CLI_FAIL("Illegal INGRESS Queue Cache index:%d", MV_NSS_DP_INVALID_PARAM, index);
			return;
	}

	/*show single*/


	MV_DP_CLI_OK("|INGRESS Queue CFG IND: %4d|<\n", index);
	MV_DP_CLI_CONT("|IND |QUEUE|PRIO |WEIGTH|LENGTH|RED TRSH| POLICER (CIR-EIR:CBS-EBS) |<\n");
	MV_DP_CLI_CONT("|%4u", index);
	mv_dp_ingr_queue_cfg_show_sinlge(&ingr_queue[index]);
	MV_DP_CLI_CONT("|INGRESS Queue CFG END|<\n");
}

static void mv_dp_ingr_queue_cfg_show_sinlge(mv_nss_dp_ingress_queue_cfg_t *q)
{

	if (!q) {
		MV_DP_CLI_FAIL("Null ingress queue ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}



	MV_DP_CLI_CONT("|%5u|%5d|%6u|%6u|%8u|%6u-%6u:%6u-%6u|<\n",
		       q->queue,
		       q->sched_priority,
		       q->sched_weight,
		       q->tail_drop_thresh,
		       q->red_thresh,
		       q->policer.cir, q->policer.eir,
		       q->policer.cbs, q->policer.ebs);

}

static void mv_dp_queue_cfg_show_single(mv_nss_dp_queue_stats_t *stats)
{

	if (!stats) {
		MV_DP_CLI_FAIL("Null Queue Statistics", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	MV_DP_CLI_CONT("|%5u|%10llu|%10llu|<\n",
		       (u32)stats->queue,
		       stats->pkts,
		       stats->errors
		       );

}

static void mv_dp_queue_cfg_get(enum mv_dp_sys_dir_type t, int q_id)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.xid = q_id;

	if (!MV_DP_SYSFS_QUEUE_ID_OK(q_id)) {
		MV_DP_CLI_FAIL("Queue ID is out of range:%u\n", MV_NSS_DP_INVALID_PARAM, q_id);
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

	if (t == QOS_DIR_TYPE_INGRESS) {
		res.cb = mv_dp_ingr_queue_cfg_get_cb;
		rc = mv_nss_dp_ingress_queue_cfg_get(q_id, &res);
	} else if (t == QOS_DIR_TYPE_EGRESS) {
		res.cb = mv_dp_egr_queue_cfg_get_cb;
		rc = mv_nss_dp_egress_queue_cfg_get(q_id, &res);
	} else {
		MV_DP_CLI_FAIL("INVALID QUEUE Direction Type:%d\n", MV_NSS_DP_FAILED, t);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto err;
	}

	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("READ QueueID CFG:%d Direction:%d\n", rc, q_id, t);
		goto err;
	}

	MV_DP_CLI_TMSG("Get QueueID CFG%d, direction:%d\n", q_id, t);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Queue CFG\n", MV_NSS_DP_API_EXEC_TIMEOUT);
	}
err:
	if (res.cookie)
		kfree(res.cookie);

}



static void mv_dp_queue_cfg_read(int i, enum mv_dp_sys_dir_type t, int q_id)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.xid = i;

	if (!MV_DP_SYSFS_QUEUE_ID_OK(q_id)) {
		MV_DP_CLI_FAIL("Queue ID is out of range:%u\n", MV_NSS_DP_INVALID_PARAM, q_id);
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

	if (t == QOS_DIR_TYPE_INGRESS) {
		res.cb = mv_dp_ingr_queue_cfg_read_cb;
		rc = mv_nss_dp_ingress_queue_cfg_get(q_id, &res);
	} else if (t == QOS_DIR_TYPE_EGRESS) {
		res.cb = mv_dp_egr_queue_cfg_read_cb;
		rc = mv_nss_dp_egress_queue_cfg_get(q_id, &res);
	} else {
		MV_DP_CLI_FAIL("INVALID QUEUE Direction Type:%d\n", MV_NSS_DP_FAILED, t);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto err;
	}

	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("READ QueueID CFG:%d Direction:%d\n", rc, q_id, t);
		goto err;
	}

	MV_DP_CLI_TMSG("Read QueueID CFG%d, direction:%d\n", q_id, t);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Queue CFG\n", MV_NSS_DP_API_EXEC_TIMEOUT);
	}
err:
	if (res.cookie)
		kfree(res.cookie);

}


static void mv_dp_ingr_queue_cfg_get_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get INGRESS Queue CFG Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.ingress_queue_cfg) {
		MV_DP_CLI_FAIL_CB("Get INGRESS Queue CFG: Empty ptr Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Get INGRESS Queue id:%d CFG\n", event, event->xid);
	MV_DP_CLI_CONT("|QUEUE|PRIO |WEIGHT|LENGTH|RED TRSH| POLICER (CIR-EIR:CBS-EBS) |<\n");
	mv_dp_ingr_queue_cfg_show_sinlge(event->params.ingress_queue_cfg);
	MV_DP_CLI_CONT("|INGRESS Queue CFG END|<\n");

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_egr_queue_cfg_get_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get EGRESS Queue CFG Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.egress_queue_cfg) {
		MV_DP_CLI_FAIL_CB("Get EGRESS Queue CFG: Empty ptr Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Get EGRESS Queue id:%d CFG\n", event, event->xid);
	MV_DP_CLI_CONT("|QUEUE|PRIO |WEIGHT|LENGTH| POLICER (CIR-EIR:CBS-EBS) | SHAPER  (CIR-EIR:CBS-EBS) |<\n");
	mv_dp_egr_queue_cfg_show_sinlge(event->params.egress_queue_cfg);
	MV_DP_CLI_CONT("|EGRESS Queue CFG END|<\n");

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}



static void mv_dp_ingr_queue_cfg_set_cb(mv_nss_dp_event_t *event)
{
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("SET INGRESS Queue CFG Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("SET INGRESS Queue CFG Index:%d\n", event, event->xid);


err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_egr_queue_cfg_set_cb(mv_nss_dp_event_t *event)
{
	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("SET EGRESS Queue CFG Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("SET EGRESS Queue CFG Index:%d\n", event, event->xid);


err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_egr_queue_cfg_read_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("READ EGRESS Queue CFG Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.egress_queue_cfg) {
		MV_DP_CLI_FAIL_CB("READ EGRESS Queue CFG: Empty ptr Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!MV_DP_SYSFS_EGRESS_QUEUE_OK(event->xid)) {
		MV_DP_CLI_FAIL_CB("Illegal EGRESS Queue CFG Cache index: %d\n", event, event->xid);
		goto err;
	}

	memcpy(&egr_queue[event->xid], event->params.egress_queue_cfg, sizeof(mv_nss_dp_egress_queue_cfg_t));
	MV_DP_CLI_OK_CB("READ EGRESS Queue CFG to Index:%d\n", event, event->xid);


err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}


static void mv_dp_ingr_queue_cfg_read_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("READ INGRESS Queue CFG Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.ingress_queue_cfg) {
		MV_DP_CLI_FAIL_CB("READ INGRESS Queue CFG: Empty ptr Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!MV_DP_SYSFS_INGRESS_QUEUE_OK(event->xid)) {
		MV_DP_CLI_FAIL_CB("Illegal INGRESS Queue CFG Cache index: %d\n", event, event->xid);
		goto err;
	}

	memcpy(&ingr_queue[event->xid], event->params.ingress_queue_cfg, sizeof(mv_nss_dp_ingress_queue_cfg_t));
	MV_DP_CLI_OK_CB("READ INGRESS Queue CFG to Index:%d\n", event, event->xid);


err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}


static void mv_dp_ingr_queue_stats_get_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get INGRESS Queue STATS Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.queue_stats) {
		MV_DP_CLI_FAIL_CB("Get INGRESS Queue STATS: Empty ptr Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Get INGRESS Queue id:%d STATS\n", event, event->xid);
	MV_DP_CLI_CONT("|QUEUE|RX PAKETS |RX ERRORS |<\n");
	mv_dp_queue_cfg_show_single(event->params.queue_stats);
	MV_DP_CLI_CONT("|INGRESS Queue CFG END|<\n");

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_ingr_queue_stats_reset_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Reset INGRESS Queue STATS Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Reset INGRESS Queue id:%d STATS\n", event, event->xid);

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_egr_queue_stats_get_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Get EGRESS Queue STATS Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	if (!event->params.queue_stats) {
		MV_DP_CLI_FAIL_CB("Get EGRESS Queue STATS: Empty ptr Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Get EGRESS Queue id:%d STATS\n", event, event->xid);
	MV_DP_CLI_CONT("|QUEUE|TX PAKETS |TX ERRORS |<\n");
	mv_dp_queue_cfg_show_single(event->params.queue_stats);
	MV_DP_CLI_CONT("|EGRESS Queue CFG END|<\n");

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_egr_queue_stats_reset_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Reset EGRESS Queue STATS Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}

	MV_DP_CLI_OK_CB("Reset EGRESS Queue id:%d STATS\n", event, event->xid);

err:	if (event->cookie && event->status != MV_NSS_DP_API_EXEC_TIMEOUT)
		complete(event->cookie);
}

static void mv_dp_queue_cfg_commit(int i, enum mv_dp_sys_dir_type t)
{
	if (t == QOS_DIR_TYPE_INGRESS)
		return mv_dp_ingr_queue_cfg_commit(i);
	if (t == QOS_DIR_TYPE_EGRESS)
		return mv_dp_egr_queue_cfg_commit(i);

	MV_DP_CLI_FAIL("INVALID QUEUE Direction:%d\n", MV_NSS_DP_FAILED, t);
}


static void mv_dp_ingr_queue_cfg_commit(int ind)
{
	mv_nss_dp_result_spec_t	res;
	int			tmp_count;
	mv_nss_dp_ingress_queue_cfg_t *queue;
	struct completion	*compl_ptr;
	mv_nss_dp_status_t rc;

	if (ind == -1) {
		/*commit all*/
		tmp_count = ingr_q_allocated;
		queue = ingr_queue;
	} else if (!MV_DP_SYSFS_INGRESS_QUEUE_OK(ind)) {
		MV_DP_CLI_FAIL("Illegal INGRESS Queue Cache index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
		return;
	} else	{
		tmp_count = 1;
		queue = &ingr_queue[ind];
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
	res.cb = mv_dp_ingr_queue_cfg_set_cb;
	res.xid = ind;

	rc = mv_nss_dp_ingress_queue_cfg_set(queue, tmp_count, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Commit INGRESS Queue CFG Index: %d\n", rc, ind);
		goto err;
	}

	MV_DP_CLI_TMSG("Commit INGRESS Queue CFG Index:%d\n", ind);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Commit INGRESS Queue CFG Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
	}
err:
	if (res.cookie)
		kfree(res.cookie);
}

static void mv_dp_egr_queue_cfg_commit(int ind)
{
	mv_nss_dp_result_spec_t	res;
	int			tmp_count;
	mv_nss_dp_egress_queue_cfg_t *queue;
	struct completion	*compl_ptr;
	mv_nss_dp_status_t rc;

	if (ind == -1) {
		/*commit all*/
		tmp_count = egr_q_allocated;
		queue = egr_queue;
	} else if (!MV_DP_SYSFS_EGRESS_QUEUE_OK(ind)) {
		MV_DP_CLI_FAIL("Illegal EGRESS Queue Cache index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
		return;
	} else	{
		tmp_count = 1;
		queue = &egr_queue[ind];
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
	res.cb = mv_dp_egr_queue_cfg_set_cb;
	res.xid = ind;

	rc = mv_nss_dp_egress_queue_cfg_set(queue, tmp_count, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Commit EGRESS Queue CFG Index: %d\n", rc, ind);
		goto err;
	}

	MV_DP_CLI_TMSG("Commit EGRESS Queue CFG Index:%d\n", ind);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Commit EGRESS Queue CFG Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
	}
err:
	if (res.cookie)
		kfree(res.cookie);
}


static void mv_dp_queue_cfg_id_set(int ind, enum mv_dp_sys_dir_type t, uint q_id)
{

	if (!MV_DP_SYSFS_QUEUE_ID_OK(q_id)) {
		MV_DP_CLI_FAIL("Illegal Queue ID: %u\n", MV_NSS_DP_INVALID_PARAM, q_id);
		return;
	}

	if (t == QOS_DIR_TYPE_INGRESS) {
		if (!MV_DP_SYSFS_INGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal INGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}
		ingr_queue[ind].queue = q_id;
		MV_DP_CLI_OK("INGRESS Queue CFG ID set for Cache index:%d to:%d\n", ind, q_id);
	} else if (t == QOS_DIR_TYPE_EGRESS) {
		if (!MV_DP_SYSFS_EGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal EGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}
		egr_queue[ind].queue = q_id;
		MV_DP_CLI_OK("EGRESS Queue CFG ID set for Cache index:%d to:%d\n", ind, q_id);

	} else {

		MV_DP_CLI_FAIL("ILLEGAL Queue Direction Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
	}
}


static void mv_dp_queue_cfg_length_set(int ind, enum mv_dp_sys_dir_type t, u16 length)
{
	if (t == QOS_DIR_TYPE_INGRESS) {
		if (!MV_DP_SYSFS_INGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal INGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}
		ingr_queue[ind].tail_drop_thresh = length;
		MV_DP_CLI_OK("INGRESS Queue CFG Length set for Cache index:%d to:%d\n", ind, length);
	} else if (t == QOS_DIR_TYPE_EGRESS) {
		if (!MV_DP_SYSFS_EGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal EGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}

		egr_queue[ind].tail_drop_thresh = length;
		MV_DP_CLI_OK("EGRESS Queue CFG Length set for Cache index:%d to:%d\n", ind, length);

	} else {

		MV_DP_CLI_FAIL("ILLEGAL Queue Direction Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
	}

}

static void mv_dp_queue_cfg_weight_set(int ind, enum mv_dp_sys_dir_type t, u16 weight)
{
	if (t == QOS_DIR_TYPE_INGRESS) {
		if (!MV_DP_SYSFS_INGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal INGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}
		ingr_queue[ind].sched_weight = weight;
		MV_DP_CLI_OK("INGRESS Queue CFG Weight set for Cache index:%d to:%d\n", ind, weight);
	} else if (t == QOS_DIR_TYPE_EGRESS) {
		if (!MV_DP_SYSFS_EGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal EGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}

		egr_queue[ind].sched_weight = weight;
		MV_DP_CLI_OK("EGRESS Queue CFG Weight set for Cache index:%d to:%d\n", ind, weight);

	} else {

		MV_DP_CLI_FAIL("ILLEGAL Queue Direction Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
	}

}


static void mv_dp_queue_cfg_prio_set(int ind, enum mv_dp_sys_dir_type t, int prio)
{


	if (t == QOS_DIR_TYPE_INGRESS) {
		if (!MV_DP_SYSFS_INGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal INGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}
		ingr_queue[ind].sched_priority = prio;
		MV_DP_CLI_OK("INGRESS Queue CFG Priority set for Cache index:%d to:%d\n", ind, prio);

	} else if (t == QOS_DIR_TYPE_EGRESS) {
		if (!MV_DP_SYSFS_EGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal EGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}
		egr_queue[ind].sched_priority = prio;
		MV_DP_CLI_OK("EGRESS Queue CFG Priority set for Cache index:%d to:%d\n", ind, prio);

	} else {

		MV_DP_CLI_FAIL("ILLEGAL Queue Direction Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
	}
}

static void mv_dp_queue_cfg_policer_set(int ind, enum mv_dp_sys_dir_type t, u32 cir, u32 eir, u32 cbs, u32 ebs)
{
	if (t == QOS_DIR_TYPE_INGRESS) {
		if (!MV_DP_SYSFS_INGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal INGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}
		ingr_queue[ind].policer.cir = cir;
		ingr_queue[ind].policer.eir = eir;
		ingr_queue[ind].policer.cbs = cbs;
		ingr_queue[ind].policer.ebs = ebs;

		MV_DP_CLI_OK("INGRESS Queue CFG Policer set for Cache index:%d to:CIR=%u,EIR=%u,CBS=%u,EBS=%u\n",
			     ind, ingr_queue[ind].policer.cir,
			     ingr_queue[ind].policer.eir,
			     ingr_queue[ind].policer.cbs,
			     ingr_queue[ind].policer.ebs);
	} else if (t == QOS_DIR_TYPE_EGRESS) {
		if (!MV_DP_SYSFS_EGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal EGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}

		egr_queue[ind].policer.cir = cir;
		egr_queue[ind].policer.eir = eir;
		egr_queue[ind].policer.cbs = cbs;
		egr_queue[ind].policer.ebs = ebs;

		MV_DP_CLI_OK("EGRESS Queue CFG Policer set for Cache index:%d to:CIR=%u,EIR=%u,CBS=%u,EBS=%u\n",
			     ind, egr_queue[ind].policer.cir,
			     egr_queue[ind].policer.eir,
			     egr_queue[ind].policer.cbs,
			     egr_queue[ind].policer.ebs);
	} else {
		MV_DP_CLI_FAIL("ILLEGAL Queue Direction Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
	}
}


static void mv_dp_queue_cfg_shaper_set(int ind, enum mv_dp_sys_dir_type t, u32 cir, u32 eir, u32 cbs, u32 ebs)
{
	/*only egress*/
	if (t == QOS_DIR_TYPE_EGRESS) {
		if (!MV_DP_SYSFS_EGRESS_QUEUE_OK(ind)) {
			MV_DP_CLI_FAIL("Illegal EGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
			return;
		}
		egr_queue[ind].shaper.cir = cir;
		egr_queue[ind].shaper.eir = eir;
		egr_queue[ind].shaper.cbs = cbs;
		egr_queue[ind].shaper.ebs = ebs;
		MV_DP_CLI_OK("EGRESS Queue CFG Policer set for Cache index:%d to:CIR=%u,EIR=%u,CBS=%u,EBS=%u\n",
			     ind, egr_queue[ind].shaper.cir,
			     egr_queue[ind].shaper.eir,
			     egr_queue[ind].shaper.cbs,
			     egr_queue[ind].shaper.ebs);

	} else {

		MV_DP_CLI_FAIL("ILLEGAL Queue Direction Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
	}

}



static void mv_dp_ingr_queue_cfg_red_set(int ind, u16 red)
{


	if (!MV_DP_SYSFS_INGRESS_QUEUE_OK(ind)) {
		MV_DP_CLI_FAIL("Illegal INGRESS Queue CFG index: %d\n", MV_NSS_DP_INVALID_PARAM, ind);
		return;
	}
	ingr_queue[ind].red_thresh = red;
	MV_DP_CLI_OK("INGRESS Queue CFG RED Threshold set for Cache index:%d to:%d\n", ind, red);

}

static void mv_dp_queue_stats_get(enum mv_dp_sys_dir_type t, int q_id)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.xid = q_id;

	if (!MV_DP_SYSFS_QUEUE_ID_OK(q_id)) {
		MV_DP_CLI_FAIL("Queue ID is out of range:%u\n", MV_NSS_DP_INVALID_PARAM, q_id);
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

	if (t == QOS_DIR_TYPE_INGRESS) {
		res.cb = mv_dp_ingr_queue_stats_get_cb;
		rc = mv_nss_dp_ingress_queue_stats_get(q_id, &res);
	} else if (t == QOS_DIR_TYPE_EGRESS) {
		res.cb = mv_dp_egr_queue_stats_get_cb;
		rc = mv_nss_dp_egress_queue_stats_get(q_id, &res);
	} else {
		MV_DP_CLI_FAIL("INVALID QUEUE Direction Type:%d\n", MV_NSS_DP_FAILED, t);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto err;
	}

	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("READ QueueID Stats:%d Direction:%d\n", rc, q_id, t);
		goto err;
	}

	MV_DP_CLI_TMSG("Get QueueID Stats%d, direction:%d\n", q_id, t);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Get Queue Stats\n", MV_NSS_DP_API_EXEC_TIMEOUT);
	}
err:
	if (res.cookie)
		kfree(res.cookie);

}

static void mv_dp_queue_stats_reset(enum mv_dp_sys_dir_type t, int q_id)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;
	struct completion *compl_ptr;

	/*the index is stored in xid*/
	res.xid = q_id;

	if (!MV_DP_SYSFS_QUEUE_ID_OK(q_id)) {
		MV_DP_CLI_FAIL("Queue ID is out of range:%u\n", MV_NSS_DP_INVALID_PARAM, q_id);
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

	if (t == QOS_DIR_TYPE_INGRESS) {
		res.cb = mv_dp_ingr_queue_stats_reset_cb;
		rc = mv_nss_dp_ingress_queue_stats_reset(q_id, &res);
	} else if (t == QOS_DIR_TYPE_EGRESS) {
		res.cb = mv_dp_egr_queue_stats_reset_cb;
		rc = mv_nss_dp_egress_queue_stats_reset(q_id, &res);
	} else {
		MV_DP_CLI_FAIL("INVALID QUEUE Direction Type:%d\n", MV_NSS_DP_FAILED, t);
		rc = MV_NSS_DP_INVALID_PARAM;
		goto err;
	}

	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Reset QueueID Stats:%d Direction:%d\n", rc, q_id, t);
		goto err;
	}

	MV_DP_CLI_TMSG("Reset QueueID Stats%d, direction:%d\n", q_id, t);
	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("Reset Queue Stats\n", MV_NSS_DP_API_EXEC_TIMEOUT);
	}
err:
	if (res.cookie)
		kfree(res.cookie);

}

#ifdef MV_DP_QOS_SHADOW
static void mv_dp_queue_shadow_show(int i, enum mv_dp_sys_dir_type t)
{
	if (t == QOS_DIR_TYPE_INGRESS)
		return mv_dp_ingr_queue_shadow_show(i);
	if (t == QOS_DIR_TYPE_EGRESS)
		return mv_dp_egr_queue_shadow_show(i);

	MV_DP_CLI_FAIL("ILLEGAL Queue Direction Type:%d\n", MV_NSS_DP_INVALID_PARAM, t);
}


static void mv_dp_ingr_queue_shadow_show(int index)
{
	int i;
	if (index == -1) {
		/*show all*/
		MV_DP_CLI_OK("|INGRESS Queue CFG Shadow|TOTAL:%d|<\n", MV_NSS_DP_PRIO_NUM);
		mv_dp_qos_show_ingr_entry(0, MV_DP_CLI_HDR);

		for (i = 0; i < MV_NSS_DP_PRIO_NUM; i++)
			mv_dp_qos_show_ingr_entry(i, MV_DP_CLI_DATA);

		MV_DP_CLI_CONT("|INGRESS Queue CFG Shadow END|<\n");
		return;
	} else if (!MV_DP_Q_ID_INGR_IS_OK(index)) {
			MV_DP_CLI_FAIL("Illegal INGRESS Queue ID:%d", MV_NSS_DP_INVALID_PARAM, index);
			return;
	}

	/*show single*/
	MV_DP_CLI_OK("|INGRESS Queue CFG Shadow|TOTAL:%d|<\n", MV_NSS_DP_PRIO_NUM);
	mv_dp_qos_show_ingr_entry(index, MV_DP_CLI_DATA | MV_DP_CLI_HDR);
	MV_DP_CLI_CONT("|INGRESS Queue CFG END|<\n");
}


static void mv_dp_egr_queue_shadow_show(int index)
{
	int i;
	if (index == -1) {
		/*show all*/
		MV_DP_CLI_OK("|EGRESS Queue CFG Shadow|TOTAL:%d|<\n", MV_NSS_DP_PRIO_NUM);
		mv_dp_qos_show_egr_entry(0, MV_DP_CLI_HDR);
		for (i = 0; i < MV_NSS_DP_PRIO_NUM; i++)
			mv_dp_qos_show_egr_entry(i, MV_DP_CLI_DATA);

		MV_DP_CLI_CONT("|EGRESS Queue CFG Shadow END|<\n");
		return;
	} else if (!MV_DP_Q_ID_EGR_IS_OK(index)) {
			MV_DP_CLI_FAIL("Illegal EGRESS Queue ID:%d", MV_NSS_DP_INVALID_PARAM, index);
			return;
	}

	/*show single*/
	MV_DP_CLI_OK("|EGRESS Queue CFG Shadow|TOTAL:%d|<\n", MV_NSS_DP_PRIO_NUM);
	mv_dp_qos_show_egr_entry(index, MV_DP_CLI_DATA | MV_DP_CLI_HDR);
	MV_DP_CLI_CONT("|EGRESS Queue CFG END|<\n");
}
#endif
