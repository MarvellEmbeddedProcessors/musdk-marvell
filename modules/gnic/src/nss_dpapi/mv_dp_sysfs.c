
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

#include "mv_dp_defs.h"
#include "mv_dp_sysfs.h"

#include "mv_dp_main.h"
#include "mv_dp_sysfs_client.h"
#include "mv_dp_sysfs_port.h"
#include "mv_dp_sysfs_vlan.h"
#include "mv_dp_sysfs_dbg.h"
#include "mv_dp_sysfs_flow.h"
#include "mv_dp_sysfs_dtls.h"
#include "mv_dp_sysfs_qos.h"
#include "mv_dp_sysfs_queue.h"
#include "mv_dp_sysfs_hash.h"
#include "mv_dp_sysfs_mc.h"



#define MV_DP_SYSFS_SYSFS_DELAY		(0)


static struct  platform_device *mv_dp_sysfs_dev;

static unsigned mv_dp_sysfs_delay; /*if not zero -- sync sysfs execution with this ms timeout */


/*Forward decl*/
static void mv_dp_sysfs_init_mod(unsigned int n, unsigned int def_port);
static void mv_dp_sysfs_shutdown_mod(void);
static void mv_dp_sysfs_state_get(void);
static void mv_dp_sysfs_bypass_state_set(unsigned int enable);
static void mv_dp_sysfs_bypass_state_get(void);
static int mv_dp_sysfs_help(char *str);
static ssize_t mv_dp_sysfs_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t len);
static ssize_t mv_dp_sysfs_show(struct device *dev,
			  struct device_attribute *attr, char *buf);

static void mv_dp_sysfs_init_cb(mv_nss_dp_event_t *event);
static void mv_dp_sysfs_shutdown_cb(mv_nss_dp_event_t *event);
static void mv_dp_sysfs_bypass_state_set_cb(mv_nss_dp_event_t *event);
static void mv_dp_sysfs_bypass_state_get_cb(mv_nss_dp_event_t *event);

static int mv_dp_sysfs_help(char *b)
{
	int o = 0;

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cd            client   - go to client subdir\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cd            dbg      - go to debug subdir\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cd            dtls     - go to dtls subdir\n");
/*	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cd            flow     - go to flow subdir\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cd            hash     - go to hash subdir\n");*/
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cd            port     - go to port subdir\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cd            qos      - go to Quality Of Service subdir\n");
/*	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cd            queue    - go to ingress/egress queue subdir\n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cd            vlan     - go to vlan subdir\n");*/

	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat           help     - show help\n");
#if 0
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat           shutdown - put DPAPI module OFFLINE\n");
#endif
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [n] [p]  > init   - INIT DPAPI with [n]- sized pending buffer, [d] - def dest port\n");
/*	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat          state_get - Retrieve current network sub-system state\n");*/
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "echo [e]  > bypass_state_set - Set bypass state Enable/Disable, [e]=1/0 \n");
	o += scnprintf(b + o, PAGE_SIZE - o,
		       "cat         bypass_state_get - Get bypass state Enabled/Disabled\n");
	return o;
}

static ssize_t mv_dp_sysfs_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	int off = 0;
	const char  *name = attr->attr.name;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;


	if (!strcmp(name, "help")) {
		return mv_dp_sysfs_help(buf);
	} else if (!strcmp(name, "shutdown")) {
		mv_dp_sysfs_shutdown_mod();
	} else if (!strcmp(name, "state_get")) {
		mv_dp_sysfs_state_get();
	} else if (!strcmp(name, "bypass_state_get")) {
		mv_dp_sysfs_bypass_state_get();
	} else {

		MV_DP_CLI_FAIL("Parse Error CMD: <%s>\n", MV_NSS_DP_INVALID_PARAM, name);
		off = mv_dp_sysfs_help(buf);
	}

	return off;
}

static ssize_t mv_dp_sysfs_store(struct device *dev,
			   struct device_attribute *attr, const char *buf,
			   size_t len)
{
	const char *name = attr->attr.name;
	unsigned int    n, p;


	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	n = p = 0;

	if (!strcmp(name, "init")) {
		if (2 != sscanf(buf, "%d %d", &n, &p))
			goto err;
		mv_dp_sysfs_init_mod(n, p);
	} else if (!strcmp(name, "bypass_state_set")) {
			if (1 != sscanf(buf, "%d", &n))
				goto err;
			mv_dp_sysfs_bypass_state_set(n);
	} else {
		goto err;

	}

	return len;

err:
	MV_DP_CLI_FAIL("Parse Error <%s>\n", MV_NSS_DP_INVALID_PARAM, attr->attr.name);
	return -EINVAL;
}

static DEVICE_ATTR(help,		S_IRUSR, mv_dp_sysfs_show, NULL);
#if 0
static DEVICE_ATTR(shutdown,		S_IRUSR, mv_dp_sysfs_show, NULL);
#endif
static DEVICE_ATTR(init,		S_IWUSR, NULL, mv_dp_sysfs_store);
/*static DEVICE_ATTR(state_get,	S_IRUSR, mv_dp_sysfs_show, NULL);*/
static DEVICE_ATTR(bypass_state_set, S_IWUSR, NULL, mv_dp_sysfs_store);
static DEVICE_ATTR(bypass_state_get, S_IRUSR, mv_dp_sysfs_show, NULL);

static struct attribute *mv_dp_sysfs_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_init.attr,
#if 0
	&dev_attr_shutdown.attr,
#endif
/*	&dev_attr_state_get.attr,*/
	&dev_attr_bypass_state_set.attr,
	&dev_attr_bypass_state_get.attr,
	NULL
};


static struct attribute_group mv_dp_sysfs_group = {
	.attrs = mv_dp_sysfs_attrs,
};


int mv_dp_sysfs_init(void)
{
	struct device *pd;
	int rc;

	pd = bus_find_device_by_name(&platform_bus_type, NULL, "dpapi");

	if (!pd) {
		mv_dp_sysfs_dev = platform_device_register_simple("dpapi", -1, NULL, 0);
		pd = bus_find_device_by_name(&platform_bus_type, NULL, "dpapi");
	}

	if (!pd) {
		MV_DP_LOG_ERR("Cannot mount root sysFS: no dpapi object found %s:%d\n", MV_DP_RC_ERR_NO_DP_KO, __func__, __LINE__);
		return -ENOMEM;
	}

	rc = sysfs_create_group(&pd->kobj, &mv_dp_sysfs_group);
	if (rc) {
		MV_DP_LOG_ERR("Root sysfs mount failed\n", rc);
		return rc;
	}

	MV_DP_LOG_DBG1("DPAPI Root sysFS mounted\n");

	return rc;
}

int mv_dp_sysfs_init_entities(void)
{

	struct device *pd;

	pd = bus_find_device_by_name(&platform_bus_type, NULL, "dpapi");

	if (!pd) {
		MV_DP_LOG_ERR("Cannot mount child sysFS: no dpapi object found %s:%d\n", MV_DP_RC_ERR_NO_DP_KO, __func__, __LINE__);
		return -1;
	}

	mv_dp_sysfs_delay = MV_DP_SYSFS_SYSFS_DELAY;

	mv_dp_client_sysfs_init(&pd->kobj);
	mv_dp_dbg_sysfs_init(&pd->kobj);
	mv_dp_port_sysfs_init(&pd->kobj);
	/*mv_dp_vlan_sysfs_init(&pd->kobj);*/
	/*mv_dp_flow_sysfs_init(&pd->kobj);*/
	mv_dp_dtls_sysfs_init(&pd->kobj);
	mv_dp_qos_sysfs_init(&pd->kobj);
	/*mv_dp_queue_sysfs_init(&pd->kobj);*/
	/*mv_dp_hash_sysfs_init(&pd->kobj);*/
	/*mv_dp_mc_sysfs_init(&pd->kobj);*/

	MV_DP_LOG_DBG1("Entities sysFS mounted\n");

	return 0;
}


void mv_dp_sysfs_exit(void)
{

	struct device *pd;

	pd = bus_find_device_by_name(&platform_bus_type, NULL, "dpapi");
	if (!pd) {
		MV_DP_LOG_ERR("Cannot find dpapi ko at%s:%d\n", MV_DP_RC_ERR_NO_DP_KO, __func__, __LINE__);
		return;
	}

	platform_device_unregister(mv_dp_sysfs_dev);
}

void mv_dp_sysfs_exit_entities(void)
{

	struct device *pd;

	pd = bus_find_device_by_name(&platform_bus_type, NULL, "dpapi");
	if (!pd) {
		MV_DP_LOG_ERR("Cannot find dpapi ko at%s:%d\n", MV_DP_RC_ERR_NO_DP_KO, __func__, __LINE__);
		return;
	}

	mv_dp_sysfs_delay = 0;
	mv_dp_client_sysfs_exit(&pd->kobj);
	mv_dp_dbg_sysfs_exit(&pd->kobj);
	mv_dp_port_sysfs_exit(&pd->kobj);
	/*mv_dp_vlan_sysfs_exit(&pd->kobj);*/
	/*mv_dp_flow_sysfs_exit(&pd->kobj);*/
	mv_dp_dtls_sysfs_exit(&pd->kobj);
	mv_dp_qos_sysfs_exit(&pd->kobj);
	/*mv_dp_queue_sysfs_exit(&pd->kobj);*/
	/*mv_dp_hash_sysfs_exit(&pd->kobj);*/
	/*mv_dp_mc_sysfs_exit(&pd->kobj);*/

}

/*calls default evt handler*/
static void mv_dp_sysfs_shutdown_mod(void)
{
	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;

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

	res.cb = mv_dp_sysfs_shutdown_cb;
	res.xid = 0;


	if (MV_NSS_DP_OK != mv_nss_dp_shutdown(&res)) {
		MV_DP_CLI_FAIL("DPAPI SHUTDOWN\n", MV_NSS_DP_FAILED);
		if (res.cookie)
			kfree(res.cookie);
		return ;
	}

	MV_DP_CLI_TMSG("DPAPI SHUTDOWN\n");

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("DPAPI SHUTDOWN - Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}

/*calls default evt handler*/
static void mv_dp_sysfs_init_mod(unsigned int n, unsigned int def_port)
{
	mv_nss_dp_result_spec_t res;
	struct completion *compl_ptr;
	mv_nss_dp_init_cfg_t cfg;

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

	res.cb = mv_dp_sysfs_init_cb;
	res.xid = (def_port << 24 | n);

	cfg.requests_num = n;
	cfg.port_dst = def_port;
	cfg.notify_cb = 0;



	if (MV_NSS_DP_OK != mv_nss_dp_init(&cfg, &res)) {
		MV_DP_CLI_FAIL("DPAPI INIT\n", MV_NSS_DP_FAILED);
		if (res.cookie)
			kfree(res.cookie);
		return ;
	}

	MV_DP_CLI_TMSG("DPAPI INIT\n");

	if (res.cookie) {
		if (wait_for_completion_timeout(res.cookie, msecs_to_jiffies(mv_dp_sysfs_delay_get())) == 0)
			MV_DP_CLI_FAIL("DPAPI INIT - Timeout\n", MV_NSS_DP_API_EXEC_TIMEOUT);
		kfree(res.cookie);
	}
}

static void mv_dp_sysfs_bypass_state_set(unsigned int enable)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;

	res.cb = mv_dp_sysfs_bypass_state_set_cb;
	res.xid = enable;

	rc = mv_nss_dp_bypass_state_set(enable, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Bypass State Set:%s\n", rc, mv_dp_err_name_get(rc));
		return;
	}

	MV_DP_CLI_TMSG("Bypass State Set enable:%d\n", enable);

}

static void mv_dp_sysfs_bypass_state_get(void)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_status_t rc;

	res.cb = mv_dp_sysfs_bypass_state_get_cb;
	res.xid = 0;

	rc = mv_nss_dp_bypass_state_get(&res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("Bypass State Get:%s\n", rc, mv_dp_err_name_get(rc));
		return;
	}

	MV_DP_CLI_TMSG("Bypass State Get\n");

}

const char *mv_dp_state_print(mv_nss_dp_state_t state)
{
	switch (state) {
	case MV_NSS_DP_STATE_NONINIT: return "MV_NSS_DP_STATE_NONINIT";
	case MV_NSS_DP_STATE_LIVE: return "MV_NSS_DP_STATE_LIVE";
	case MV_NSS_DP_STATE_INACTIVE: return "MV_NSS_DP_STATE_INACTIVE";
	default:return "";
	}
}

static void mv_dp_sysfs_state_get(void)
{
	mv_nss_dp_result_spec_t res;
	mv_nss_dp_state_t state;
	mv_nss_dp_status_t rc;

	res.cb = 0;
	res.xid = 0;

	rc = mv_nss_dp_state_get(&state, &res);
	if (MV_NSS_DP_OK != rc) {
		MV_DP_CLI_FAIL("State Get Status:%s\n", rc, mv_dp_err_name_get(rc));
		return;
	}

	MV_DP_CLI_OK("State Get status:%s\n", mv_dp_state_print(state));
}

static void mv_dp_sysfs_init_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("DPAPI INIT Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("DPAPI INIT\n", event);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);
}


static void mv_dp_sysfs_bypass_state_set_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Bypass State Set: %s\n", event, mv_dp_err_name_get(event->status));
		return;
	}


	MV_DP_CLI_OK_CB("Bypass State Set:%d\n", event, event->xid);
}

static void mv_dp_sysfs_bypass_state_get_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("Bypass State Get: %s\n", event, mv_dp_err_name_get(event->status));
		return;
	}

	MV_DP_CLI_OK_CB("Bypass State Get:%s\n", event, (*event->params.bypass_state?"Enabled":"Disabled"));
}

static void mv_dp_sysfs_shutdown_cb(mv_nss_dp_event_t *event)
{

	if (!event) {
		MV_DP_CLI_FAIL("null event ptr\n", MV_NSS_DP_INVALID_PARAM);
		return;
	}

	if (MV_NSS_DP_OK != event->status) {
		MV_DP_CLI_FAIL_CB("DPAPI SHUTDOWN Status: %s\n", event, mv_dp_err_name_get(event->status));
		goto err;
	}


	MV_DP_CLI_OK_CB("DPAPI SHUTDOWN\n", event);

err:
	if (event->cookie && (event->status != MV_NSS_DP_API_EXEC_TIMEOUT))
		complete(event->cookie);
}



int mv_dp_sysfs_delay_get(void)
{
	return mv_dp_sysfs_delay;
}
void mv_dp_sysfs_delay_set(int to)
{
	mv_dp_sysfs_delay = to;
}



void mv_dp_sysfs_ip_to_str(char *dst_str, mv_nss_dp_ip_addr_t  *ip_ptr)
{
	uint8_t		*ip_addr;

	if (!dst_str)
		return;

	if (!ip_ptr) {
		sprintf(dst_str, "null");
		return;
	}

	ip_addr = (uint8_t *)ip_ptr->ip;

	switch (ip_ptr->ver) {
	case 4:
		sprintf(dst_str, "%d.%d.%d.%d", ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);
		break;

	case 6:

	    sprintf(dst_str, "%X:%X:%X:%X:%X:%X:%X:%X",
		    be16_to_cpu(*((uint16_t *)ip_addr)),        be16_to_cpu(*((uint16_t *)(ip_addr + 2))),
		    be16_to_cpu(*((uint16_t *)(ip_addr + 4))),  be16_to_cpu(*((uint16_t *)(ip_addr + 6))),
		    be16_to_cpu(*((uint16_t *)(ip_addr + 8))),  be16_to_cpu(*((uint16_t *)(ip_addr + 10))),
		    be16_to_cpu(*((uint16_t *)(ip_addr + 12))), be16_to_cpu(*((uint16_t *)(ip_addr + 14))));
		break;

	default:
		sprintf(dst_str, "null");

	}
}


