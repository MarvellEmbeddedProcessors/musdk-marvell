/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */
#include <linux/platform_device.h>
#include "giu_nic_sysfs_md_cls.h"
#include "giu_nic_md.h"

static struct kobject *metadata_kobj;

static ssize_t metadata_help(char *b)
{
	int o = 0;

	o += sprintf(b+o, "echo [32 digits] > tx_metadata - set tx metadata default in hex, (lower case, w/o 0x)\n");
	o += sprintf(b+o, "echo [1 or 0]    > rx_metadata_trace - enable or disable rx metadata tracing\n");
	o += sprintf(b+o, "echo [1 or 0]    > tx_metadata_classify - turn on/off tx metadata classify\n");
	o += sprintf(b+o, "cd   metadata_classify_info - directory metadata_classify_info\n");
	o += sprintf(b+o, "cat  tx_metadata            - get default tx metadata val in hex\n");
	o += sprintf(b+o, "cat  rx_metadata            - get rx metadata val in hex\n");
	o += sprintf(b+o, "cat  rx_metadata_trace      - get rx metadata debug flag\n");
	o += sprintf(b+o, "cat  tx_metadata_classify   - show tx metadata classify status\n");

	return o;
}

static ssize_t metadata_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	const char *name = attr->attr.name;
	struct meta_data_info md;
	int len;
	bool fl;

	if (!strcmp(name, "tx_metadata")) {
		agnic_get_tx_meta_data(&md);
		len =  snprintf(buf, META_DATA_STR_LEN + 1, "%016llx ", md.meta_data[0]);
		len += snprintf(buf + len, META_DATA_STR_LEN, "%016llx", md.meta_data[1]);
		len += snprintf(buf + len, 2, "\n");
	} else if (!strcmp(name, "rx_metadata")) {
		agnic_get_rx_meta_data(&md);
		len =  snprintf(buf, META_DATA_STR_LEN + 1, "%016llx ", md.meta_data[0]);
		len += snprintf(buf + len, META_DATA_STR_LEN, "%016llx", md.meta_data[1]);
		len += snprintf(buf + len, 2, "\n");
	} else if (!strcmp(name, "rx_metadata_trace")) {
		fl = agnic_get_rx_meta_data_store();
		len = snprintf(buf, 3, "%d\n", (fl) ? 1:0);
	} else if (!strcmp(name, "tx_metadata_classify")) {
		fl = agnic_get_tx_meta_data_classify();
		len = snprintf(buf, 3, "%d\n", (fl) ? 1:0);
	} else
		return metadata_help(buf);

	return len;
}

static ssize_t metadata_port_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t len)
{
	const char *name = attr->attr.name;
	struct meta_data_info md;
	u64 val;
	bool enable;

	char buf2[META_DATA_LEN + 1];

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "rx_metadata_trace")) {
		if (buf[0] == '0')
			enable = false;
		else
			enable = true;

		agnic_set_rx_meta_data_store(enable);
		return len;

	} else if (!strcmp(name, "tx_metadata_classify")) {
		if (buf[0] == '0')
			enable = false;
		else
			enable = true;

		agnic_set_tx_meta_data_classify(enable);

		return len;

	} else if (!strcmp(name, "tx_metadata")) {
		if (len != META_DATA_STR_LEN + 1)
			return -EINVAL;

		memcpy(buf2, buf, META_DATA_LEN);
		buf2[META_DATA_LEN] = '\0';

		if (kstrtou64(buf2, 16, &val) != 0)
			return -EINVAL;

		md.meta_data[0] = val;

		memcpy(buf2, buf + META_DATA_LEN, META_DATA_LEN);
		buf2[META_DATA_LEN] = '\0';

		if (kstrtou64(buf2, 16, &val) != 0)
			return -EINVAL;

		md.meta_data[1] = val;
		agnic_set_tx_meta_data(&md);
	} else
		return -ENOENT;

	return len;
}


static DEVICE_ATTR(help,	S_IRUSR, metadata_show, NULL);
static DEVICE_ATTR(tx_metadata,	S_IRUSR | S_IWUSR, metadata_show, metadata_port_store);
static DEVICE_ATTR(rx_metadata,	S_IRUSR, metadata_show, NULL);
static DEVICE_ATTR(rx_metadata_trace, S_IRUSR | S_IWUSR, metadata_show, metadata_port_store);
static DEVICE_ATTR(tx_metadata_classify, S_IRUSR | S_IWUSR, metadata_show, metadata_port_store);

static struct attribute *metadata_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_tx_metadata.attr,
	&dev_attr_rx_metadata.attr,
	&dev_attr_rx_metadata_trace.attr,
	&dev_attr_tx_metadata_classify.attr,
	NULL
};

static struct attribute_group metadata_group = {
	.attrs = metadata_attrs,
};

int agnic_metadata_sysfs_init(struct kobject *gbe_kobj)
{
	int err;

	metadata_kobj = kobject_create_and_add("md", gbe_kobj);

	if (!metadata_kobj) {
		pr_err("%s: cannot create metadata_kobj kobject\n", __func__);
		return -ENOMEM;
	}

	err = sysfs_create_group(metadata_kobj, &metadata_group);
	if (err)
		pr_err("sysfs group %s failed %d\n", metadata_group.name, err);

	agnic_metadata_classify_sysfs_init(metadata_kobj);

	return err;
}

int agnic_metadata_sysfs_exit(struct kobject *gbe_kobj)
{
	agnic_metadata_classify_sysfs_exit(metadata_kobj);
	sysfs_remove_group(gbe_kobj, &metadata_group);
	kobject_put(metadata_kobj);

	return 0;
}
