/*
* ***************************************************************************
* Copyright (C) 2018 Marvell International Ltd.
* ***************************************************************************
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
* ***************************************************************************
*/

#include "giu_nic.h"
#include "giu_nic_sysfs.h"


static ssize_t mv_rss_help(char *buf)
{
	int off = 0;

	off += scnprintf(buf + off, PAGE_SIZE,  "cat               rss_mode  - Show rss hash mode.\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "                               0 - 2-Tuple\n");
	off += scnprintf(buf + off, PAGE_SIZE,  "                               1 - 5-Tuple\n");

	return off;
}


static ssize_t mv_rss_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	const char      *name = attr->attr.name;
	int             off = 0, rss_mode;
	struct net_device *netdev = dev_get_drvdata(dev);
	struct agnic_adapter *adapter;

	adapter = netdev_priv(netdev);

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (!strcmp(name, "rss_mode")) {
		/* Convert rss enum to sysfs rss value */
		rss_mode = adapter->rss_mode - ING_HASH_TYPE_2_TUPLE;
		off += scnprintf(buf + off, PAGE_SIZE, "%d\n", rss_mode);
	} else {
		off += mv_rss_help(buf);
	}

	return off;
}

static DEVICE_ATTR(help,		S_IRUSR, mv_rss_show, NULL);
static DEVICE_ATTR(rss_mode,		S_IRUSR, mv_rss_show, NULL);

static struct attribute *rss_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_rss_mode.attr,
	NULL
};

static struct attribute_group rss_group = {
	.name = "rss",
	.attrs = rss_attrs,
};

int agnic_rss_sysfs_init(struct kobject *kobj)
{
	int err = 0;

	err = sysfs_create_group(kobj, &rss_group);
	if (err)
		pr_err("sysfs group %s failed %d\n", rss_group.name, err);

	return err;
}

int agnic_rss_sysfs_exit(struct kobject *kobj)
{
	sysfs_remove_group(kobj, &rss_group);

	return 0;
}

