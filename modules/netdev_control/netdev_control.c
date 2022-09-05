/* SPDX-License-Identifier: GPL-2.0-only */
/* Copyright (c) 2021 Marvell. */

#include <linux/printk.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/netdevice.h>

#define NETDEV_SYSFS_PORT_INFO_DIR     "/sys/kernel/netdev_control/"
#define NETDEV_SYSFS_FILE              "netdev_control_state"
#define MAX_NETDEV_LEN                 256

static ssize_t netdev_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);

static struct kobj_attribute netdev_mailbox_attribute = __ATTR(net_dev, 0200, NULL, netdev_store);
struct kobject *netdev_mailbox_kobject;

/* This Function will be called from Init function */
static int netdev_control_init(void)
{
	int err;

	/* Creating a directory in /sys/kernel/ */
	netdev_mailbox_kobject = kobject_create_and_add(NETDEV_SYSFS_FILE, kernel_kobj);
	if (!netdev_mailbox_kobject)
		return -ENOMEM;

	/* Create sysfs file for etx_value /sys/devices/netdev_queue_state */
	err = sysfs_create_file(netdev_mailbox_kobject, &netdev_mailbox_attribute.attr);
	if (err)
		pr_err("failed to create the sysfs file in %s%s\n", NETDEV_SYSFS_PORT_INFO_DIR,
			NETDEV_SYSFS_FILE);
	return err;
}

static ssize_t netdev_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct net_device *netdev = NULL;
	char netdev_name[MAX_NETDEV_LEN];
	int queue_state;

	memset(netdev_name, 0, MAX_NETDEV_LEN);

	/* Get netdev name */
	if (sscanf(buf, "%s %d", &netdev_name[0], &queue_state) != 2 ||
		((queue_state != 1) && (queue_state != 0))) {
		pr_err("Wrong input");
		return -EINVAL;
	}

	/* search by defined interface name */
	netdev = dev_get_by_name(&init_net, netdev_name);
	if (!netdev)
		return -EINVAL;

	if (queue_state == 1)
		netif_tx_wake_all_queues(netdev);
	else if (queue_state == 0)
		netif_tx_stop_all_queues(netdev);

	dev_put(netdev);
	return count;
}

void netdev_control_remove(void)
{
	pr_info("***netdev_control_remove\n");
	kobject_put(netdev_mailbox_kobject);
	sysfs_remove_file(netdev_mailbox_kobject, &netdev_mailbox_attribute.attr);
}

module_init(netdev_control_init);
module_exit(netdev_control_remove);
MODULE_AUTHOR("MARVELL");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Marvell NETDEV_CONTROL");
