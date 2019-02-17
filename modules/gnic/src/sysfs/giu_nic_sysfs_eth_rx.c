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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/capability.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>

#include "giu_nic.h"
#include "giu_nic_sysfs.h"
#include "giu_nic_sysfs_eth.h"

static ssize_t agnic_help(char *b)
{
	int o = 0;

	o += sprintf(b+o, "echo [tc]             > num_rx_queues - show number of RX queues for port <p>\n");
	o += sprintf(b+o, "echo [tc] [rxq]       > rxqCounters   - show RXQ counters for <p/rxq>.\n");

	return o;
}

static ssize_t agnic_show(struct device *dev,
			  struct device_attribute *attr, char *buf)
{
	return agnic_help(buf);
}

static ssize_t agnic_port_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t len)
{
	const char *name = attr->attr.name;
	int err;
	unsigned int tc, rxq, arg_num;
	struct net_device *netdev = dev_get_drvdata(dev);
	struct agnic_adapter *adapter;

	adapter = netdev_priv(netdev);

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	/* Read port and value */
	err = tc = rxq = 0;
	arg_num = sscanf(buf, "%d %d", &tc, &rxq);
	if (arg_num != 2) {
		dev_err(dev, "%s: Error: 2 arguments are expected (TC and Q)\n", __func__);
		return len;
	}

	if (!strcmp(name, "num_rx_queues")) {
		DBG_MSG("tc %d: num_rx_queues=%d\n", tc, adapter->num_rx_queues);
	} else if (!strcmp(name, "rxqCounters")) {
		struct agnic_ring *rx_ring;
		u32 curr_counter, cur_total_pkts;

		/* TODO: check per TC for Q valid */
		if (rxq >= adapter->num_rx_queues) {
			dev_err(dev, "%s: tc %d: no queue id %d\n", __func__, tc, rxq);
			err = 1;
			goto sysfs_error;
		}

		rx_ring = adapter->rx_ring[rxq];

		/* Get current total_packet */
		curr_counter = rx_ring->total_packets;

		/* Calculate num of packets since last read */
		if (curr_counter >= rx_ring->prev_counter)
			cur_total_pkts = curr_counter - rx_ring->prev_counter;
		else
			/* Handle case of counter wrap-around */
			cur_total_pkts = curr_counter + (UINT_MAX - rx_ring->prev_counter + 1);

		/* Save current total_packets which were read */
		rx_ring->prev_counter = curr_counter;

		DBG_MSG("\n------ [TC #%d, rxq #%d counters] -----\n", tc, rxq);
		DBG_MSG("rxqCounters=%d\n", cur_total_pkts);
	} else {
		err = 1;
		dev_err(dev, "%s: illegal operation <%s>\n", __func__, attr->attr.name);
	}

sysfs_error:
	if (err) {
		dev_err(dev, "%s: error %d\n", __func__, err);
		return -EINVAL;
	}

	return len;
}


static DEVICE_ATTR(help,	  S_IRUSR, agnic_show, NULL);
static DEVICE_ATTR(num_rx_queues, S_IWUSR, NULL, agnic_port_store);
static DEVICE_ATTR(rxqCounters,	  S_IWUSR, NULL, agnic_port_store);

static struct attribute *agnic_attrs[] = {
	&dev_attr_help.attr,
	&dev_attr_num_rx_queues.attr,
	&dev_attr_rxqCounters.attr,
	NULL
};

static struct attribute_group agnic_rx_group = {
	.name = "rx",
	.attrs = agnic_attrs,
};

int agnic_rx_sysfs_init(struct kobject *gbe_kobj)
{
	int err;

	err = sysfs_create_group(gbe_kobj, &agnic_rx_group);
	if (err)
		pr_err("sysfs group %s failed %d\n", agnic_rx_group.name, err);

	return err;
}

int agnic_rx_sysfs_exit(struct kobject *gbe_kobj)
{
	sysfs_remove_group(gbe_kobj, &agnic_rx_group);

	return 0;
}
