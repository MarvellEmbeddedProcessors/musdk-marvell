/*
* ***************************************************************************
* Copyright (c) 2018 Marvell.
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
*
* User I/O driver for Armada GIU NIC.
*/

#include <linux/uio_driver.h>

#include "giu_nic.h"


#define AGNIC_UIO_HDR		"uio_agnic_tc_%u"
#define AGNIC_UIO_VERSION	"1.0"

#define MAX_UIO_DEVS		AGNIC_MAX_TC

struct uio_info uio[MAX_UIO_DEVS];


void agnic_uio_notify(struct agnic_adapter *adapter, int tc)
{
	uio_event_notify(&uio[tc]);
}

/*
 * agnic_uio_probe() - agnic uio probe routine
 * - register uio devices per TC
 *
 */
int agnic_uio_probe(struct device *dev)
{
	struct net_device       *netdev = dev_get_drvdata(dev);
	struct agnic_adapter    *adapter;
	struct uio_info *uinfo;
	char temp_buff[20];
	int tc_idx, ret = 0;

	adapter = netdev_priv(netdev);

	for (tc_idx = 0; tc_idx < adapter->num_tcs; ++tc_idx) {
		uinfo = &uio[tc_idx];
		snprintf(temp_buff, sizeof(temp_buff), AGNIC_UIO_HDR, tc_idx);
		uinfo->name = devm_kstrdup(dev, temp_buff, GFP_KERNEL);
		uinfo->version = AGNIC_UIO_VERSION;
		uinfo->irq = UIO_IRQ_CUSTOM;

		ret = uio_register_device(dev, uinfo);
		if (ret) {
			dev_err(dev, "Failed to register uio device\n");
			goto fail_uio;
		}
		pr_info("Registered uio for tx %d\n", tc_idx);
	}

	pr_info("Registered %d uio devices\n", tc_idx);

	return 0;

fail_uio:
	/* Free current uio mem allocation */
	devm_kfree(dev, (void *)uio[tc_idx].name);

	/* Free rest of uios mem allocation and unregister */
	while (--tc_idx >= 0) {
		devm_kfree(dev, (void *)uio[tc_idx].name);
		uio_unregister_device(&uio[tc_idx]);
	}

	return ret;
}

/*
 * agnic_uio_remove() - agnic uio remove routine
 * - free mem allocations
 * - unregister uio devices
 *
 */
int agnic_uio_remove(struct device *dev)
{
	struct net_device       *netdev = dev_get_drvdata(dev);
	struct agnic_adapter    *adapter;
	int tc_idx;

	adapter = netdev_priv(netdev);

	for (tc_idx = 0; tc_idx < adapter->num_tcs; ++tc_idx) {
		devm_kfree(dev, (void *)uio[tc_idx].name);
		uio_unregister_device(&uio[tc_idx]);
	}

	return 0;
}
