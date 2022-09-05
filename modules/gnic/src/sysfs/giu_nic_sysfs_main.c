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
*/
#include <linux/netdevice.h>
#include <linux/platform_device.h>
/*#include <linux/interrupt.h>*/

#include "giu_nic_sysfs.h"
#include "giu_nic_sysfs_eth.h"


char *agnic_pdev_name = "agnic-plat";
char *agnic_dev_name = "f06f00a0.agnic-plat";

struct platform_device *agnic_pdev;

/* agnic_sysfs_init - Register the Agnic sysfs under platform devices
 *
 *	We loop for the platform device. If no such, we register a new
 *
 */
int agnic_sysfs_init(void)
{
	struct device *pd;
	struct device *agnic_dev;
	struct net_device *netdev;

	/* Loop for agnic platform device */
	pd = bus_find_device_by_name(&platform_bus_type, NULL, agnic_pdev_name);
	if (!pd) {
		/* If no such device, register a new one */
		agnic_pdev = platform_device_register_simple(agnic_pdev_name, -1, NULL, 0);
		pd = bus_find_device_by_name(&platform_bus_type, NULL, agnic_pdev_name);
	}

	/* Look for the agnic device so we can take the needed info from it and register it
	 * in the new platform device
	 */
	agnic_dev = bus_find_device_by_name(&platform_bus_type, NULL, agnic_dev_name);
	if (!agnic_dev) {
		pr_err("%s: cannot find %s device\n", __func__, agnic_dev_name);
		return -1;
	}

	/* If a new platform device was registered, we need to register the relevant structures in it
	 * So we take netdev instance and register it at the new device.
	 */
	if (agnic_pdev) {
		netdev = dev_get_drvdata(agnic_dev);
		dev_set_drvdata(pd, netdev);
	}

	/* ETH Rx sysfs */
	agnic_rx_sysfs_init(&pd->kobj);
	/* RSS sysfs */
	agnic_rss_sysfs_init(&pd->kobj);

	/* MD sysfs */
#ifdef CONFIG_MV_NSS_ENABLE
	agnic_metadata_sysfs_init(&pd->kobj);
#endif

	return 0;
}

void agnic_sysfs_exit(void)
{
	struct device *pd;

	pd = bus_find_device_by_name(&platform_bus_type, NULL, agnic_pdev_name);
	if (pd) {
		agnic_rx_sysfs_exit(&pd->kobj);
		agnic_rss_sysfs_exit(&pd->kobj);
#ifdef CONFIG_MV_NSS_ENABLE
		agnic_metadata_sysfs_exit(&pd->kobj);
#endif
	}

	if (agnic_pdev)
		platform_device_unregister(agnic_pdev);
}
