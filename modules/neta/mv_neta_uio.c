/*
 * User I/O driver for NETA Packet Processor.
 *
 * Copyright (C) 2017
 *
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
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/uio_driver.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/mm.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>

#define DRIVER_NAME	"mv_neta_uio_drv"
#define DRIVER_VERSION	"0.1"
#define DRIVER_AUTHOR	"Marvell"
#define DRIVER_DESC	"UIO platform driver for NETA PP"

#define MAX_UIO_DEVS	3

/*
 * neta_uio_pdev_info
 * local information for uio module driver
 *
 * @uio_num:  number of uio devices
 * @map_num:  number of uio memory regions
 * @dev:      device pointer
 * @info:     uio_info array
 *
 */
struct neta_uio_pdev_info {
	int uio_num;
	int map_num;
	struct device *dev;
	char name[16];
	struct uio_info uio[MAX_UIO_DEVS];
};

/*
 * neta_uio_probe() - mv_pp_uio_drv platform driver probe routine
 * - register uio devices filled with memory maps retrieved from device tree
 */
static int neta_uio_probe(struct platform_device *pdev)
{
	struct platform_device *mem;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *port_node;
	struct neta_uio_pdev_info  *pdev_info;
	struct uio_info *uio;
	struct resource *res;
	int err = 0, mem_cnt = 0;
	const char *if_name;
	int tmp, port_cntr;

	if (!np) {
		dev_err(dev, "Non DT case is not supported\n");
		return -EINVAL;
	}

	pdev_info = devm_kzalloc(dev, sizeof(struct neta_uio_pdev_info),
				   GFP_KERNEL);
	if (!pdev_info) {
		err = -ENOMEM;
		goto fail;
	}

	pdev_info->dev = dev;
	port_node = NULL;
	mem = NULL;

	port_node = of_parse_phandle(np, "port_access", 0);
	if (!port_node) {
		dev_err(dev, "port_access node is not found\n");
		err = -EINVAL;
		goto fail_uio;
	}
	mem = of_find_device_by_node(port_node);
	if (!mem)
		goto fail_uio;

	of_property_read_string(np, "port_name", &if_name);
	tmp = sscanf(if_name, "eth%d", &port_cntr);
	if (tmp != 1)
		goto fail_uio;

	for (int idx = 0; idx < MAX_UIO_DEVS; ++idx) {
		int i;

		uio = &pdev_info->uio[idx];
		snprintf(pdev_info->name, sizeof(pdev_info->name), "uio_neta_%d", port_cntr);
		uio->name = pdev_info->name;
		uio->version = DRIVER_VERSION;

		for (i = 0; i < MAX_UIO_MAPS; i++, ++mem_cnt) {
			res = platform_get_resource(mem, IORESOURCE_MEM, mem_cnt);
			if (!res)
				break;

			uio->mem[i].memtype = UIO_MEM_PHYS;
			uio->mem[i].addr = res->start & PAGE_MASK;
			uio->mem[i].size = PAGE_ALIGN(resource_size(res));
			uio->mem[i].name = "neta_regs";
		}
		if (i) {
			err = uio_register_device(dev, uio);
			if (err) {
				dev_err(dev, "Failed to register uio device\n");
				goto fail_uio;
			}
			pdev_info->uio_num++;
		}
		if (!res)
			break;
	}
	if (!mem_cnt) {
		err = -EIO;
		goto fail_uio;
	}

	pdev_info->map_num = mem_cnt;

	dev_info(dev, "Registered %d uio devices, %d register maps attached\n",
		pdev_info->uio_num, pdev_info->map_num);

	platform_set_drvdata(pdev, pdev_info);

	return 0;

fail_uio:
	devm_kfree(dev, pdev_info);
fail:
	return err;
}

/*
 * neta_uio_remove() - NETA UIO platform driver release routine
 * - unregister uio devices
 */
static int neta_uio_remove(struct platform_device *pdev)
{
	struct neta_uio_pdev_info *pdev_info = platform_get_drvdata(pdev);

	if (!pdev_info)
		return -EINVAL;

	if (pdev_info->uio_num != 0) {
		for (int idx = 0; idx <= pdev_info->uio_num; ++idx)
			uio_unregister_device(&pdev_info->uio[idx]);
	}

	devm_kfree(&pdev->dev, pdev_info);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id neta_uio_of_match[] = {
	{ .compatible	= "marvell,uio-neta", },
	{ }
};

static struct platform_driver neta_uio_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= DRIVER_NAME,
		.of_match_table	= neta_uio_of_match,
	},
	.probe	= neta_uio_probe,
	.remove	= neta_uio_remove,
};

/* *INDENT-ON* */

module_platform_driver(neta_uio_driver);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:" DRIVER_NAME);
