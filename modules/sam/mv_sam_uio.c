/*
 * User I/O driver for Armada 7K/8K Packet Processor.
 *
 * Copyright (C) 2016, ENEA AB
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

#define DRIVER_NAME	"mv_sam_uio_drv"
#define DRIVER_VERSION	"0.1"
#define DRIVER_AUTHOR	"Marvell"
#define DRIVER_DESC	"UIO platform driver for Security Accelerator"

#define MAX_UIO_DEVS	3

/*
 * sam_uio_pdev_info
 * local information for uio module driver
 *
 * @uio_num:  number of uio devices
 * @map_num:  number of uio memory regions
 * @dev:      device pointer
 * @info:     uio_info array
 *
 */
struct sam_uio_pdev_info {
	int uio_num;
	int map_num;
	struct device *dev;
	char name[16];
	struct uio_info uio[MAX_UIO_DEVS];
};

/*
 * sam_uio_probe() - mv_pp_uio_drv platform driver probe routine
 * - register uio devices filled with memory maps retrieved from device tree
 */
static int sam_uio_probe(struct platform_device *pdev)
{
	struct platform_device *eip_pdev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *eip_node;
	struct sam_uio_pdev_info  *pdev_info;
	struct uio_info *uio;
	struct resource *res;
	int err = 0, mem_cnt = 0;
	static u8 cpn_count;

	if (!np) {
		dev_err(dev, "Non DT case is not supported\n");
		return -EINVAL;
	}

	pdev_info = devm_kzalloc(dev, sizeof(struct sam_uio_pdev_info),
				   GFP_KERNEL);
	if (!pdev_info) {
		err = -ENOMEM;
		goto fail;
	}

	pdev_info->dev = dev;

	eip_node = of_parse_phandle(np, "eip_access", 0);
	if (!eip_node) {
		dev_err(dev, "eip_access node is not found\n");
		err = -EINVAL;
		goto fail_uio;
	}
	eip_pdev = of_find_device_by_node(eip_node);
	if (!eip_pdev)
		goto fail_uio;

	for (int idx = 0; idx < MAX_UIO_DEVS; ++idx) {
		int i;

		uio = &pdev_info->uio[idx];
		snprintf(pdev_info->name, sizeof(pdev_info->name), "uio_%s_%d",  eip_node->name, cpn_count);
		uio->name = pdev_info->name;
		uio->version = DRIVER_VERSION;

		for (i = 0; i < MAX_UIO_MAPS; i++, ++mem_cnt) {
			res = platform_get_resource(eip_pdev, IORESOURCE_MEM, mem_cnt);
			if (!res)
				break;

			uio->mem[i].memtype = UIO_MEM_PHYS;
			uio->mem[i].addr = res->start & PAGE_MASK;
			uio->mem[i].size = PAGE_ALIGN(resource_size(res));
			uio->mem[i].name = "regs";
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

	cpn_count++;
	return 0;

fail_uio:
	devm_kfree(dev, pdev_info);
fail:
	return err;
}

/*
 * sam_uio_remove() - SAM UIO platform driver release routine
 * - unregister uio devices
 */
static int sam_uio_remove(struct platform_device *pdev)
{
	struct sam_uio_pdev_info *pdev_info = platform_get_drvdata(pdev);

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

static const struct of_device_id sam_uio_of_match[] = {
	{ .compatible	= "marvell,uio-sam", },
	{ }
};

static struct platform_driver sam_uio_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= DRIVER_NAME,
		.of_match_table	= sam_uio_of_match,
	},
	.probe	= sam_uio_probe,
	.remove	= sam_uio_remove,
};

/* *INDENT-ON* */

module_platform_driver(sam_uio_driver);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:" DRIVER_NAME);
