/*
 * User I/O driver for Armada 7K/8K DMA-XOR v2.
 *
 * Copyright (c) 2017 Marvell.
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
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>


#define _D(fmt, ...) \
	pr_debug("%s -> %s: " fmt, DRIVER_NAME, __func__, __VA_ARGS__)
#define _E(fmt, ...) \
	pr_err("%s -> %s: " fmt, DRIVER_NAME, __func__, __VA_ARGS__)

#define DRIVER_NAME	"mv_dmax2_uio_drv"
#define DRIVER_VERSION	"1.3"
#define DRIVER_AUTHOR	"Marvell"
#define DRIVER_DESC	"UIO platform driver for Armada DMA-XOR"

/*
 * uio_pdrv_dmax2_info
 * local information for uio module driver
 *
 * @uio_num:  number of uio devices
 * @map_num:  number of uio memory regions
 * @dev:      device pointer
 * @info:     uio_info array
 *
 */
struct uio_pdrv_dmax2_info {
	int map_num;
	struct device *dev;
	char name[16];
	struct uio_info uio;
};


/*
 * mv_dmax2_uio_probe() - mv_dmax2_uio_drv platform driver probe routine
 * - register uio devices filled with memory maps retrieved from device tree
 *
 */
static int mv_dmax2_uio_probe(struct platform_device *pdev)
{
	struct platform_device *xor_pdev;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *xor_node;
	struct uio_pdrv_dmax2_info *uio_pdrv_dmax2;
	struct uio_info *uio;
	struct resource *res;
	char *tmp_name;
	int err = 0, mem_cnt = 0;
	static u8 dmax2_cnt;

	if (!np) {
		dev_err(dev, "Non DT case is not supported\n");
		err = -EINVAL;
		goto fail;
	}

	xor_node = of_parse_phandle(np, "xor_access", 0);
	if (!xor_node) {
		dev_err(dev, "xor_access node not found\n");
		return -EINVAL;
	}
	xor_pdev = of_find_device_by_node(xor_node);
	if (!xor_pdev) {
		dev_err(dev, "xor_access pdev not found\n");
		err = -EINVAL;
		goto fail;
	}

	uio_pdrv_dmax2 = devm_kzalloc(dev, sizeof(struct uio_pdrv_dmax2_info),
				   GFP_KERNEL);
	if (!uio_pdrv_dmax2) {
		err = -ENOMEM;
		goto fail;
	}

	uio_pdrv_dmax2->map_num = -EIO;
	uio_pdrv_dmax2->dev = dev;
	snprintf(uio_pdrv_dmax2->name, sizeof(uio_pdrv_dmax2->name), "uio_%s_%d",  xor_node->name, dmax2_cnt);

	uio = &uio_pdrv_dmax2->uio;
	uio->name = uio_pdrv_dmax2->name;
	uio->version = DRIVER_VERSION;

	for (int i = 0; i < MAX_UIO_MAPS; ++i, ++mem_cnt) {
		res =
		    platform_get_resource(xor_pdev, IORESOURCE_MEM, mem_cnt);
		if (!res)
			break;
		uio->mem[i].memtype = UIO_MEM_PHYS;
		uio->mem[i].addr = res->start & PAGE_MASK;
		uio->mem[i].size = PAGE_ALIGN(resource_size(res));
		if (strstr(res->name, "/") < 0)
			uio->mem[i].name = res->name;
		else {
			tmp_name = devm_kzalloc(dev, 4, GFP_KERNEL);
			sprintf(tmp_name, "%d", i);
			uio->mem[i].name = tmp_name;
		}
	}

	if (!mem_cnt) {
		err = -EIO;
		goto fail_uio;
	}

	err = uio_register_device(dev, uio);
	if (err) {
		dev_err(dev, "Failed to register uio device\n");
		goto fail_uio;
	}

	uio_pdrv_dmax2->map_num = mem_cnt;
	pr_info("Registered uio device having %d register maps attached\n",
		uio_pdrv_dmax2->map_num);

	platform_set_drvdata(pdev, uio_pdrv_dmax2);

	dmax2_cnt++;
	return 0;

fail_uio:
	devm_kfree(dev, uio_pdrv_dmax2);
fail:
	return err;
}

/*
 * mv_dmax2_uio_remove() - mv_dmax2_uio_drv platform driver release routine
 * - unregister uio devices
 *
 */
static int mv_dmax2_uio_remove(struct platform_device *pdev)
{
	struct uio_pdrv_dmax2_info *uio_pdrv_dmax2 = platform_get_drvdata(pdev);

	if (!uio_pdrv_dmax2)
		return -EINVAL;

	uio_unregister_device(&uio_pdrv_dmax2->uio);
	devm_kfree(uio_pdrv_dmax2->dev, uio_pdrv_dmax2);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id mv_dmax2_of_match[] = {
	{ .compatible	= "marvell,uio-xor-v2", },
	{ }
};

static struct platform_driver mv_dmax2_uio_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= DRIVER_NAME,
		.of_match_table	= mv_dmax2_of_match,
	},
	.probe	= mv_dmax2_uio_probe,
	.remove	= mv_dmax2_uio_remove,
};

/* *INDENT-ON* */

module_platform_driver(mv_dmax2_uio_driver);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:" DRIVER_NAME);
