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

#include "../include/mv_pp_uio.h"


#define _D(fmt, ...) \
	pr_debug("%s -> %s: " fmt, DRIVER_NAME, __func__, __VA_ARGS__)
#define _E(fmt, ...) \
	pr_err("%s -> %s: " fmt, DRIVER_NAME, __func__, __VA_ARGS__)

#define DRIVER_NAME	"mv_pp_uio_drv"
#define DRIVER_VERSION	"1.3"
#define DRIVER_AUTHOR	"ENEA AB"
#define DRIVER_DESC	"UIO platform driver for Armada Packet Processor"

#define MAX_UIO_DEVS	3

struct pp_hw {
	/* Common clocks */
	struct clk *pp_clk;
	struct clk *gop_clk;
	struct clk *gop_core_clk;
	struct clk *mg_clk;
	struct clk *mg_core_clk;
};

/* CMA information about buffers - used by garbage collection */
struct cma_ctx {
	atomic_t buf_free;
	atomic_t buf_alloc;

	/* List with allocated buffers */
	struct list_head list;
};

/*
 * uio_pdrv_pp_info
 * local information for uio module driver
 *
 * @uio_num:  number of uio devices
 * @map_num:  number of uio memory regions
 * @hw:       pp_hw info - clk, phy
 * @lock:     lock to protect global resources
 * @dev:      device pointer
 * @info:     uio_info array
 *
 */
struct uio_pdrv_pp_info {
	int uio_num;
	int map_num;
	struct pp_hw hw;
	/* lock for global resources */
	struct mutex lock;
	struct device *dev;
	struct uio_info uio[MAX_UIO_DEVS];
	struct cma_ctx cma_client;
};


/*
 * mv_pp_uio_probe() - mv_pp_uio_drv platform driver probe routine
 * - enable pp clocks
 * - register uio devices filled with memory maps retrieved from device tree
 *
 */
static int mv_pp_uio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct uio_pdrv_pp_info *uio_pdrv_pp;
	struct uio_info *uio;
	struct resource *res;
	int err = 0, mem_cnt = 0;
	static u8 cpn_count = 0;
	char temp_buff[20];

	if (!np) {
		dev_err(dev, "Non DT case is not supported\n");
		return -EINVAL;
	}

	uio_pdrv_pp = devm_kzalloc(dev, sizeof(struct uio_pdrv_pp_info),
				   GFP_KERNEL);
	if (!uio_pdrv_pp) {
		err = -ENOMEM;
		goto fail;
	}

	uio_pdrv_pp->uio_num = -EIO;
	uio_pdrv_pp->map_num = -EIO;
	uio_pdrv_pp->dev = dev;
	mutex_init(&uio_pdrv_pp->lock);

	for (int idx = 0; idx < MAX_UIO_DEVS; ++idx) {
		uio = &uio_pdrv_pp->uio[idx];
		snprintf(temp_buff, sizeof(temp_buff), UIO_PP_HDR, cpn_count);
		uio->name = devm_kstrdup(dev, temp_buff, GFP_KERNEL);
		uio->version = DRIVER_VERSION;

		for (int i = 0; i < MAX_UIO_MAPS; ++i, ++mem_cnt) {
			res =
			    platform_get_resource(pdev, IORESOURCE_MEM,
						  mem_cnt);
			if (!res)
				break;
			uio->mem[i].memtype = UIO_MEM_PHYS;
			uio->mem[i].addr = res->start & PAGE_MASK;
			uio->mem[i].size = PAGE_ALIGN(resource_size(res));
			uio->mem[i].name = res->name;
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

		uio_pdrv_pp->uio_num = idx;

		if (!res)
			break;
	}
	uio_pdrv_pp->map_num = mem_cnt;
	pr_info("Registered %d uio devices, having %d register maps attached\n",
	   uio_pdrv_pp->uio_num + 1, uio_pdrv_pp->map_num);

	platform_set_drvdata(pdev, uio_pdrv_pp);

	cpn_count++;
	return 0;

fail_uio:
	devm_kfree(dev, uio_pdrv_pp);
fail:
	return err;
}

/*
 * mv_pp_uio_remove() - mv_pp_uio_drv platform driver release routine
 * - disable pp clocks
 * - unregister uio devices
 *
 */
static int mv_pp_uio_remove(struct platform_device *pdev)
{
	struct uio_pdrv_pp_info *uio_pdrv_pp = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	if (!uio_pdrv_pp)
		return -EINVAL;

	if (uio_pdrv_pp->uio_num != -EIO) {
		for (int idx = 0; idx <= uio_pdrv_pp->uio_num; ++idx) {
			devm_kfree(dev, (void *)uio_pdrv_pp->uio[idx].name);
			uio_unregister_device(&uio_pdrv_pp->uio[idx]);
		}
	}
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id mv_pp_of_match[] = {
	{ .compatible	= "marvell,mv-pp-uio", },
	{ }
};

static struct platform_driver mv_pp_uio_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= DRIVER_NAME,
		.of_match_table	= mv_pp_of_match,
	},
	.probe	= mv_pp_uio_probe,
	.remove	= mv_pp_uio_remove,
};

/* *INDENT-ON* */

module_platform_driver(mv_pp_uio_driver);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:" DRIVER_NAME);
