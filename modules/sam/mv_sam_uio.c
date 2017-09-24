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
#include <linux/io.h>
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

/* UIO device per SAM CIO/Ring to support Interrupt per ring */
#define MAX_UIO_DEVS			4

#define SAM_EIP197_xDR_REGS_OFFS	0x80000
#define SAM_EIP97_xDR_REGS_OFFS		0x0

#define SAM_RING_OFFS(ring)		((ring) * 0x1000)

#define HIA_RDR_REGS_OFFSET		0x800
#define HIA_RDR_STAT_REG		(HIA_RDR_REGS_OFFSET + 0x3C)

/* HIA_RDR_y_STAT register */
#define SAM_RDR_STAT_IRQ_OFFS		0
#define SAM_RDR_STAT_IRQ_BITS		8
#define SAM_RDR_STAT_IRQ_MASK		GENMASK(SAM_RDR_STAT_IRQ_OFFS + SAM_RDR_STAT_IRQ_BITS, SAM_RDR_STAT_IRQ_OFFS)

struct sam_uio_ring_info {
	int ring;
	char ring_name[32];
	char *xdr_regs_vbase;
	struct uio_info uio;
};

/*
 * sam_uio_pdev_info
 * local information for uio module driver
 *
 * @uio_num:  number of uio devices
 * @dev:      device pointer
 * @info:     uio_info array
 *
 */
struct sam_uio_pdev_info {
	int uio_num;
	struct device *dev;
	char name[16];
	char *regs_vbase;
	struct sam_uio_ring_info rings[MAX_UIO_DEVS];
};


static int sam_uio_remove(struct platform_device *pdev);

static inline void sam_uio_ring_ack_irq(struct sam_uio_ring_info *ring_priv, u32 mask)
{
	u32 *addr, val32;

	addr = (u32 *)(ring_priv->xdr_regs_vbase + HIA_RDR_STAT_REG);

	val32 = mask & SAM_RDR_STAT_IRQ_MASK;

	/* pr_debug("%s: writel: %p = 0x%08x\n", __func__, addr, val32); */
	writel(val32, addr);
}

irqreturn_t sam_ring_irq_handler(int irq, struct uio_info *dev_info)
{
	struct sam_uio_ring_info *ring_priv;

	ring_priv = (struct sam_uio_ring_info *)dev_info->priv;

	/* Ack all interrupts */
	sam_uio_ring_ack_irq(ring_priv, SAM_RDR_STAT_IRQ_MASK);

	return IRQ_HANDLED;
}

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
	int err = 0;
	char irq_name[6] = {0};
	char *xdr_regs_vbase;
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
	platform_set_drvdata(pdev, pdev_info);

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

	res = platform_get_resource(eip_pdev, IORESOURCE_MEM, 0);
	if (!res)
		goto fail_uio;

	pdev_info->regs_vbase = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(pdev_info->regs_vbase)) {
		dev_err(dev, "failed to get resource\n");
		goto fail_uio;
	}
	if (!strcmp(eip_node->name, "eip197"))
		xdr_regs_vbase = pdev_info->regs_vbase + SAM_EIP197_xDR_REGS_OFFS;
	else if (!strcmp(eip_node->name, "eip97"))
		xdr_regs_vbase = pdev_info->regs_vbase + SAM_EIP97_xDR_REGS_OFFS;
	else {
		dev_err(dev, "unknown HW type\n");
		goto fail_uio;
	}

	snprintf(pdev_info->name, sizeof(pdev_info->name), "uio_%s_%d",  eip_node->name, cpn_count);

	for (int idx = 0; idx < MAX_UIO_DEVS; idx++) {
		struct sam_uio_ring_info *ring_info = &pdev_info->rings[idx];

		uio = &ring_info->uio;

		snprintf(ring_info->ring_name, sizeof(ring_info->ring_name), "%s:%d",  pdev_info->name, idx);
		ring_info->ring = idx;
		ring_info->xdr_regs_vbase = xdr_regs_vbase + SAM_RING_OFFS(ring_info->ring);
		uio->name = ring_info->ring_name;
		uio->version = DRIVER_VERSION;

		snprintf(irq_name, sizeof(irq_name), "ring%d", idx);
		uio->irq = platform_get_irq_byname(eip_pdev, irq_name);
		if (uio->irq < 0) {
			dev_warn(dev, "unable to get IRQ number for '%s'\n", irq_name);
		} else {
			pr_debug("%s - irq #%ld\n", irq_name, uio->irq);
			uio->irq_flags = IRQF_SHARED;
			uio->handler = sam_ring_irq_handler;
		}
		uio->priv = (void *)ring_info;

		uio->mem[0].memtype = UIO_MEM_PHYS;
		uio->mem[0].addr = res->start & PAGE_MASK;
		uio->mem[0].size = PAGE_ALIGN(resource_size(res));
		uio->mem[0].name = "regs";

		err = uio_register_device(dev, uio);
		if (err) {
			dev_err(dev, "Failed to register uio device\n");
			goto fail_uio;
		}
		pdev_info->uio_num++;
	}
	dev_info(dev, "Registered %d uio devices\n", pdev_info->uio_num);

	cpn_count++;
	return 0;

fail_uio:
	sam_uio_remove(pdev);
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
		for (int idx = 0; idx < pdev_info->uio_num; idx++)
			uio_unregister_device(&pdev_info->rings[idx].uio);
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
