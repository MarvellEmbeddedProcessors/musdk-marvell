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
#include <linux/miscdevice.h>
#include <linux/uio_driver.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/mm.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/skbuff.h>



#include "../include/mv_pp_uio.h"


#define _D(fmt, ...) \
	pr_debug("%s -> %s: " fmt, DRIVER_NAME, __func__, __VA_ARGS__)
#define _E(fmt, ...) \
	pr_err("%s -> %s: " fmt, DRIVER_NAME, __func__, __VA_ARGS__)

#define DRIVER_NAME	"mv_pp_uio_drv"
#define DRIVER_VERSION	"1.3"
#define DRIVER_AUTHOR	"ENEA AB"
#define DRIVER_DESC	"UIO platform driver for Armada Packet Processor"

#define MISC_DEV_NAME	"uio-cma"

#define MAX_UIO_DEVS	3

static const struct file_operations pp_misc_fops;

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
 * @misc:     misc device
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
	struct miscdevice misc;
	struct uio_info uio[MAX_UIO_DEVS];
	struct cma_ctx cma_client;
};

/*
 * mv_pp_clk_unbind() - enable all clks for pp device
 *
 */
static int mv_pp_clk_bind(struct uio_pdrv_pp_info *uio_pdrv)
{
	struct pp_hw *hw;
	int err = 0;

	hw = &uio_pdrv->hw;

	hw->gop_core_clk = devm_clk_get(uio_pdrv->dev, "gop_core_clk");
	if (IS_ERR(hw->gop_core_clk))
		return PTR_ERR(hw->gop_core_clk);
	err = clk_prepare_enable(hw->gop_core_clk);
	if (err < 0)
		goto err_clk;

	hw->gop_clk = devm_clk_get(uio_pdrv->dev, "gop_clk");
	if (IS_ERR(hw->gop_clk))
		return PTR_ERR(hw->gop_clk);
	err = clk_prepare_enable(hw->gop_clk);
	if (err < 0)
		goto err_clk;

	hw->mg_core_clk = devm_clk_get(uio_pdrv->dev, "mg_core_clk");
	if (IS_ERR(hw->mg_clk))
		return PTR_ERR(hw->mg_core_clk);
	err = clk_prepare_enable(hw->mg_core_clk);
	if (err < 0)
		goto err_clk;

	hw->mg_clk = devm_clk_get(uio_pdrv->dev, "mg_clk");
	if (IS_ERR(hw->mg_clk))
		return PTR_ERR(hw->mg_clk);
	err = clk_prepare_enable(hw->mg_clk);
	if (err < 0)
		goto err_clk;

	hw->pp_clk = devm_clk_get(uio_pdrv->dev, "pp_clk");
	if (IS_ERR(hw->pp_clk))
		return PTR_ERR(hw->pp_clk);
	err = clk_prepare_enable(hw->pp_clk);
	if (err < 0)
		goto err_clk;

err_clk:
	return err;
}

/*
 * mv_pp_clk_unbind() - disable all clks for pp device
 *
 */
static void mv_pp_clk_unbind(struct pp_hw *hw)
{
	clk_disable_unprepare(hw->pp_clk);
	clk_disable_unprepare(hw->gop_core_clk);
	clk_disable_unprepare(hw->gop_clk);
	clk_disable_unprepare(hw->mg_core_clk);
	clk_disable_unprepare(hw->mg_clk);
}

static int cma_check_buf(struct cma_admin *ptr)
{
	if (!ptr) {
		_E("This addr %p is not valid\n", ptr);
		return -EFAULT;
	}

	if (ptr->magic != CMA_MAGIC_NUMBER) {
		_E("Wrong magic number detected: %p (%X)", ptr, ptr->magic);
		return -EFAULT;
	}

	return 0;
}

/*
 * cma_calloc() - CMA allocator
 *
 */
static int cma_calloc(struct uio_pdrv_pp_info *uio_pdrv_pp, void *argp)
{
	struct cma_admin *ptr;
	uint64_t paddr;
	uint64_t param;
	uint64_t size;
	struct cma_ctx *ctx = &uio_pdrv_pp->cma_client;

	if (copy_from_user(&size, argp, sizeof(size))) {
		_E("Copy size arg %p from user failed", argp);
		return -EFAULT;
	}

	size  = PAGE_ALIGN(size);

	/* allocate space from CMA */
	ptr = dma_zalloc_coherent(uio_pdrv_pp->dev, size, &paddr,
			GFP_KERNEL | GFP_DMA);
	if (!ptr) {
		_E("Not enough CMA memory to alloc %lld bytes", size);
		return -ENOMEM;
	}

	ptr->magic = CMA_MAGIC_NUMBER;
	ptr->paddr = paddr;
	ptr->size  = size;
	ptr->kvaddr = (uint64_t)ptr;

	param = (uint64_t)ptr;

	_D("Alloc %p = %lld Bytes\n", (void *)ptr, size);

	if (copy_to_user(argp, &param, sizeof(param)))
	{
		dma_free_coherent(uio_pdrv_pp->dev, ptr->size,
				(void *)ptr->kvaddr,
				ptr->paddr);
		_E("Return physical address of buffer (%p) to user failed",
				(void *)param);
		return -EFAULT;
	}

	atomic_inc(&ctx->buf_alloc);

	INIT_LIST_HEAD(&ptr->list);

	list_add(&ptr->list, &ctx->list);

	return 0;
}

/*
 * _cma_free() - internal cma release handler
 *
 */
static int _cma_free(struct uio_pdrv_pp_info *uio_pdrv_pp,
		struct cma_admin *ptr)
{
	struct cma_ctx *ctx = &uio_pdrv_pp->cma_client;

	if (cma_check_buf(ptr)) {
		return -EFAULT;
	}

	/* To avoid double free on the same buffer */
	ptr->magic = ~CMA_MAGIC_NUMBER;

	_D("CMA released: %p = %ld Bytes\n", (void *)ptr->paddr, ptr->size);

	atomic_inc(&ctx->buf_free);
	list_del(&ptr->list);

	dma_free_coherent(uio_pdrv_pp->dev, ptr->size, (void *)ptr->kvaddr,
			ptr->paddr);

	return 0;
}

/*
 * cma_free() - cma release handler
 *
 */
static int cma_free(struct uio_pdrv_pp_info *uio_pdrv_pp, void *argp)
{
	struct cma_admin *ptr;

	if (copy_from_user(&ptr, argp, sizeof(struct cma_admin *)))
		return -EFAULT;

	return _cma_free(uio_pdrv_pp, ptr);
}

/*
 * cma_open() : open operation for uio cma module driver
 *
 */
static int cma_open(struct inode *inode, struct file *file)
{
	return 0;
}

/*
 * cma_mmap() - mmap operation for uio cma mapping.
 *
 */
static int cma_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct miscdevice *misc = filp->private_data;
	struct uio_pdrv_pp_info *uio_pdrv_pp =
	    container_of(misc, struct uio_pdrv_pp_info, misc);
	int err = 0;

	/* vm_pgoff is multiple of page size  */
	struct cma_admin *ptr = (struct cma_admin *)
					(((uint64_t)vma->vm_pgoff) << 12);

	if (vma->vm_pgoff == 0)
		return -EFAULT;

	if (cma_check_buf(ptr)) {
		return -EFAULT;
	}

	/*
	 * vm_pgoff was used to encode kernel virtual address of admin area,
	 * its value must be set to zero in order to avoid -ENXIO in
	 * dma_mmap_coherent().
	 */
	vma->vm_pgoff = 0;

	if (vma->vm_flags & VM_WRITE)
	{
		err = dma_mmap_coherent(uio_pdrv_pp->dev, vma,
				(void *)((uint64_t)ptr->kvaddr + CMA_PAGE_SIZE),
				ptr->paddr + CMA_PAGE_SIZE,
				ptr->size - CMA_PAGE_SIZE);
		if (err) {
			return err;
		}
		else {
			ptr->uvaddr = vma->vm_start;
			_D("Mapped payload vaddr: %p, paddr: %p = %ld Bytes\n",
					(void *)ptr->uvaddr,
					(void *)ptr->kvaddr,
					ptr->size);
		}
	}
	else {
		/* The user map signal admin area only */
		err = dma_mmap_coherent(uio_pdrv_pp->dev, vma,
					(void *)ptr->kvaddr,
					ptr->paddr, CMA_PAGE_SIZE);
		if (err) {
			return err;
		}
		_D("Mapped admin %p = %d Bytes\n", (void *)ptr->paddr,
				CMA_PAGE_SIZE);
	}

	return 0;
}

/*
 * cma_ioctl() - ioctl operation for uio cma device.
 *
 */
static long cma_ioctl(struct file *filp,
			  unsigned int cmd, unsigned long args)
{
	void __user *argp = (void __user *)args;
	struct miscdevice *misc = filp->private_data;
	struct uio_pdrv_pp_info *uio_pdrv_pp =
	    container_of(misc, struct uio_pdrv_pp_info, misc);
	int err = 0;

	if (_IOC_TYPE(cmd) != _IOC_TYPE(PP_UIO_IOC_KEY)) {
		_E("ioctl(): bad command type 0x%x (should be 0x%x)\n",
		   _IOC_TYPE(cmd), _IOC_TYPE(PP_UIO_IOC_KEY));
	}

	switch (cmd & PP_UIO_IOC_MASK) {
		case PP_IOC_CMA_ALLOC:
			err = cma_calloc(uio_pdrv_pp, argp);
			break;

		case PP_IOC_CMA_FREE:
			err = cma_free(uio_pdrv_pp, argp);
			break;
		default:
			_E("Unknown ioctl {0x%x} received.\n", cmd);
			return -EINVAL;
	}

	return err;
}

/*
 * pp_misc_release() - release operation for uio misc device.
 *
 */
static int cma_release(struct inode *inode, struct file *filp)
{

	struct miscdevice *misc = filp->private_data;
	struct uio_pdrv_pp_info *uio_pdrv_pp =
	    container_of(misc, struct uio_pdrv_pp_info, misc);
	struct list_head *node, *q;
	int garbage = 0;
	struct cma_admin *adm;


	list_for_each_safe(node, q, &uio_pdrv_pp->cma_client.list) {
		adm = list_entry(node, struct cma_admin, list);
		if (_cma_free(uio_pdrv_pp, adm)) {
			_E("Buffer corrupt detected %p", adm);
			break;
		}
		garbage++;
	}

	int free = atomic_read(&uio_pdrv_pp->cma_client.buf_free);
	int alloc = atomic_read(&uio_pdrv_pp->cma_client.buf_alloc);

	_D("CMA: total alloc %d, total free: %d (garbage %d)",
			alloc, free, garbage);

	if (free != alloc) {
		_E("There are %d  alien buffers", alloc - free);
	}

	return 0;
}

/*
 * mv_pp_uio_probe() - mv_pp_uio_drv platform driver probe routine
 * - enable pp clocks
 * - register uio devices filled with memory maps retrieved from device tree
 * - register misc device for CMA memory pools
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
	err = mv_pp_clk_bind(uio_pdrv_pp);

	if (!dev->archdata.dma_coherent) {
		dev_err(dev, "Not dma_coherent\n");
	}

	pdev->dev.dma_mask = kmalloc(sizeof(*pdev->dev.dma_mask), GFP_KERNEL);
	err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (err == 0)
		dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(&pdev->dev, "mv_pp_uio: cannot set dma_mask\n");
	}

	if (err)
		goto fail;

	for (int idx = 0; idx < MAX_UIO_DEVS; ++idx) {
		uio = &uio_pdrv_pp->uio[idx];
		uio->name = (cpn_count == 0) ? UIO_PP_0 : UIO_PP_1;
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
	_E("Registered %d uio devices, having %d register maps attached\n",
	   uio_pdrv_pp->uio_num + 1, uio_pdrv_pp->map_num);

	struct miscdevice *misc;

	if (uio_pdrv_pp->map_num > 0 && !cpn_count) {
		/* *INDENT-OFF* */
		misc		= &uio_pdrv_pp->misc;
		misc->minor	= MISC_DYNAMIC_MINOR;
		misc->name	= MISC_DEV_NAME;
		misc->fops	= &pp_misc_fops;
		misc->parent	= dev;
		/* *INDENT-ON* */

		err = misc_register(misc);
		if (err) {
			dev_err(dev, "Failed to register cma device\n");
			goto fail_misc;
		}
		struct cma_ctx *ctx = (struct cma_ctx *) &uio_pdrv_pp->cma_client;

		atomic_set(&ctx->buf_alloc, 0);
		atomic_set(&ctx->buf_free, 0);

		INIT_LIST_HEAD(&ctx->list);
		_D("Registered cma device: %s\n", misc->name);
	}
	platform_set_drvdata(pdev, uio_pdrv_pp);

	cpn_count++;
	return 0;

fail_misc:
	for (int idx = 0; idx <= uio_pdrv_pp->uio_num; ++idx)
		uio_unregister_device(&uio_pdrv_pp->uio[idx]);
fail_uio:
	mv_pp_clk_unbind(&uio_pdrv_pp->hw);
	devm_kfree(dev, uio_pdrv_pp);
fail:
	return err;
}

/*
 * mv_pp_uio_remove() - mv_pp_uio_drv platform driver release routine
 * - disable pp clocks
 * - unregister uio devices
 * - unregister misc device
 *
 */
static int mv_pp_uio_remove(struct platform_device *pdev)
{
	struct uio_pdrv_pp_info *uio_pdrv_pp = platform_get_drvdata(pdev);

	if (!uio_pdrv_pp)
		return -EINVAL;

	mv_pp_clk_unbind(&uio_pdrv_pp->hw);

	if (uio_pdrv_pp->uio_num != -EIO)
		for (int idx = 0; idx <= uio_pdrv_pp->uio_num; ++idx)
			uio_unregister_device(&uio_pdrv_pp->uio[idx]);
	if (uio_pdrv_pp->map_num > 0)
		misc_deregister(&uio_pdrv_pp->misc);

	_E("Detached -> uio: {%s} devices and misc: {%s} device",
	   uio_pdrv_pp->uio[0].name, uio_pdrv_pp->misc.name);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

/* *INDENT-OFF* */
static const struct file_operations pp_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= cma_open,
	.mmap		= cma_mmap,
	.release	= cma_release,
	.unlocked_ioctl	= cma_ioctl,
};

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
