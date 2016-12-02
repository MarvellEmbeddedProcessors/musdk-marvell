/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software
 *	  without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

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

#include "../include/musdk_uio_ioctls.h"


#define DRIVER_NAME	"musdk_uio_drv"
#define DRIVER_VERSION	"1.3"
#define DRIVER_AUTHOR	"ENEA AB"
#define DRIVER_DESC	"UIO platform driver for Armada US SDK"

#define MISC_DEV_NAME	"uio-cma"

#define MAX_UIO_DEVS	3


/* CMA information about buffers - used by garbage collection */
struct cma_ctx {
	atomic_t buf_free;
	atomic_t buf_alloc;

	/* List with allocated buffers */
	struct list_head list;
};

/*
 * uio_pdrv_musdk_info
 * local information for uio module driver
 *
 * @uio_num:  number of uio devices
 * @map_num:  number of uio memory regions
 * @lock:     lock to protect global resources
 * @dev:      device pointer
 * @misc:     misc device
 * @info:     uio_info array
 *
 */
struct uio_pdrv_musdk_info {
	int uio_num;
	int map_num;
	/* lock for global resources */
	struct mutex lock;
	struct device *dev;
	struct miscdevice misc;
	struct uio_info uio[MAX_UIO_DEVS];
	struct cma_ctx cma_client;
};


static const struct file_operations musdk_misc_fops;


static int cma_check_buf(struct cma_admin *ptr)
{
	if (!ptr) {
		pr_err("This addr %p is not valid\n", ptr);
		return -EFAULT;
	}

	if (ptr->magic != CMA_MAGIC_NUMBER) {
		pr_err("Wrong magic number detected: %p (%X)", ptr, ptr->magic);
		return -EFAULT;
	}

	return 0;
}

/*
 * cma_calloc() - CMA allocator
 *
 */
static int cma_calloc(struct uio_pdrv_musdk_info *uio_pdrv_musdk, void *argp)
{
	struct cma_admin *ptr;
	u64 paddr;
	u64 param;
	u64 size;
	struct cma_ctx *ctx = &uio_pdrv_musdk->cma_client;

	if (copy_from_user(&size, argp, sizeof(size))) {
		pr_err("Copy size arg %p from user failed", argp);
		return -EFAULT;
	}

	size  = PAGE_ALIGN(size);

	/* allocate space from CMA */
	ptr = dma_zalloc_coherent(uio_pdrv_musdk->dev, size, &paddr,
			GFP_KERNEL | GFP_DMA);
	if (!ptr) {
		pr_err("Not enough CMA memory to alloc %lld bytes", size);
		return -ENOMEM;
	}

	ptr->magic = CMA_MAGIC_NUMBER;
	ptr->paddr = paddr;
	ptr->size  = size;
	ptr->kvaddr = (u64)ptr;

	param = (u64)ptr;

	pr_debug("Alloc %p = %lld Bytes\n", (void *)ptr, size);

	if (copy_to_user(argp, &param, sizeof(param))) {
		dma_free_coherent(uio_pdrv_musdk->dev, ptr->size,
				(void *)ptr->kvaddr,
				ptr->paddr);
		pr_err("Return physical address of buffer (%p) to user failed",
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
static int _cma_free(struct uio_pdrv_musdk_info *uio_pdrv_musdk,
		struct cma_admin *ptr)
{
	struct cma_ctx *ctx = &uio_pdrv_musdk->cma_client;

	if (cma_check_buf(ptr)) {
		return -EFAULT;
	}

	/* To avoid double free on the same buffer */
	ptr->magic = ~CMA_MAGIC_NUMBER;

	pr_debug("CMA released: %p = %ld Bytes\n", (void *)ptr->paddr, ptr->size);

	atomic_inc(&ctx->buf_free);
	list_del(&ptr->list);

	dma_free_coherent(uio_pdrv_musdk->dev, ptr->size, (void *)ptr->kvaddr,
			ptr->paddr);

	return 0;
}

/*
 * cma_free() - cma release handler
 *
 */
static int cma_free(struct uio_pdrv_musdk_info *uio_pdrv_musdk, void *argp)
{
	struct cma_admin *ptr;

	if (copy_from_user(&ptr, argp, sizeof(struct cma_admin *)))
		return -EFAULT;

	return _cma_free(uio_pdrv_musdk, ptr);
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
	struct uio_pdrv_musdk_info *uio_pdrv_musdk =
	    container_of(misc, struct uio_pdrv_musdk_info, misc);
	int err = 0;

	/* vm_pgoff is multiple of page size  */
	struct cma_admin *ptr = (struct cma_admin *)
					(((u64)vma->vm_pgoff) << 12);

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

	if (vma->vm_flags & VM_WRITE) {
		err = dma_mmap_coherent(uio_pdrv_musdk->dev, vma,
				(void *)((u64)ptr->kvaddr + CMA_PAGE_SIZE),
				ptr->paddr + CMA_PAGE_SIZE,
				ptr->size - CMA_PAGE_SIZE);
		if (err)
			return err;
		else {
			ptr->uvaddr = vma->vm_start;
			pr_debug("Mapped payload vaddr: %p, paddr: %p = %ld Bytes\n",
					(void *)ptr->uvaddr,
					(void *)ptr->kvaddr,
					ptr->size);
		}
	} else {
		/* The user map signal admin area only */
		err = dma_mmap_coherent(uio_pdrv_musdk->dev, vma,
					(void *)ptr->kvaddr,
					ptr->paddr, CMA_PAGE_SIZE);
		if (err)
			return err;
		pr_debug("Mapped admin %p = %d Bytes\n", (void *)ptr->paddr,
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
	struct uio_pdrv_musdk_info *uio_pdrv_musdk =
	    container_of(misc, struct uio_pdrv_musdk_info, misc);
	int err = 0;

	if (_IOC_TYPE(cmd) != MUSDK_IOC_TYPE_BASE) {
		pr_err("ioctl(): bad command type 0x%x (should be 0x%x)\n",
		   _IOC_TYPE(cmd), MUSDK_IOC_TYPE_BASE);
		return -EIO;
	}

	switch (cmd) {
		case MUSDK_IOC_CMA_ALLOC:
			err = cma_calloc(uio_pdrv_musdk, argp);
			break;

		case MUSDK_IOC_CMA_FREE:
			err = cma_free(uio_pdrv_musdk, argp);
			break;
		default:
			pr_err("Unknown ioctl {0x%x} received.\n", cmd);
			return -EINVAL;
	}

	return err;
}

/*
 * cma_release() - release operation for uio misc device.
 *
 */
static int cma_release(struct inode *inode, struct file *filp)
{

	struct miscdevice *misc = filp->private_data;
	struct uio_pdrv_musdk_info *uio_pdrv_musdk =
	    container_of(misc, struct uio_pdrv_musdk_info, misc);
	struct list_head *node, *q;
	int garbage = 0;
	struct cma_admin *adm;

	list_for_each_safe(node, q, &uio_pdrv_musdk->cma_client.list) {
		adm = list_entry(node, struct cma_admin, list);
		if (_cma_free(uio_pdrv_musdk, adm)) {
			pr_err("Buffer corrupt detected %p", adm);
			break;
		}
		garbage++;
	}

	int free = atomic_read(&uio_pdrv_musdk->cma_client.buf_free);
	int alloc = atomic_read(&uio_pdrv_musdk->cma_client.buf_alloc);

	pr_debug("CMA: total alloc %d, total free: %d (garbage %d)",
			alloc, free, garbage);

	if (free != alloc)
		pr_err("There are %d  alien buffers", alloc - free);

	return 0;
}

/*
 * musdk_uio_probe() - musdk_uio_drv platform driver probe routine
 * - register uio devices filled with memory maps retrieved from device tree
 * - register misc device for CMA memory pools
 *
 */
static int musdk_uio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct uio_pdrv_musdk_info *uio_pdrv_musdk;
	struct uio_info *uio;
	struct resource *res;
	int err = 0, mem_cnt = 0;

	if (!np) {
		dev_err(dev, "Non DT case is not supported\n");
		return -EINVAL;
	}

	uio_pdrv_musdk = devm_kzalloc(dev, sizeof(struct uio_pdrv_musdk_info),
				   GFP_KERNEL);
	if (!uio_pdrv_musdk) {
		err = -ENOMEM;
		goto fail;
	}

	uio_pdrv_musdk->uio_num = -EIO;
	uio_pdrv_musdk->map_num = -EIO;
	uio_pdrv_musdk->dev = dev;
	mutex_init(&uio_pdrv_musdk->lock);

	if (!dev->archdata.dma_coherent)
		dev_warn(dev, "Not dma_coherent\n");

	dev->dma_mask = kmalloc(sizeof(*dev->dma_mask), GFP_KERNEL);
	err = dma_set_mask(dev, DMA_BIT_MASK(32));
	if (err == 0)
		dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (err) {
		dev_err(dev, "mv_pp_uio: cannot set dma_mask\n");
		goto fail;
	}

	for (int idx = 0; idx < MAX_UIO_DEVS; ++idx) {
		uio = &uio_pdrv_musdk->uio[idx];
		uio->name = DEV_MUSDK_NAME;
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

		if (!mem_cnt)
			continue;

		err = uio_register_device(dev, uio);
		if (err) {
			dev_err(dev, "Failed to register uio device\n");
			goto fail_uio;
		}

		uio_pdrv_musdk->uio_num = idx;

		if (!res)
			break;
	}

	uio_pdrv_musdk->map_num = mem_cnt;
	if (uio_pdrv_musdk->uio_num != -EIO)
		pr_info("Registered %d uio devices, having %d register maps attached\n",
			uio_pdrv_musdk->uio_num + 1, uio_pdrv_musdk->map_num);

	struct miscdevice *misc;

	/* *INDENT-OFF* */
	misc		= &uio_pdrv_musdk->misc;
	misc->minor	= MISC_DYNAMIC_MINOR;
	misc->name	= MISC_DEV_NAME;
	misc->fops	= &musdk_misc_fops;
	misc->parent	= dev;
	/* *INDENT-ON* */

	err = misc_register(misc);
	if (err) {
		dev_err(dev, "Failed to register cma device\n");
		goto fail_misc;
	}
	struct cma_ctx *ctx = (struct cma_ctx *) &uio_pdrv_musdk->cma_client;

	atomic_set(&ctx->buf_alloc, 0);
	atomic_set(&ctx->buf_free, 0);

	INIT_LIST_HEAD(&ctx->list);
	pr_info("Registered cma device: %s\n", misc->name);

	platform_set_drvdata(pdev, uio_pdrv_musdk);

	return 0;

fail_misc:
	for (int idx = 0; idx <= uio_pdrv_musdk->uio_num; ++idx)
		uio_unregister_device(&uio_pdrv_musdk->uio[idx]);
fail_uio:
	devm_kfree(dev, uio_pdrv_musdk);
fail:
	return err;
}

/*
 * musdk_uio_remove() - musdk_uio_drv platform driver release routine
 * - unregister uio devices
 * - unregister misc device
 *
 */
static int musdk_uio_remove(struct platform_device *pdev)
{
	struct uio_pdrv_musdk_info *uio_pdrv_musdk = platform_get_drvdata(pdev);

	if (!uio_pdrv_musdk)
		return -EINVAL;

	misc_deregister(&uio_pdrv_musdk->misc);
	pr_debug("Detached -> uio: {%s} devices and misc: {%s} device",
	   uio_pdrv_musdk->uio[0].name, uio_pdrv_musdk->misc.name);

	if (uio_pdrv_musdk->uio_num != -EIO)
		for (int idx = 0; idx <= uio_pdrv_musdk->uio_num; ++idx)
			uio_unregister_device(&uio_pdrv_musdk->uio[idx]);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

/* *INDENT-OFF* */
static const struct file_operations musdk_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= cma_open,
	.mmap		= cma_mmap,
	.release	= cma_release,
	.unlocked_ioctl	= cma_ioctl,
};

static const struct of_device_id musdk_of_match[] = {
	{ .compatible	= "marvell,musdk-uio", },
	{ }
};

static struct platform_driver musdk_uio_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= DRIVER_NAME,
		.of_match_table	= musdk_of_match,
	},
	.probe	= musdk_uio_probe,
	.remove	= musdk_uio_remove,
};

/* *INDENT-ON* */

module_platform_driver(musdk_uio_driver);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:" DRIVER_NAME);
