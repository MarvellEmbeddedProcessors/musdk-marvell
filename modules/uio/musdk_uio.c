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
#include <linux/of_reserved_mem.h>

#include <linux/mm.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/cma.h>

#include <linux/dma-contiguous.h>
#include <linux/of_reserved_mem.h>


#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/skbuff.h>

#include "../include/musdk_uio_ioctls.h"


#define DRIVER_NAME	"musdk_uio_drv"
#define DRIVER_VERSION	"1.3"
#define DRIVER_AUTHOR	"ENEA AB"
#define DRIVER_DESC	"UIO platform driver for Armada US SDK"

#define MISC_DEV_NAME	"uio-cma"
#define MISC_REGION_DEV_NAME "uio-cma-reg%u"


#define MAX_UIO_DEVS	3

#define CMA_MAGIC_NUMBER	0xAA55AA55
#define CMA_PAGE_SIZE		4096

/*
 * Administration area used to keep information about each contiguous
 * buffer allocated
 */
struct cma_admin {
	union {
		struct {
			u32			magic;
			/* Physical memory address of buffer allocated */
			phys_addr_t		paddr;
			/* Kernel virtual memory address */
			void			*kvaddr;
			/* size of buffer with admin area */
			size_t			size;
			struct list_head	list;
		};
		u8				rsvd[CMA_PAGE_SIZE];
	};
};


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
	atomic_t refcount;
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

static struct cma_admin *cma_find_admin_by_paddr(struct uio_pdrv_musdk_info *cma_pdrv,
						 phys_addr_t paddr)
{
	struct list_head *node, *q;
	struct cma_admin *adm, *ptr = NULL;

	/* Find cma_admin handle by paddr */
	list_for_each_safe(node, q, &cma_pdrv->cma_client.list) {
		adm = list_entry(node, struct cma_admin, list);
		if (paddr == adm->paddr) {
			ptr = adm;
			break;
		}
	}
	return ptr;
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
	ptr = kmalloc(sizeof(struct cma_admin), GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	size  = PAGE_ALIGN(size);

	/* allocate space from CMA */
	ptr->kvaddr = dma_zalloc_coherent(uio_pdrv_musdk->dev, size, &paddr,
			GFP_KERNEL | GFP_DMA);
	if (!ptr->kvaddr) {
		pr_err("Not enough CMA memory to alloc %lld bytes", size);
		kfree(ptr);
		return -ENOMEM;
	}

	ptr->magic = CMA_MAGIC_NUMBER;
	ptr->paddr = paddr;
	ptr->size  = size;
	ptr->kvaddr = ptr;

	pr_debug("dev (%s) CMA buffer allocated: size = %zd Bytes, kvaddr = %p, paddr = 0x%llx\n",
		dev_name(uio_pdrv_musdk->dev), (size_t)size, ptr->kvaddr, paddr);

	param = (u64)paddr;

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

	pr_debug("CMA buffer released: size = %zd Bytes, kvaddr = %p, paddr = 0x%llx\n",
		(size_t)ptr->size, (void *)ptr->kvaddr, ptr->paddr);

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
	u64 paddr;
	struct cma_admin *ptr = NULL;

	if (copy_from_user(&paddr, argp, sizeof(paddr)))
		return -EFAULT;

	ptr = cma_find_admin_by_paddr(uio_pdrv_musdk, (phys_addr_t)paddr);

	return _cma_free(uio_pdrv_musdk, ptr);
}

/*
 * musdk_cma_open() : open operation for uio cma module driver
 *
 */
static int musdk_cma_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct uio_pdrv_musdk_info *uio_pdrv_musdk =
	    container_of(misc, struct uio_pdrv_musdk_info, misc);

	atomic_inc(&uio_pdrv_musdk->refcount);

	pr_debug("CMA: reference counter: %d\n", atomic_read(&uio_pdrv_musdk->refcount));

	return 0;
}

/*
 * musdk_cma_mmap() - mmap operation for uio cma mapping.
 *
 */
static int musdk_cma_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct miscdevice *misc = filp->private_data;
	struct uio_pdrv_musdk_info *uio_pdrv_musdk =
	    container_of(misc, struct uio_pdrv_musdk_info, misc);
	int err = 0;
	phys_addr_t paddr = vma->vm_pgoff << PAGE_SHIFT;
	struct cma_admin *ptr;

	if (vma->vm_pgoff == 0)
		return -EFAULT;

	/* Find cma_admin handle */
	ptr = cma_find_admin_by_paddr(uio_pdrv_musdk, paddr);

	if (cma_check_buf(ptr)) {
		return -EFAULT;
	}
	vma->vm_pgoff = 0;

	err = dma_mmap_coherent(uio_pdrv_musdk->dev, vma,
				ptr->kvaddr, ptr->paddr, ptr->size);
	if (err) {
		pr_err("Can't mmap CMA buffer. err = %d, size = %zd\n",
			err, ptr->size);
		return err;
	}
	pr_debug("CMA buffer remapped: vm_start=0x%lx, size = %ld bytes, paddr = 0x%llx\n",
		vma->vm_start, vma->vm_end - vma->vm_start, paddr);

	return 0;
}

/*
 * musdk_cma_ioctl() - ioctl operation for uio cma device.
 *
 */
static long musdk_cma_ioctl(struct file *filp,
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
static int musdk_cma_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct uio_pdrv_musdk_info *uio_pdrv_musdk =
	    container_of(misc, struct uio_pdrv_musdk_info, misc);
	struct list_head *node, *q;
	int garbage = 0;
	struct cma_admin *adm;

	pr_debug("CMA: reference counter: %d\n", atomic_read(&uio_pdrv_musdk->refcount));
	if (!atomic_dec_and_test(&uio_pdrv_musdk->refcount))
		return 0;

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
#define UIO_DEFAULT_CMA_MEMORY	(~0)
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct uio_pdrv_musdk_info *uio_pdrv_musdk;
	u32 mem_region_index = UIO_DEFAULT_CMA_MEMORY;
	int err = 0;

	/* number of probes are according to dts.
	 * First pdev is for "uio-cma". Following pdevs are for regions, "uio-cma-reg%u".
	*/
	if (!np) {
		dev_err(dev, "Non DT case is not supported\n");
		return -EINVAL;
	}

	of_property_read_u32(np, "uio-mem-region", &mem_region_index);

	of_reserved_mem_device_init(&pdev->dev);

	uio_pdrv_musdk = devm_kzalloc(dev, sizeof(struct uio_pdrv_musdk_info),
				   GFP_KERNEL);
	if (!uio_pdrv_musdk) {
		err = -ENOMEM;
		goto fail;
	}

	uio_pdrv_musdk->uio_num = -EIO;
	uio_pdrv_musdk->map_num = -EIO;
	uio_pdrv_musdk->dev = dev;
	atomic_set(&uio_pdrv_musdk->refcount, 0);
	mutex_init(&uio_pdrv_musdk->lock);

	if (!dev->archdata.dma_coherent)
		dev_warn(dev, "Not dma_coherent\n");

	dev->dma_mask = kmalloc(sizeof(*dev->dma_mask), GFP_KERNEL);
	err = dma_set_mask(dev, DMA_BIT_MASK(40));
	if (err == 0)
		dma_set_coherent_mask(dev, DMA_BIT_MASK(40));
	if (err) {
		dev_err(dev, "%s:%s ", DEV_MUSDK_NAME, "cannot set dma_mask\n");
		goto fail;
	}

	struct miscdevice *misc;

	misc		= &uio_pdrv_musdk->misc;
	misc->minor	= MISC_DYNAMIC_MINOR;
	if (mem_region_index != UIO_DEFAULT_CMA_MEMORY)
		misc->name = kasprintf(GFP_KERNEL, MISC_REGION_DEV_NAME, mem_region_index);
	else
		misc->name = MISC_DEV_NAME;
	misc->fops	= &musdk_misc_fops;
	misc->parent	= dev;

	err = misc_register(misc);
	if (err) {
		dev_err(dev, "Failed to register cma device\n");
		goto fail;
	}
	struct cma_ctx *ctx = (struct cma_ctx *) &uio_pdrv_musdk->cma_client;

	atomic_set(&ctx->buf_alloc, 0);
	atomic_set(&ctx->buf_free, 0);

	INIT_LIST_HEAD(&ctx->list);
	pr_info("Registered cma device: %s\n", misc->name);

	platform_set_drvdata(pdev, uio_pdrv_musdk);
	return 0;

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
	int idx;

	if (!uio_pdrv_musdk)
		return -EINVAL;

	misc_deregister(&uio_pdrv_musdk->misc);
	pr_debug("Detached -> uio: {%s} devices and misc: {%s} device",
	   uio_pdrv_musdk->uio[0].name, uio_pdrv_musdk->misc.name);

	if (uio_pdrv_musdk->uio_num != -EIO)
		for (idx = 0; idx <= uio_pdrv_musdk->uio_num; ++idx)
			uio_unregister_device(&uio_pdrv_musdk->uio[idx]);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

/* *INDENT-OFF* */
static const struct file_operations musdk_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= musdk_cma_open,
	.mmap		= musdk_cma_mmap,
	.release	= musdk_cma_release,
	.unlocked_ioctl	= musdk_cma_ioctl,
	.compat_ioctl	= musdk_cma_ioctl,
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
