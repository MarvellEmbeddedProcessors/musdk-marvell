/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_reserved_mem.h>
#include <linux/version.h>

#include <linux/mm.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/cma.h>

#include <linux/of_reserved_mem.h>

#include <linux/dma-map-ops.h>

#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/skbuff.h>

#include "../include/musdk_cma_ioctls.h"


#define DRIVER_NAME	"musdk_cma"
#define DRIVER_VERSION	"1.0"
#define DRIVER_AUTHOR	"MARVELL"
#define DRIVER_DESC	"Platform driver for Armada US SDK CMA Memory Mngmt"

#define MISC_DEV_NAME	"musdk-cma"
#define MISC_REGION_DEV_NAME "musdk-cma-reg%u"


#define MAX_UIO_DEVS	3
#define MAX_MUSDK_DEVS	3
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

struct musdk_cma {
	struct miscdevice misc;
	struct device *dev;
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

static struct cma_admin *cma_find_admin_by_paddr(struct musdk_cma *cma_mem,
						 phys_addr_t paddr)
{
	struct list_head *node, *q;
	struct cma_admin *adm, *ptr = NULL;

	/* Find cma_admin handle by paddr */
	list_for_each_safe(node, q, &cma_mem->cma_client.list) {
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
static int cma_calloc(struct musdk_cma *cma_mem, void *argp)
{
	struct cma_admin *ptr;
	u64 paddr;
	u64 param;
	u64 size;
	struct cma_ctx *ctx = &cma_mem->cma_client;

	if (copy_from_user(&size, argp, sizeof(size))) {
		pr_err("Copy size arg %p from user failed", argp);
		return -EFAULT;
	}
	ptr = kmalloc(sizeof(struct cma_admin), GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	size  = PAGE_ALIGN(size);

	/* allocate space from CMA */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	ptr->kvaddr = dma_alloc_coherent(cma_mem->dev, size, &paddr,
			GFP_KERNEL | GFP_DMA);
#else
	ptr->kvaddr = dma_zalloc_coherent(cma_mem->dev, size, &paddr,
			GFP_KERNEL | GFP_DMA);
#endif
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
		dev_name(cma_mem->dev), (size_t)size, ptr->kvaddr, paddr);

	param = (u64)paddr;

	if (copy_to_user(argp, &param, sizeof(param))) {
		dma_free_coherent(cma_mem->dev, ptr->size,
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
static int _cma_free(struct musdk_cma *cma_mem,
		struct cma_admin *ptr)
{
	struct cma_ctx *ctx = &cma_mem->cma_client;

	if (cma_check_buf(ptr))
		return -EFAULT;

	/* To avoid double free on the same buffer */
	ptr->magic = ~CMA_MAGIC_NUMBER;

	pr_debug("CMA buffer released: size = %zd Bytes, kvaddr = %p, paddr = 0x%llx\n",
		(size_t)ptr->size, (void *)ptr->kvaddr, ptr->paddr);

	atomic_inc(&ctx->buf_free);
	list_del(&ptr->list);

	dma_free_coherent(cma_mem->dev, ptr->size, (void *)ptr->kvaddr,
			ptr->paddr);

	kfree(ptr);

	return 0;
}

/*
 * cma_free() - cma release handler
 *
 */
static int cma_free(struct musdk_cma *cma_mem, void *argp)
{
	u64 paddr;
	struct cma_admin *ptr = NULL;

	if (copy_from_user(&paddr, argp, sizeof(paddr)))
		return -EFAULT;

	ptr = cma_find_admin_by_paddr(cma_mem, (phys_addr_t)paddr);

	return _cma_free(cma_mem, ptr);
}

static int musdk_cma_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct musdk_cma *cma_mem = container_of(misc, struct musdk_cma, misc);

	atomic_inc(&cma_mem->refcount);

	pr_debug("CMA: reference counter: %d\n", atomic_read(&cma_mem->refcount));

	return 0;
}

static int musdk_cma_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct miscdevice *misc = filp->private_data;
	struct musdk_cma *cma_mem = container_of(misc, struct musdk_cma, misc);

	int err = 0;
	phys_addr_t paddr = vma->vm_pgoff << PAGE_SHIFT;
	struct cma_admin *ptr;

	if (vma->vm_pgoff == 0)
		return -EFAULT;

	/* Find cma_admin handle */
	ptr = cma_find_admin_by_paddr(cma_mem, paddr);

	if (cma_check_buf(ptr))
		return -EFAULT;

	vma->vm_pgoff = 0;

	err = dma_mmap_coherent(cma_mem->dev, vma,
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


static long musdk_cma_ioctl(struct file *filp,
			  unsigned int cmd, unsigned long args)
{
	void __user *argp = (void __user *)args;
	struct miscdevice *misc = filp->private_data;
	struct musdk_cma *cma_mem =  container_of(misc, struct musdk_cma, misc);
	int err = 0;

	if (_IOC_TYPE(cmd) != MUSDK_IOC_TYPE_BASE) {
		pr_err("ioctl(): bad command type 0x%x (should be 0x%x)\n",
		   _IOC_TYPE(cmd), MUSDK_IOC_TYPE_BASE);
		return -EIO;
	}

	switch (cmd) {

	case MUSDK_IOC_CMA_ALLOC:
		err = cma_calloc(cma_mem, argp);
		break;

	case MUSDK_IOC_CMA_FREE:
		err = cma_free(cma_mem, argp);
		break;
	default:
		pr_err("Unknown ioctl {0x%x} received.\n", cmd);
		return -EINVAL;
	}

	return err;
}


static int musdk_cma_release(struct inode *inode, struct file *filp)
{
	struct miscdevice *misc = filp->private_data;
	struct musdk_cma *cma_mem = container_of(misc, struct musdk_cma, misc);
	struct list_head *node, *q;
	int garbage = 0;
	struct cma_admin *adm;

	pr_debug("CMA: reference counter: %d\n", atomic_read(&cma_mem->refcount));
	if (!atomic_dec_and_test(&cma_mem->refcount))
		return 0;


	list_for_each_safe(node, q, &cma_mem->cma_client.list) {
		adm = list_entry(node, struct cma_admin, list);
		if (_cma_free(cma_mem, adm)) {
			pr_err("Buffer corrupt detected %p", adm);
			break;
		}
		garbage++;
	}

	int free = atomic_read(&cma_mem->cma_client.buf_free);
	int alloc = atomic_read(&cma_mem->cma_client.buf_alloc);

	pr_debug("CMA: total alloc %d, total free: %d (garbage %d)",
			alloc, free, garbage);

	if (free != alloc)
		pr_err("There are %d  alien buffers", alloc - free);

	return 0;
}

/*
 * musdk_cma_probe() - musdk_cma_driver platform driver probe routine
 * - register misc device for CMA memory pools
 */
static int musdk_cma_probe(struct platform_device *pdev)
{
#define MUSDK_DEF_CMA_MEM	(~0)
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct musdk_cma *cma_mem;
	struct miscdevice *misc;
	u32 mem_region_index = MUSDK_DEF_CMA_MEM;
	int err = 0;

	/* number of probes are according to dts.
	 * First pdev is for "musdk-cma". Following pdevs are for regions, "musdk-cma-reg%u".
	*/
	if (!np) {
		dev_err(dev, "Non DT case is not supported\n");
		return -EINVAL;
	}

	of_property_read_u32(np, "cma-mem-region", &mem_region_index);

	of_reserved_mem_device_init(&pdev->dev);

	cma_mem = devm_kzalloc(dev, sizeof(struct musdk_cma), GFP_KERNEL);
	if (!cma_mem) {
		err = -ENOMEM;
		goto fail;
	}

	cma_mem->dev = dev;
	atomic_set(&cma_mem->refcount, 0);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 20, 0)
	if (!dev_is_dma_coherent(dev))
#else
	if (!dev->archdata.dma_coherent)
#endif
		dev_warn(dev, "Not dma_coherent\n");

	dev->dma_mask = kmalloc(sizeof(*dev->dma_mask), GFP_KERNEL);
	err = dma_set_mask(dev, DMA_BIT_MASK(40));
	if (err == 0)
		dma_set_coherent_mask(dev, DMA_BIT_MASK(40));
	if (err) {
		dev_err(dev, "%s: cannot set dma_mask\n", of_node_full_name(np));
		goto fail;
	}

	misc		= &cma_mem->misc;
	misc->minor	= MISC_DYNAMIC_MINOR;
	if (mem_region_index != MUSDK_DEF_CMA_MEM)
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
	struct cma_ctx *ctx = (struct cma_ctx *) &cma_mem->cma_client;

	atomic_set(&ctx->buf_alloc, 0);
	atomic_set(&ctx->buf_free, 0);

	INIT_LIST_HEAD(&ctx->list);
	pr_info("Registered cma device: %s\n", misc->name);

	platform_set_drvdata(pdev, cma_mem);
	return 0;

fail:
	return err;
}

/*
 * musdk_cma_remove() - musdk_uio_drv platform driver release routine
 * - unregister uio devices
 * - unregister misc device
 *
 */
static int musdk_cma_remove(struct platform_device *pdev)
{
	struct musdk_cma *cma_mem = platform_get_drvdata(pdev);

	if (!cma_mem)
		return -EINVAL;

	misc_deregister(&cma_mem->misc);
	pr_debug("Detached misc: {%s} device\n", cma_mem->misc.name);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct file_operations musdk_misc_fops = {
	.owner		= THIS_MODULE,
	.open		= musdk_cma_open,
	.mmap		= musdk_cma_mmap,
	.release	= musdk_cma_release,
	.unlocked_ioctl	= musdk_cma_ioctl,
	.compat_ioctl	= musdk_cma_ioctl,
};

static const struct of_device_id musdk_of_match[] = {
	{ .compatible	= "marvell,musdk-cma", },
	{ .compatible	= "marvell,musdk-uio", },
	{ }
};

static struct platform_driver musdk_cma_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= DRIVER_NAME,
		.of_match_table	= musdk_of_match,
	},
	.probe	= musdk_cma_probe,
	.remove	= musdk_cma_remove,
};

/* *INDENT-ON* */

module_platform_driver(musdk_cma_driver);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:" DRIVER_NAME);
