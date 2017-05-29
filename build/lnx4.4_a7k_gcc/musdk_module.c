/******************************************************************************
 *      Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of Marvell nor the names of its contributors may be
 *        used to endorse or promote products derived from this software
 *        without specific prior written permission.
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

#include <linux/platform_device.h>

#include "std_internal.h"
#include "drivers/mv_pp2.h"

#define DRIVER_NAME     "musdk"
#define DRIVER_VERSION  "0.1"
#define DRIVER_AUTHOR   "Marvell"
#define DRIVER_DESC     "Marvell User Space Development Kit"

#define MISC_DEV_NAME   "musdk"

#define MUSDK_MODULE_DMA_MEM_SIZE	(48 * 1024 * 1024)

static int musdk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int err;

	if (!dev->of_node) {
		dev_err(dev, "Device Tree does not contain a \"marvell,musdk-uio\" node.\n");
		return -EINVAL;
	}

	err = mv_sys_dma_mem_init(dev, MUSDK_MODULE_DMA_MEM_SIZE);
	if (err)
		return err;


	/* TODO: complete here …. */

	return 0;
}

static int musdk_remove(struct platform_device *pdev)
{
	mv_sys_dma_mem_destroy();

	return 0;
}

static const struct of_device_id musdk_of_match[] = {
	{ .compatible   = "marvell,musdk-uio", },
	{ }
};

static struct platform_driver musdk_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= DRIVER_NAME,
		.of_match_table = musdk_of_match,
	},
	.probe  = musdk_probe,
	.remove = musdk_remove,
};

module_platform_driver(musdk_driver);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_ALIAS("platform:" DRIVER_NAME);
