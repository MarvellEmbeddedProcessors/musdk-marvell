/*
** Copyright (C) 1999 - 2015 Intel Corporation.
** Copyright (c) 2015 Marvell.
**
** This code was derived from Intel's ixgbe Linux driver.
**
** This program is free software: you can redistribute it and/or
** modify it under the terms of the GNU General Public License as
** published by the Free Software Foundation, either version 2 of the
** License, or any later version.
**
** This program is distributed in the hope that it will be useful, but
** WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
** General Public License for more details.
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/msi.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/io.h>
#include "giu_nic.h"
#include "giu_nic_hw.h"
#include "giu_nic_uio.h"
#include "giu_nic_sysfs.h"
#ifdef CONFIG_MV_NSS_ENABLE
#include "nss_dpapi.h"
#endif

#define CFG_MEM_VALID			(1 << 0)
#define CFG_MEM_BAR_MIN_ALIGN_BITS	(12)
#define CFG_MEM_BAR_MIN_ALIGN		(1 << CFG_MEM_BAR_MIN_ALIGN_BITS)
#define CFG_MEM_64B_MAGIC_MASK		(0xFFFFLL << 48)
#define CFG_MEM_64B_MAGIC_VAL		(0xCAFELL << 48)
/* in 64bits reg, we allow address of 44bits with 12bits alignment */
#define CFG_MEM_64B_ADDR_MASK		\
	(0x00000FFFFFFFFFFFLL & ~(CFG_MEM_BAR_MIN_ALIGN - 1))
/* in 32bits reg, we allow address of 36bits with 12bits alignment */
#define CFG_MEM_32B_ADDR_MASK		\
	(0x0000000FFFFFFFFFLL & ~(CFG_MEM_BAR_MIN_ALIGN - 1))
#define CFG_MEM_32B_ADDR_SHIFT		(4)

struct agnic_plat_data {
	struct platform_device *pdev;
	/* Virtual address of the registers that holds the physical base address
	 * of the GIU shared memory.
	 */
	void *sh_mem_base_reg;

	/* MSI-X table virtual address */
	u64 msix_tbl_base;
};


/*
 * agnic_plat_irq_enable - Enable interrupts reception for all q-vectors.
 *
 * Note: management interrupt are not enabled here.
 */
static void agnic_plat_irq_enable(struct agnic_adapter *adapter)
{
	struct msix_entry *entry;
	int vector;

	if (!adapter->msix_entries)
		return;

	for (vector = 0; vector < adapter->num_vectors; vector++) {
		entry = &adapter->msix_entries[vector];
		enable_irq(entry->vector);
	}
}

/*
 * agnic_plat_irq_disable - Mask off interrupt generation on the NIC
 *
 * Note: management interrupt are not disabled here.
 */
static void agnic_plat_irq_disable(struct agnic_adapter *adapter)
{
	struct msix_entry *entry;
	int vector;

	if (!adapter->msix_entries)
		return;

	for (vector = 0; vector < adapter->num_vectors; vector++) {
		entry = &adapter->msix_entries[vector];
		disable_irq(entry->vector);
	}
}

static int agnic_plat_enable_msi(struct agnic_adapter *adapter)
{
	return 0;
}

static int agnic_plat_disable_msi(struct agnic_adapter *adapter)
{
	return 0;
}

static int agnic_plat_enable_msix(struct agnic_adapter *adapter)
{
	/* N/A: not relevant for platform mode */
	return 0;
}

static int agnic_plat_disable_msix(struct agnic_adapter *adapter)
{
	/* N/A: not relevant for platform mode */
	return 0;
}

/*
 * agnic_set_msi_msg - Set agnic Interrupt-message (IMSG) parameters.
 *
 * This callback is provided to platform_msi_domain_alloc_irqs API.
 * It is called every time request_irq is called.
 * Each time the msi index is incremented and it's used to determine
 * which entry in MSI-X table will be used.
 */
static void agnic_set_msi_msg(struct msi_desc *desc, struct msi_msg *msg)
{
	void __iomem *entry_base;
	struct device *dev = msi_desc_to_dev(desc);
	struct net_device *netdev;
	struct agnic_adapter *adapter;
	struct agnic_plat_data *plat_data;
	u32 entry_idx = AGNIC_SET_MSI_IDX(desc->platform.msi_index);

	/* Retrieve the platform data from the device/netdev/adapter */
	netdev = dev_get_drvdata(dev);
	adapter = netdev_priv(netdev);
	plat_data = adapter->bus_data;

	/* Print the MSI message only during irq request (as this callback is also
	 *  called during irq release process)
	 */
	if (msg->address_lo != 0) {
		dev_info(dev, "Register msi-x msg id %d at entry %d\n",
				desc->platform.msi_index, entry_idx);
		pr_debug("MSI-X id %d msg info: 0x%llx = 0x%x\n",
				desc->platform.msi_index, msg->address_lo |
				((u64)msg->address_hi) << 32, msg->data);
	}

	entry_base = ((void *)plat_data->msix_tbl_base) + (entry_idx * PCI_MSIX_ENTRY_SIZE);

	/* Write the MSI message to the MSI-X table entry */
	writel(msg->address_lo, entry_base + PCI_MSIX_ENTRY_LOWER_ADDR);
	writel(msg->address_hi, entry_base + PCI_MSIX_ENTRY_UPPER_ADDR);
	writel(msg->data, entry_base + PCI_MSIX_ENTRY_DATA);
}

/*
 * agnic_plat_release_msix - Release allocated MSIs.
 *
 */
static void agnic_plat_release_msix(void *data)
{
	struct device *dev = data;

	platform_msi_domain_free_irqs(dev);
}

/*
 * agnic_pci_acquire_msix_vectors - acquire MSI-X vectors
 *
 * Attempts to acquire a suitable range of MSI-X vector interrupts. Will
 * return a negative error code if unable to acquire MSI-X vectors for any
 * reason.
 */
static int agnic_plat_acquire_msix_vectors(struct agnic_adapter *adapter)
{
	struct msi_desc *desc;
	int q_vectors, vectors_max;
	int num_available_cpus = cpumask_weight(&adapter->available_cpus);
	int i = 0, ret;

	/* Leave 1 CPU for the application (unless we have a single CPU) */
	if (num_available_cpus > 1)
		num_available_cpus--;

	/* We start by asking for one vector per queue pair */
	vectors_max = max(adapter->num_rx_queues,
			  adapter->num_tx_queues);

	/* Limit the number of MSIx interrupts to the number of CPUs. */
	vectors_max = min_t(int, vectors_max, num_available_cpus);
	q_vectors = vectors_max;

	/* If both Rx and Tx interrutps are enabled, allocate double amount of interrupts */
	if ((adapter->msix_flags & AGNIC_FLAG_MSIX_ENABLED) == AGNIC_FLAG_MSIX_ENABLED)
		vectors_max *= 2;

	/* Some vectors are necessary for non-queue interrupts */
	vectors_max += NON_Q_VECTORS;

	adapter->msix_entries = kcalloc(vectors_max, sizeof(struct msix_entry),
			GFP_KERNEL);
	if (!adapter->msix_entries)
		return -ENOMEM;

	/* Allocate Platform MSI vectors.
	 * We pass a callback (agnic_set_msi_msg) that will  be called each time
	 * request_irq is called (during open flow). msix_entries structure is
	 * used to save the MSI-X table entry index and IRQ number.
	 */
	ret = platform_msi_domain_alloc_irqs(adapter->dev, vectors_max, agnic_set_msi_msg);
	if (ret < 0) {
		/* A negative count of allocated vectors indicates an error in
		 * acquiring within the specified range of MSI-X vectors
		 */
		agnic_dev_warn("Failed to allocate MSI-X interrupts (ret = %d).\n",
				ret);

		adapter->flags &= ~AGNIC_FLAG_MSIX_ENABLED;
		kfree(adapter->msix_entries);
		adapter->msix_entries = NULL;

		return ret;
	}

	/* Update msix_entries structure with MSI-X table entry index and IRQ number */
	for_each_msi_entry(desc, adapter->dev) {
		adapter->msix_entries[i].entry = AGNIC_SET_MSI_IDX(desc->platform.msi_index);
		adapter->msix_entries[i].vector = desc->irq;
		pr_debug("msi index %d (entry %d) irq id is %d\n", desc->platform.msi_index,
						adapter->msix_entries[i].entry, desc->irq);

		i++;
	}

	/* MSIx vectors allocation succeeded (at least partially) */
	adapter->flags |= adapter->msix_flags;
	adapter->num_vectors = vectors_max;
	adapter->num_q_vectors = q_vectors;

	/* Add callback to free MSIs on teardown */
	devm_add_action(adapter->dev, agnic_plat_release_msix, adapter->dev);

	return 0;
}

static void agnic_xmit_notify(struct agnic_adapter *adapter, int tc)
{
	agnic_uio_notify(adapter, tc);
}

static int agnic_plat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct agnic_adapter *adapter;
	struct agnic_config_mem	*nic_cfg = NULL;
	struct net_device *netdev;
	struct agnic_plat_data *plat_data = NULL;
	phys_addr_t config_mem_phys_base;
	int timeout = 3000;
	struct resource *res;
	int ret;

	agnic_probe_debug("probing the device.\n");

	plat_data = devm_kzalloc(&pdev->dev, sizeof(*plat_data), GFP_KERNEL);
	if (!plat_data) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate plat_dev info.\n");
		goto err_alloc;
	}
	plat_data->pdev = pdev;

	/* Map shared memory base address register from DT. */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev_info(dev, "Mapping device resource %pR.\n", res);
	plat_data->sh_mem_base_reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(plat_data->sh_mem_base_reg)) {
		dev_err(dev, "Failed in mapping agnic-plat shared memory base register.\n");
		ret = PTR_ERR(plat_data->sh_mem_base_reg);
		goto err_map;
	}

	/* Check that the base address includes the required magic value and
	 * valid bit, in order to make sure that the mgmt firmware has written
	 * the value, and that we are not using some "garbage" base address.
	 */
	do {
		/* Get physical address of shared memory. */
		config_mem_phys_base = readl(plat_data->sh_mem_base_reg) |
			((u64)readl(plat_data->sh_mem_base_reg + 4) << 32);

		if ((config_mem_phys_base & CFG_MEM_VALID) &&
			((config_mem_phys_base & CFG_MEM_64B_MAGIC_MASK) == CFG_MEM_64B_MAGIC_VAL))
			break;

		usleep_range(1000, 2000);
		timeout--;
	} while (timeout);

	if (timeout)
		/* we can use 64bits (since we got the majic word) */
		/* Clear the valid and magic bits out of the base address. */
		config_mem_phys_base &= CFG_MEM_64B_ADDR_MASK;
	else {
		/* we can't use 64bits; let's try 32bits */
		if (!(config_mem_phys_base && (config_mem_phys_base & CFG_MEM_VALID))) {
			dev_err(dev, "Valid bit and/or Magic value not part of the config-mem\n"
				"\tbase addr register (%p @ %p)!\n",
				(void *)config_mem_phys_base, plat_data->sh_mem_base_reg);
			ret = -EFAULT;
			goto err_map;
		}
		/* we can use 32bits (since we got the valid) */
		config_mem_phys_base <<= CFG_MEM_32B_ADDR_SHIFT;
		/* Clear the valid and magic bits out of the base address. */
		config_mem_phys_base &= CFG_MEM_32B_ADDR_MASK;
	}

	/* DMA coherency configurations */
	pdev->dev.dma_mask = kmalloc(sizeof(*pdev->dev.dma_mask), GFP_KERNEL);
	if (!pdev->dev.dma_mask) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate dma_mask.\n");
		goto err_dma_alloc;
	}
	ret = dma_set_mask(&pdev->dev, DMA_BIT_MASK(40));
	if (ret) {
		dev_err(&pdev->dev, "agnic: cannot set dma_mask\n");
		goto err_netdev_alloc;
	}
	dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));

	dev_info(dev, "agnic configuration memory @ 0x%p.\n", (void *)config_mem_phys_base);
	/* Map the agnic configuration memory space. */
	agnic_probe_debug("Mapping Shared memory.\n");
	nic_cfg = phys_to_virt(config_mem_phys_base);
	if (!nic_cfg) {
		dev_err(dev, "Unable to map NIC config struct %p\n",
			(void *)config_mem_phys_base);
		ret = -EFAULT;
		goto err_netdev_alloc;
	}

	/* Reset the register (in BAR0) which holds the shared memory base address
	 * (and valid bit and magic word) to allow re-loading of the module.
	 * Otherwise, this data will stay valid and next time the
	 * module is loaded, it will have it ready even though
	 * the device didn't configure it.
	 */
	writel(0, plat_data->sh_mem_base_reg);
	writel(0, plat_data->sh_mem_base_reg + 4);

	/* Allocate net_device from agnic_net */
	netdev = agnic_net_alloc_netdev(dev);
	if (netdev == NULL) {
		ret = -EFAULT;
		goto err_netdev_alloc;
	}

	/* Prepare & register net device. */
	adapter = netdev_priv(netdev);

	/* Set physical address of config memory. */
	adapter->nic_cfg_phys_base = config_mem_phys_base;
	adapter->nic_cfg_on_local_mem = true;

	/* Set bus callback functions. */
	adapter->bus_acquire_msix_vectors = agnic_plat_acquire_msix_vectors;
	adapter->bus_disable_msix = agnic_plat_disable_msix;
	adapter->bus_enable_msix = agnic_plat_enable_msix;
	adapter->bus_enable_msi = agnic_plat_enable_msi;
	adapter->bus_disable_msi = agnic_plat_disable_msi;
	adapter->bus_irq_enable = agnic_plat_irq_enable;
	adapter->bus_irq_disable = agnic_plat_irq_disable;

	/* Set Data-path callback functions */
	adapter->xmit_notify = agnic_xmit_notify;

	/* Call the agnic netdev driver code, for networking specific
	 * initializations.
	 */
	ret = agnic_net_probe(dev, nic_cfg, plat_data);
	if (ret) {
		dev_err(dev, "agnic_net_probe failed (%d).\n", ret);
		goto err_net_probe;
	}

	/* Set MSI-X table base address
	 * (only after probe the MSI-X table offset is set).
	 */
	plat_data->msix_tbl_base = (u64)adapter->nic_cfg_base + (u64)nic_cfg->msi_x_tbl_offset;

	/* Register uio devices */
	ret = agnic_uio_probe(dev);
	if (ret) {
		dev_err(dev, "agnic_uio_probe failed (%d).\n", ret);
		goto err_net_probe;
	}

#ifdef CONFIG_MV_NSS_ENABLE
	/* TODO: move this call to agnic_net_probe once both plat and pci
	 * modes are supported.
	 */
	ret = agnic_sysfs_init();
	if (ret) {
		dev_err(dev, "agnic_sysfs_init failed (%d).\n", ret);
		goto err_sysfs_probe;
	}

	/* nss_dpapi init routine as part of module init */
	ret = mv_dpapi_init(netdev);
	if (ret) {
		dev_err(dev, "mv_dpapi_init failed (%d).\n", ret);
		goto err_dpapi_init;
	}
#endif

	return 0;
#ifdef CONFIG_MV_NSS_ENABLE
err_dpapi_init:
	agnic_sysfs_exit();
err_sysfs_probe:
	agnic_uio_remove(dev);
#endif
err_net_probe:
	if (netdev)
		agnic_net_free_netdev(netdev);
err_netdev_alloc:
	kfree(pdev->dev.dma_mask);
err_dma_alloc:
	if (nic_cfg)
		iounmap((void *)nic_cfg);
err_map:
err_alloc:
	return ret;
}

/**
 * agnic_plat_remove - Device Removal Routine
 * @pdev: Platform device information struct
 *
 **/
static int agnic_plat_remove(struct platform_device *pdev)
{
	struct net_device *netdev = platform_get_drvdata(pdev);
	struct agnic_adapter *adapter = netdev_priv(netdev);
	struct agnic_plat_data *plat_data = adapter->bus_data;
	struct device *dev = &pdev->dev;

#ifdef CONFIG_MV_NSS_ENABLE
	/* nss_dpapi exit routine as part or module exit */
	mv_dpapi_exit();
#endif
	/* unregister uio devices */
	agnic_uio_remove(dev);

	/* TODO: move this call to agnic_net_probe once both plat and pci
	 * modes are supported.
	 */
	agnic_sysfs_exit();

	agnic_net_remove(dev);

	adapter->flags &= ~AGNIC_FLAG_MSIX_ENABLED;
	kfree(adapter->msix_entries);
	adapter->msix_entries = NULL;

	kfree(pdev->dev.dma_mask);

	iounmap((void *)adapter->nic_cfg_base);

	devm_iounmap(dev, plat_data->sh_mem_base_reg);

	devm_kfree(dev, plat_data);

	return 0;
}


static const struct of_device_id agnic_plat_dt_ids[] = {
	{ .compatible = "marvell,armada-giu-nic", },
	{},
};

MODULE_DEVICE_TABLE(of, agnic_plat_dt_ids);

static struct platform_driver agnic_plat_driver = {
	.probe          = agnic_plat_probe,
	.remove         = agnic_plat_remove,
	.driver         = {
		.owner  = THIS_MODULE,
		.name   = "agnic_plat",
		.of_match_table = of_match_ptr(agnic_plat_dt_ids),
	},
};

module_platform_driver(agnic_plat_driver);

MODULE_DESCRIPTION("Armada Platform GIU-NIC Driver");
MODULE_AUTHOR("Shadi Ammouri <shadi@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1");

