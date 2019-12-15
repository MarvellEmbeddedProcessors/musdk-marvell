/*
** Copyright (C) 1999 - 2015 Intel Corporation.
** Copyright (C) 2015-2016 Marvell International Ltd.
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
#include <linux/pci.h>
#include "giu_nic.h"
#include "giu_nic_hw.h"

struct agnic_pci_data {
	struct pci_dev *pdev;
};


/* MSI-X is currently not supported in PCI mode */
#undef AGNIC_PCI_MSIX_SUPPORTED

/* TODO: Adapt according to future assignment for AGNIC devices. */
#define PCI_DEVICE_ID_MARVELL_88F8040_NIC	0x7080
#define PCI_MAX_NUM_VF				7

/*
 * agnic_pci_irq_enable - Enable interrupts reception for all q-vectors.
 */
static void agnic_pci_irq_enable(struct agnic_adapter *adapter)
{
	struct msix_entry *entry;
	int vector;

	if (!adapter->msix_entries)
		return;

	for (vector = 0; vector < adapter->num_vectors; vector++) {
		entry = &adapter->msix_entries[vector];
#ifdef AGNIC_PCI_MSIX_SUPPORTED
		enable_irq(entry->vector);
#endif
	}
}

/*
** agnic_pci_irq_disable - Mask off interrupt generation on the NIC
*/
static void agnic_pci_irq_disable(struct agnic_adapter *adapter)
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

static int agnic_pci_disable_msix(struct agnic_adapter *adapter)
{
	return 0;
}

static int agnic_pci_enable_msix(struct agnic_adapter *adapter)
{
	return 0;
}

static int agnic_pci_enable_msi(struct agnic_adapter *adapter)
{
	return 0;
}

static int agnic_pci_disable_msi(struct agnic_adapter *adapter)
{
	return 0;
}


/*
** agnic_pci_acquire_msix_vectors - acquire MSI-X vectors
**
** Attempts to acquire a suitable range of MSI-X vector interrupts. Will
** return a negative error code if unable to acquire MSI-X vectors for any
** reason.
*/
static int agnic_pci_acquire_msix_vectors(struct agnic_adapter *adapter)
{
	struct agnic_pci_data *pci_data = adapter->bus_data;
	int i, q_vectors, vec_got, vec_req;

	/* TODO: the following code should be removed in future once we'll
	 * support more than single msix.
	if ((adapter->msix_flags & AGNIC_FLAG_MSIX_ENABLED) == AGNIC_FLAG_MSIX_ENABLED)
		adapter->msix_flags &= ~AGNIC_FLAG_TX_MSIX_ENABLED;
	*/

	/* We start by asking for one vector per queue pair */
	vec_req = max(adapter->num_rx_queues, adapter->num_tx_queues);

	/* Limit the number of MSIx interrupts to the number of CPUs. */
	vec_req = min_t(int, vec_req, num_online_cpus());
	q_vectors = vec_req;

	/* If both Rx and Tx interrutps are enabled, allocate double amount of interrupts */
	if ((adapter->msix_flags & AGNIC_FLAG_MSIX_ENABLED) == AGNIC_FLAG_MSIX_ENABLED)
		vec_req *= 2;

	/* Some vectors are necessary for non-queue interrupts */
	vec_req += NON_Q_VECTORS;

	/* TODO: the following code should be removed in future once we'll
	 * support more than single msix.
	if (vec_req != 1) {
		for (i = 0; i < vec_req; i++)
			adapter->msix_entries[i].entry = AGNIC_MSIX_ID_INVALID;
		return -1;
	}
	*/

	adapter->msix_entries = kcalloc(vec_req, sizeof(struct msix_entry),
			GFP_KERNEL);
	if (!adapter->msix_entries)
		return -ENOMEM;

#ifndef AGNIC_PCI_MSIX_SUPPORTED
	/* set all msix entries to NOT_SUPPORTED and exit with error
	 * as it will tell the caller layer that MSI-X were not allocated
	 */
	for (i = 0; i < vec_req; i++)
		adapter->msix_entries[i].entry = AGNIC_MSIX_ID_INVALID;

	return -1;
#endif

	for (i = 0; i < vec_req; i++)
		adapter->msix_entries[i].entry = i;

	vec_got = pci_enable_msix_exact(pci_data->pdev,
					adapter->msix_entries,
					vec_req);

	/* TODO: temporary, we don't allow to get less vectors then what we request.
	*/
	if ((vec_got < 0) || (vec_got != vec_req)) {
		/* A negative count of allocated vectors indicates an error in
		 * acquiring within the specified range of MSI-X vectors
		 */
		agnic_dev_warn("Failed to allocate MSI-X interrupts (req=%d, ret=%d).\n",
				vec_req, vec_got);

		adapter->flags &= ~AGNIC_FLAG_MSIX_ENABLED;
		kfree(adapter->msix_entries);
		adapter->msix_entries = NULL;

		return vec_got;
	}

	/* need to update the msix-id with the ids allowed in agnic */
	for (i = 0; i < vec_req; i++)
		adapter->msix_entries[i].entry = AGNIC_SET_MSI_IDX(adapter->msix_entries[i].entry);

	/* MSIx vectors allocation succeeded (at least partially) */
	adapter->flags |= adapter->msix_flags;
	adapter->num_vectors = vec_got;
	adapter->num_q_vectors = q_vectors;

	return 0;
}

static void agnic_xmit_notify(struct agnic_adapter *adapter, int tc)
{
}

static int agnic_pci_probe(struct pci_dev *pci_dev, const struct pci_device_id *ent)
{
	struct device *dev = &pci_dev->dev;
	struct agnic_adapter *adapter;
	struct agnic_config_mem	*nic_cfg = NULL;
	struct net_device *netdev = NULL;
	struct agnic_pci_data *pci_data = NULL;
	int bars_mask = 0;
	int ret;
	int err;
	int bar;
	unsigned int offset, length;

	agnic_probe_debug("probing the device.\n");

	pci_data = kzalloc(sizeof(*pci_data), GFP_KERNEL);
	if (!pci_data) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate pci_dev info.\n");
		goto err_alloc;
	}
	pci_data->pdev = pci_dev;

	/* enable the device */
	ret = pci_enable_device(pci_dev);
	if (ret) {
		dev_err(dev, "Failed to enable PCI device.\n");
		goto err_pci_reg;
	}

	/* Request all memory BARs */
	bars_mask = pci_select_bars(pci_dev, IORESOURCE_MEM);
	dev_info(&pci_dev->dev, "PCI memory bars are %#x\n", bars_mask);

	ret = pci_request_selected_regions(pci_dev, bars_mask, "agnic-cfg");
	if (ret) {
		dev_err(dev, "Failed to request agnic memory BARs\n");
		goto err_pci_reg;
	}

	/* Map the agnic configuration space */
	agnic_probe_debug("Mapping BAR-%d.\n", AGNIC_CONFIG_BAR_ID);
	bar = AGNIC_CONFIG_BAR_ID;
	offset = 0;
	length = AGNIC_CONFIG_BAR_SIZE;
	nic_cfg = pci_iomap_range(pci_dev, bar, offset, length);
	if (!nic_cfg) {
		dev_err(dev, "Unable to map NIC config struct %#x@%#x on bar %i\n",
				length, offset, bar);
		ret = -EFAULT;
		goto err_pci_reg;
	}

	/* Limit DMA buffers to lower 32GB of memory.
	** TODO: dma mask limitation should be removed once the NIC
	** physical address mapping is fixed to include the complete host's
	** DRAM.
	*/
	pci_set_consistent_dma_mask(pci_dev, DMA_BIT_MASK(35));

	/* DMA coherency configurations */
	pci_dev->dev.dma_mask = kmalloc(sizeof(*pci_dev->dev.dma_mask), GFP_KERNEL);
	if (!pci_dev->dev.dma_mask) {
		ret = -ENOMEM;
		dev_err(dev, "Failed to allocate dma_mask.\n");
		goto err_dma_alloc;
	}
	err = dma_set_mask(&pci_dev->dev, DMA_BIT_MASK(35));
	if (err) {
		ret = err;
		dev_err(&pci_dev->dev, "agnic: cannot set dma_mask\n");
		goto err_pci_reg;
	}
	dma_set_coherent_mask(&pci_dev->dev, DMA_BIT_MASK(32));

	/* Enable DMA access over PCIe (issued by the NIC). */
	pci_set_master(pci_dev);

	/* Save PCI config space for PM purposes */
	err = pci_save_state(pci_dev);
	if (err)
		goto err_pci_reg;

	/* Enable SRIOV */
	err = pci_enable_sriov(pci_dev, PCI_MAX_NUM_VF);
	if (err)
		goto err_pci_reg;

	/* Allocate net_device from agnic_net */
	netdev = agnic_net_alloc_netdev(&pci_dev->dev);
	if (netdev == NULL) {
		ret = -EFAULT;
		goto err_pci_reg;
	}

	/* Prepare & register net device. */
	adapter = netdev_priv(netdev);

	/* Set physical address of PCI Bar. */
	adapter->nic_cfg_phys_base = pci_resource_start(pci_dev, bar);
	adapter->nic_cfg_on_local_mem = false;

	adapter->bars_mask = bars_mask;

	/* Set bus callback functions. */
	adapter->bus_acquire_msix_vectors = agnic_pci_acquire_msix_vectors;
	adapter->bus_disable_msix = agnic_pci_disable_msix;
	adapter->bus_enable_msix = agnic_pci_enable_msix;
	adapter->bus_enable_msi = agnic_pci_enable_msi;
	adapter->bus_disable_msi = agnic_pci_disable_msi;
	adapter->bus_irq_enable = agnic_pci_irq_enable;
	adapter->bus_irq_disable = agnic_pci_irq_disable;

	/* Set Data-path callback functions */
	adapter->xmit_notify = agnic_xmit_notify;

	/* Call the agnic netdev driver code, for networking specific
	 * initializations.
	 */
	ret = agnic_net_probe(&pci_dev->dev, nic_cfg, pci_data);
	if (ret) {
		dev_err(dev, "agnic_net_probe failed (%d).\n", ret);
		goto err_pci_reg;
	}

	return 0;

err_pci_reg:
	if (netdev)
		agnic_net_free_netdev(netdev);

	kfree(pci_dev->dev.dma_mask);
err_dma_alloc:
	if (nic_cfg)
		iounmap((void *)nic_cfg);

	if (bars_mask)
		pci_release_selected_regions(pci_dev, bars_mask);
	pci_disable_device(pci_dev);

	kfree(pci_data);
err_alloc:
	return ret;
}

/**
 * agnic_pci_remove - Device Removal Routine
 * @pdev: PCI device information struct
 *
 * agnic_pci_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.  The could be caused by a
 * Hot-Plug event, or because the driver is going to be removed from
 * memory.
 **/
static void agnic_pci_remove(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct agnic_adapter *adapter = netdev_priv(netdev);

	agnic_net_remove(&pdev->dev);

	iounmap((void *)adapter->nic_cfg_base);

	pci_release_selected_regions(pdev, adapter->bars_mask);

	pci_disable_device(pdev);
}


static const struct pci_device_id agnic_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_MARVELL,
			PCI_DEVICE_ID_MARVELL_88F8040_NIC) },
	{ 0 }
};

MODULE_DEVICE_TABLE(pci, agnic_id_table);
static struct pci_driver agnic_pci_driver = {
	.name		= "agnic_pci",
	.id_table	= agnic_id_table,
	.probe		= agnic_pci_probe,
	.remove		= agnic_pci_remove,
};
module_pci_driver(agnic_pci_driver);

MODULE_DESCRIPTION("Armada PCIe GIU-NIC Driver");
MODULE_AUTHOR("Shadi Ammouri <shadi@marvell.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1");

