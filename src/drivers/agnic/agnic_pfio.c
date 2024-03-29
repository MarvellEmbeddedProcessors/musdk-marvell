/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "agnic_pfio.h"


#define AGNIC_COOKIE_DRIVER_WATERMARK	0xdeaddead


/*
 * agnic_init_q_indices - Initialize Q pointers page.
 */
static int agnic_init_q_indeces(struct agnic_pfio *pfio)
{
	int size;

	pfio->ring_indices_arr_len = AGNIC_MAX_QUEUES;
	size = pfio->ring_indices_arr_len * sizeof(u32);
	if ((pfio->nic_cfg->dev_use_size + size) > AGNIC_CONFIG_BAR_SIZE) {
		pr_err("not enough memory on BAR for rings indicex pointers!\n");
		return -ENOMEM;
	}
	pfio->ring_indices_arr = (void *)((u64)pfio->nic_cfg + pfio->nic_cfg->dev_use_size);
	/* in case the indices are allocated on the BAR, we would like to send only the offset
	 * from the bar base address
	 */
	pfio->ring_indices_arr_phys = pfio->nic_cfg->dev_use_size;
	/* Set all entries to "free" state. */
	memset(pfio->ring_indices_arr, 0xFF, size);

	return 0;
}

/*
** agnic_get_q_idx - allocate a free entry for queue control pointers.
*/
static int agnic_get_q_idx(struct agnic_pfio *pfio)
{
	int i;

	for (i = 0; i < pfio->ring_indices_arr_len; i++)
		if (pfio->ring_indices_arr[i] == 0xFFFFFFFF) {
			pfio->ring_indices_arr[i] = 0;
			return i;
		}

	/* All entries are occupied, not likely to happen. */
	pr_err("All Ring indeces are occupied (%d entries).\n",
		pfio->ring_indices_arr_len);

	return -ENODEV;
}

/*
** agnic_put_q_idx - Free a previous allocated entry for queue control pointers.
*/
static void agnic_put_q_idx(struct agnic_pfio *pfio, u32 index)
{
	pfio->ring_indices_arr[index] = 0xFFFFFFFF;
}

static void agnic_set_num_queues(struct agnic_pfio *pfio)
{
	/* Start with base case */
	pfio->num_rx_queues = pfio->num_in_tcs * pfio->num_qs_per_tc;
	pfio->num_tx_queues = pfio->num_out_tcs * pfio->num_qs_per_tc;
	pfio->num_rx_pools = pfio->num_rx_queues;
}

static int init_pfio_pci_mem(struct agnic_pfio *pfio, struct agnic_pfio_init_params *params)
{
	pfio->nic_cfg_phys_base = params->pci_bar_inf.pa;
	pfio->nic_cfg = params->pci_bar_inf.va;

	return 0;
}

static int init_pfio_plat_uio_mem(struct agnic_pfio *pfio)
{
	struct sys_iomem_params		 iomem_params;
	dma_addr_t			 addr;
	size_t				 dev_map_size;
	int				 err, bar_idx, timeout = 3000;

	iomem_params.devname = AGNIC_COMPATIBLE_STR;
	iomem_params.index = pfio->id;
	iomem_params.type = SYS_IOMEM_T_MMAP;

	err = sys_iomem_init(&iomem_params, &pfio->reg_iomem);
	if (err) {
		pr_err("failed to created IOMEM for AGNIC regs!\n");
		return err;
	}

	err = sys_iomem_map(pfio->reg_iomem, "0", &addr, &pfio->sh_mem_base_reg);
	if (err) {
		pr_err("failed to map AGNIC regs!\n");
		sys_iomem_deinit(pfio->reg_iomem);
		return err;
	}

	pr_debug("AGNIC %d regs address: pa 0x%"PRIx64", va %p\n", pfio->id, addr, pfio->sh_mem_base_reg);

	/* Check that the base address includes the required magic value and
	 * valid bit, in order to make sure that the mgmt firmware has written
	 * the value, and that we are not using some "garbage" base address.
	 */
	do {
		/* Get physical address of shared memory. */
		addr = readl(pfio->sh_mem_base_reg) |
			((u64)readl(pfio->sh_mem_base_reg + 4) << 32);

		if ((addr & CFG_MEM_VALID) &&
			((addr & CFG_MEM_64B_MAGIC_MASK) == CFG_MEM_64B_MAGIC_VAL))
			break;

		usleep(1000);
		timeout--;
	} while (timeout);
	pr_debug("got cfg reg: %"PRIx64"\n", addr);

	bar_idx = (addr >> CFG_MEM_BIDX_SHIFT) & CFG_MEM_BIDX_MASK;
	dev_map_size = (bar_idx + 1) * AGNIC_BAR_ALLOC_SIZE;
	if (timeout)
		/* we can use 64bits (since we got the majic word) */
		/* Clear the valid and magic bits out of the base address. */
		addr &= CFG_MEM_64B_ADDR_MASK;
	else {
		/* we can't use 64bits; let's try 32bits */
		if (!(addr && (addr & CFG_MEM_VALID))) {
			pr_err("Valid bit and/or Magic value not part of the config-mem\n"
				"\tbase addr register (0x%"PRIx64" @ %p)!\n",
				addr, pfio->sh_mem_base_reg);
			return -EFAULT;
		}
		/* we can use 32bits (since we got the valid) */
		addr <<= CFG_MEM_32B_ADDR_SHIFT;
		/* Clear the valid and magic bits out of the base address. */
		addr &= CFG_MEM_32B_ADDR_MASK;
	}

	if (!addr) {
		pr_err("Illegal BAR physical address (0x%"PRIx64")!\n", addr);
		return -EFAULT;
	}

	/* Set physical address of config memory. */
	pfio->nic_cfg_phys_base = addr;

	iomem_params.type = SYS_IOMEM_T_SHMEM;
	iomem_params.devname = MUSDK_CMA_DEV_STR;
	iomem_params.index = 0;
	iomem_params.size = dev_map_size;

	if (sys_iomem_init(&iomem_params, &pfio->bar_iomem)) {
		pr_err("failed to created IOMEM for AGNIC bar!\n");
		return -EINVAL;
	}

	/* addr will hold the device emulated-BARs table base-address */
	addr = pfio->nic_cfg_phys_base - bar_idx * AGNIC_BAR_ALLOC_SIZE;

	/* Map the iomem physical address */
	if (sys_iomem_map(pfio->bar_iomem, NULL, (phys_addr_t *)&addr,
			  (void **)&pfio->nic_cfg)) {
		pr_err("failed to map AGNIC bar!\n");
		/* Something went wrong with the shared-register;
		 * reset it to allow re-loading of the module.
		 */
		writel(0, pfio->sh_mem_base_reg + 4);
		sys_iomem_deinit(pfio->bar_iomem);
		return -EINVAL;
	}
	/* need t oadd now the BAR offset */
	pfio->nic_cfg = (void *)((u64)pfio->nic_cfg + bar_idx * AGNIC_BAR_ALLOC_SIZE);

	pr_debug("AGNIC %d BAR0 address: pa 0x%"PRIx64", va %p\n",
		pfio->id, pfio->nic_cfg_phys_base, pfio->nic_cfg);

	/* Reset the register (in BAR0) which holds the shared memory base address
	 * (and valid bit and magic word) to allow re-loading of the module.
	 * Otherwise, this data will stay valid and next time the
	 * module is loaded, it will have it ready even though
	 * the device didn't configure it.
	 */
	writel(0, pfio->sh_mem_base_reg);
	writel(0, pfio->sh_mem_base_reg + 4);

	return 0;
}

/*
 * Communicate management queues information with device side.
 * In case command / notification queue index is located on device memory
 * set ring producer / consumer to device memory
 * else set ring producer / consumer to notification area
 */
static int agnic_mgmt_set_mgmt_queues(struct agnic_pfio *pfio)
{
	struct agnic_q_hw_info	*cmd_q_info, *notif_q_info;
	int timeout = 1000; /* ~1 second. */

	/* Set CMD queue base address & length. */
	cmd_q_info = &pfio->nic_cfg->cmd_q;
	cmd_q_info->q_addr = pfio->cmd_ring.dma;
	cmd_q_info->len  = pfio->cmd_ring.count;
	cmd_q_info->q_prod_offs = AGNIC_RING_PROD_INDX_LOCAL_PHYS(&pfio->cmd_ring);
	cmd_q_info->q_cons_offs = AGNIC_RING_CONS_INDX_LOCAL_PHYS(&pfio->cmd_ring);

	/* Set Notification queue base address & length. */
	notif_q_info = &pfio->nic_cfg->notif_q;
	notif_q_info->q_addr = pfio->notif_ring.dma;
	notif_q_info->len  = pfio->notif_ring.count;
	notif_q_info->q_prod_offs = AGNIC_RING_PROD_INDX_LOCAL_PHYS(&pfio->notif_ring);
	notif_q_info->q_cons_offs = AGNIC_RING_CONS_INDX_LOCAL_PHYS(&pfio->notif_ring);

	/* Make sure that upper writes are executed before notifying the
	 * end-point.
	 */
	/* Notify the AGNIC */
	writel(readl(&pfio->nic_cfg->status) | AGNIC_CFG_STATUS_HOST_MGMT_READY, &pfio->nic_cfg->status);

	/* Wait for device to setup mgmt queues. */
	do {
		if (readl(&pfio->nic_cfg->status) & AGNIC_CFG_STATUS_DEV_MGMT_READY)
			break;
		udelay(1000);
		timeout--;
	} while (timeout);

	if (timeout == 0)
		pr_err("Timeout while waiting for device response.\n");

	return 0;
}

/*
** agnic_free_ring_resources - Free Rx/Tx/cmd/Notif/... ring resources (Descriptors)
*/
static int agnic_free_ring_resources(struct agnic_ring *ring)
{
	kfree(ring->cookie_list);
	ring->cookie_list = NULL;

	mv_sys_dma_mem_free(ring->desc);
	ring->desc = NULL;

	agnic_put_q_idx(ring->pfio, ring->prod_idx);
	agnic_put_q_idx(ring->pfio, ring->cons_idx);

	return 0;
}

/*
** agnic_alloc_ring_resources - allocate Rx/Tx/cmd/Notif/... ring resources (Descriptors)
*/
static int agnic_alloc_ring_resources(struct agnic_ring *ring, int desc_size, int ring_len, int alloc_b_info)
{
	int err = -ENOMEM, size;

	ring->count = ring_len;
	/* For cmd queues, the cookie holds data regarding the command. Hence, it should contain
	 * enough entries so it can serve all commands.
	 */
	ring->cookie_count = (ring->type == agnic_cmd_ring) ?
				max(ring->count * 2, (u32)AGNIC_CMD_Q_MAX_COOKIE_LEN) : ring->count;
	ring->cookie_list = NULL;
	if (alloc_b_info) {
		size = sizeof(struct agnic_cookie) * ring->cookie_count;
		ring->cookie_list = kzalloc(size, GFP_KERNEL);
		if (ring->cookie_list == NULL) {

			pr_err("Failed to allocate %d Bytes for cookie_list.\n", size);
			goto err;
		}
	}

	/* Allocate a DMA'able queue memory. */
	size = ring->count * desc_size;
	ring->desc = mv_sys_dma_mem_alloc(size, AGNIC_RINGS_ADDR_ALIGN);
	if (!ring->desc) {
		pr_err("Failed to allocate %d Bytes for descriptors ring.", size);
		goto err;
	}
	ring->dma = mv_sys_dma_mem_virt2phys(ring->desc);
	memset(ring->desc, 0, size);

	/* Get an entry in the queue indeces memory, to be pointed to by the
	 * consumer / producer pointer.
	 */
	err = agnic_get_q_idx(ring->pfio);
	if (err < 0) {
		pr_err("Unable to allocate entry for ring control pointers.\n");
		goto err;
	}
	ring->prod_idx = (u16)err;
	ring->producer_p = ring->pfio->ring_indices_arr + ring->prod_idx;
	err = agnic_get_q_idx(ring->pfio);
	if (err < 0) {
		pr_err("Unable to allocate entry for ring control pointers.\n");
		goto err;
	}
	ring->cons_idx = (u16)err;
	ring->consumer_p = ring->pfio->ring_indices_arr + ring->cons_idx;

	return 0;
err:
	ring->producer_p = NULL;
	ring->consumer_p = NULL;
	if (ring->cons_idx)
		agnic_put_q_idx(ring->pfio, ring->cons_idx);
	if (ring->prod_idx)
		agnic_put_q_idx(ring->pfio, ring->prod_idx);
	if (ring->desc)
		mv_sys_dma_mem_free(ring->desc);
	kfree(ring->cookie_list);
	ring->cookie_list = NULL;
	return err;
}

/*
** agnic_setup_mgmt_rings - Allocate cmd and notif rings.
*/
static int agnic_setup_mgmt_rings(struct agnic_pfio *pfio)
{
	struct agnic_ring *ring;
	int ret;

	/* Command Ring. */
	ring = &pfio->cmd_ring;
	ring->pfio = pfio;
	ring->type = agnic_cmd_ring;
	ret = agnic_alloc_ring_resources(ring, sizeof(struct agnic_cmd_desc),
			AGNIC_CMD_Q_LEN, true);
	if (ret) {
		pr_err("Failed to allocate command Queue.\n");
		goto cmd_err;
	}

	/* Notification Queue. */
	ring = &pfio->notif_ring;
	ring->pfio = pfio;
	ring->type = agnic_notif_ring;
	ret = agnic_alloc_ring_resources(ring, sizeof(struct agnic_cmd_desc),
			AGNIC_NOTIF_Q_LEN, false);
	if (ret) {
		pr_err("Failed to allocate NOTIF Queue.\n");
		goto notif_err;
	}

	/* Now set the MGMT queues pointers in HW. */
	ret = agnic_mgmt_set_mgmt_queues(pfio);
	if (ret)
		goto q_setup_err;

	/* We still don't have interrupts enabled at this stage, work in
	 * polling mode.
	 */
	pfio->flags |= AGNIC_FLAG_MGMT_POLL;

	spin_lock_init(&pfio->mgmt_lock);

	return 0;
q_setup_err:
	agnic_free_ring_resources(&pfio->notif_ring);
notif_err:
	agnic_free_ring_resources(&pfio->cmd_ring);
cmd_err:
	return ret;

}

/*
** agnic_alloc_data_rings - Initialize all Rx / Tx Queues resources.
*/
static int agnic_alloc_data_rings(struct agnic_pfio *pfio, enum agnic_ring_type type)
{
	int i, err = 0;
	int desc_size, num_rings, ring_len;
	struct agnic_ring **ring_ptrs;
	int alloc_cookies;
	char *name;

	if (type == agnic_rx_ring) {
		name = "Rx";
		desc_size = sizeof(struct agnic_pfio_desc);
		ring_ptrs = pfio->rx_ring;
		num_rings = pfio->num_rx_queues;
		ring_len = pfio->rx_ring_size;
		alloc_cookies = 0;
	} else {
		name = "Tx";
		desc_size = sizeof(struct agnic_pfio_desc);
		ring_ptrs  = pfio->tx_ring;
		num_rings = pfio->num_tx_queues;
		ring_len = pfio->tx_ring_size;
		alloc_cookies = 1;
	}

	for (i = 0; i < num_rings; i++) {
		ring_ptrs[i] = kzalloc(sizeof(struct agnic_ring), GFP_KERNEL);
		if (!ring_ptrs[i]) {
			pr_err("no mem for AGNIC rx ring obj!");
			err = -ENOMEM;
			goto err;
		}
		memset(ring_ptrs[i], 0, sizeof(struct agnic_ring));
		ring_ptrs[i]->pfio = pfio;
		ring_ptrs[i]->type = type;
		err = agnic_alloc_ring_resources(ring_ptrs[i], desc_size, ring_len, alloc_cookies);
		if (!err)
			continue;

		pr_err("Allocation for %s Queue %u failed\n", name, i);
		goto err;
	}

	return 0;
err:
	/* Free all allocated rings */
	while (i--)
		agnic_free_ring_resources(ring_ptrs[i]);
	return err;
}

/*
 * agnic_alloc_bp_rings - Allocate and fill NIC's Rx buffers pool(s).
 */
static int agnic_alloc_bp_rings(struct agnic_pfio *pfio)
{
	int err, i;

	for (i = 0; i < pfio->num_rx_queues; i++) {
		pfio->bp_ring[i].pfio = pfio;
		pfio->bp_ring[i].type = agnic_bpool_ring;
		err = agnic_alloc_ring_resources(&(pfio->bp_ring[i]), sizeof(struct agnic_bpool_desc),
					pfio->rx_ring_size, true);
		if (err) {
			pr_err("Failed to allocate buffers pool %d.\n", i);
			break;
		}

		pfio->bp_ring[i].bp_frag_sz = pfio->buff_size;
	}

	if (err) {
		while (i) {
			agnic_free_ring_resources(&pfio->bp_ring[i - 1]);
			i--;
		}
	}

	return err;
}

static int agnic_arrange_tcs(struct agnic_pfio *pfio, struct agnic_pfio_init_params *params)
{
	u8	i, j;

	for (i = 0; i < params->num_in_tcs; i++) {
		pfio->in_tcs[i].num_qs = params->num_qs_per_tc;
		pfio->in_tcs[i].buff_size = params->buff_size;
		pfio->in_tcs[i].pkt_offset = params->pkt_offset;

		for (j = 0; j < params->num_qs_per_tc; j++) {
			pfio->in_tcs[i].rings[j] = pfio->rx_ring[i * params->num_qs_per_tc + j];
			pfio->in_tcs[i].rings[j]->tc = i;
			pfio->in_tcs[i].rings[j]->q_idx = j;
			pfio->in_tcs[i].bp_rings[j] = &pfio->bp_ring[i * params->num_qs_per_tc + j];
			pfio->in_tcs[i].bp_rings[j]->tc = i;
			pfio->in_tcs[i].bp_rings[j]->q_idx = j;
		}
	}

	for (i = 0; i < params->num_out_tcs; i++) {
		pfio->out_tcs[i].num_qs = params->num_qs_per_tc;

		for (j = 0; j < params->num_qs_per_tc; j++) {
			pfio->out_tcs[i].rings[j] = pfio->tx_ring[i * params->num_qs_per_tc + j];
			pfio->out_tcs[i].rings[j]->tc = i;
			pfio->out_tcs[i].rings[j]->q_idx = j;
		}
	}

	return 0;
}


int agnic_pfio_init(struct agnic_pfio_init_params *params, struct agnic_pfio **pfio)
{
	struct agnic_pfio	*_pfio;
	int			 err, timeout = 5000;

	_pfio = kzalloc(sizeof(struct agnic_pfio), GFP_KERNEL);
	if (!_pfio)
		return -ENOMEM;
	memset(_pfio, 0, sizeof(struct agnic_pfio));

	_pfio->num_in_tcs = params->num_in_tcs;
	_pfio->num_out_tcs = params->num_out_tcs;
	_pfio->num_qs_per_tc = params->num_qs_per_tc;
	if (_pfio->num_qs_per_tc > 1)
		_pfio->hash_type = params->hash_type;
	_pfio->buff_size = params->buff_size;
	_pfio->pkt_offset = params->pkt_offset;

	agnic_set_num_queues(_pfio);

	if (params->pci_mode)
		err = init_pfio_pci_mem(_pfio, params);
	else
		err = init_pfio_plat_uio_mem(_pfio);
	if (err) {
		agnic_pfio_deinit(_pfio);
		return err;
	}

	/* Wait until the device firmware sets up the BARs */
	do {
		if (readl(&_pfio->nic_cfg->status) & AGNIC_CFG_STATUS_DEV_READY)
			break;
		udelay(1000);
		timeout--;
	} while (timeout);

	if (timeout == 0) {
		pr_err("Timeout while waiting for device ready.\n");
		agnic_pfio_deinit(_pfio);
		return err;
	}

	/* Get the MAC address out of the NIC configuration space. */
	/* copy the eth-addr with 4B alignment in order to prevent issues with some PCIe busses */
	*(u32 *)(_pfio->mac) = *(u32 *)(_pfio->nic_cfg->mac_addr);
	*(u32 *)(&_pfio->mac[4]) = *(u32 *)(&_pfio->nic_cfg->mac_addr[4]);

	/* Initialize _pfio fields */
	_pfio->tx_ring_size = params->out_qs_size;
	_pfio->rx_ring_size = params->in_qs_size;

	err = agnic_init_q_indeces(_pfio);
	if (err) {
		agnic_pfio_deinit(_pfio);
		return err;
	}

	err = agnic_setup_mgmt_rings(_pfio);
	if (err) {
		agnic_pfio_deinit(_pfio);
		return err;
	}

	/* Allocate transmit descriptors */
	pr_debug("Allocating Tx data rings.\n");
	err = agnic_alloc_data_rings(_pfio, agnic_tx_ring);
	if (err) {
		agnic_pfio_deinit(_pfio);
		return err;
	}

	/* Allocate receive descriptors */
	pr_debug("Allocating Rx data rings.\n");
	err = agnic_alloc_data_rings(_pfio, agnic_rx_ring);
	if (err) {
		agnic_pfio_deinit(_pfio);
		return err;
	}

	/* Allocate Rx buffer pools. */
	pr_debug("Allocating BP rings.\n");
	err = agnic_alloc_bp_rings(_pfio);
	if (err) {
		agnic_pfio_deinit(_pfio);
		return err;
	}

	err = agnic_init_pfio(_pfio);
	if (err) {
		agnic_pfio_deinit(_pfio);
		return err;
	}

	err = agnic_arrange_tcs(_pfio, params);
	if (err) {
		agnic_pfio_deinit(_pfio);
		return err;
	}

	*pfio = _pfio;
	return 0;
}

void agnic_pfio_deinit(struct agnic_pfio *pfio)
{
	u8	i, j;

	if (!pfio)
		return;

	for (i = 0; i < pfio->num_in_tcs; i++)
		for (j = 0; j < pfio->in_tcs[i].num_qs; j++)
			if (pfio->in_tcs[i].rings[j])
				agnic_free_ring_resources(pfio->in_tcs[i].rings[j]);

	for (i = 0; i < pfio->num_out_tcs; i++)
		for (j = 0; j < pfio->out_tcs[i].num_qs; j++)
			if (pfio->out_tcs[i].rings[j])
				agnic_free_ring_resources(pfio->out_tcs[i].rings[j]);

	agnic_free_ring_resources(&pfio->notif_ring);
	agnic_free_ring_resources(&pfio->cmd_ring);

	kfree(pfio);
}

int agnic_mgmt_notif_process(struct agnic_pfio *pfio, u16 cmd_code, void *msg, u16 len)
{
	struct agnic_mgmt_notification *resp = (struct agnic_mgmt_notification *)msg;

	if (!pfio) {
		pr_err("no pfio obj!");
		return -EINVAL;
	}

	pr_debug("Received notification id %d\n", cmd_code);

	switch (cmd_code) {
	case NC_PF_LINK_CHANGE:
		pfio->link = resp->link_status;
		/* TODO: what do we need to do upon 'link-down/up' ? */
		pr_info("got link %s\n", pfio->link ? "up" : "down");
		break;
	case NC_PF_KEEP_ALIVE:
		/* TODO: what do we need to do upon 'KA' ? */
		pr_debug("got KA\n");
		break;
	default:
		/* Unknown command code */
		pr_err("Unknown command code %d!! Unable to process command.\n", cmd_code);
		return -EOPNOTSUPP;
	}

	return 0;
}

int agnic_pfio_send(struct agnic_pfio		*pfio,
		  u8				 tc,
		  u8				 qid,
		  struct agnic_pfio_desc	*descs,
		  u16				*num)
{
	struct agnic_ring *txq;
	struct agnic_pfio_desc *tx_ring_base;
	u16 num_txds = *num, desc_remain;
	u16 block_size, index;
	u32 free_count, cons_val;

	/* Get queue params */
	txq = pfio->out_tcs[tc].rings[qid];

	/* Get ring base */
	tx_ring_base = (struct agnic_pfio_desc *)txq->desc;

	/* Read consumer index */
	cons_val = readl(txq->consumer_p);

	/* Calculate number of free descriptors */
	free_count = AGNIC_RING_FREE(txq->tx_prod_shadow, cons_val, txq->count);
	if (unlikely(free_count < num_txds)) {
		pr_debug("num_txds(%d), free_count(%d) (PFIO %d)\n", num_txds, free_count, pfio->id);
		num_txds = free_count;
	}

	if (unlikely(!num_txds)) {
		pr_debug("AGNIC outQ full\n");
		*num = 0;
		return 0;
	}

	/* In case there is a wrap-around, handle the number of desc till the end of queue */
	block_size = min(num_txds, (u16)(txq->count - txq->tx_prod_shadow));

	desc_remain = num_txds;
	index = 0; /* index in source descriptor array */

	/* In case there is a wrap-around, the first iteration will handle the
	 * descriptors till the end of queue. The rest will be handled at the
	 * following iteration.
	 * Note that there should be no more than 2 iterations.
	 **/
	do {
		/* Copy bulk of descriptors to descriptor ring */
		memcpy(&tx_ring_base[txq->tx_prod_shadow], &descs[index], sizeof(*tx_ring_base) * block_size);

		/* Increment producer index, update remaining descriptors count and block size */
		AGNIC_RING_PTR_INC(txq->tx_prod_shadow, block_size, txq->count);
		desc_remain -= block_size;
		index = block_size;	/* next desc index in source array */
		block_size = desc_remain; /* next desc index in target ring */
	} while (desc_remain > 0);

	/* Update Producer index in GNPT */
	/* make sure all writes are done (i.e. descriptor were copied)
	 * before incrementing the producer index
	 */
	writel(txq->tx_prod_shadow, txq->producer_p);

	/* Update number of sent descriptors */
	*num = num_txds;

	return 0;
}

int agnic_pfio_get_num_outq_done(struct agnic_pfio	*pfio,
				 u8			 tc,
				 u8			 qid,
				 u16			*num)
{
	u32 tx_num = 0;
	u32 cons_val;
	struct agnic_ring *txq = pfio->out_tcs[tc].rings[qid];

	/* Read consumer index */
	cons_val = readl_relaxed(txq->consumer_p);

	tx_num = AGNIC_RING_NUM_OCCUPIED(cons_val, txq->tx_next_to_reclaim, txq->count);
	txq->tx_next_to_reclaim = cons_val;
	*num = tx_num;

	return 0;
}

int agnic_pfio_recv(struct agnic_pfio		*pfio,
		    u8				 tc,
		    u8				 qid,
		    struct agnic_pfio_desc	*descs,
		    u16				*num)
{
	struct agnic_ring *rxq;
	struct agnic_pfio_desc *rx_ring_base;
	u16 recv_req = *num, desc_received, desc_remain = 0;
	u16 block_size, index;
	u32 prod_val;

	*num = 0;

	rxq = pfio->in_tcs[tc].rings[qid];

	/* Get ring base */
	rx_ring_base = (struct agnic_pfio_desc *)rxq->desc;

	/* Read producer index */
	prod_val = readl(rxq->producer_p);

	/* Calculate number of received descriptors in the ring.
	 * Since queue size is a power of 2, we can use below formula.
	 */
	desc_received = AGNIC_RING_NUM_OCCUPIED(prod_val, rxq->rx_cons_shadow, rxq->count);
	if (desc_received == 0) {
		pr_debug("desc_received is zero\n");
		return 0;
	}

	recv_req = min(recv_req, desc_received);

	/* In case there is a wrap around the descriptors are be stored to the
	 * end of the ring AND from the beginning of the desc ring.
	 * So the size of the first block is the number of descriptor till the
	 * end of the ring.
	 */
	if (unlikely((rxq->rx_cons_shadow + recv_req) > rxq->count)) {
		block_size = rxq->count - rxq->rx_cons_shadow;
	} else {
		/* No wrap around */
		block_size = recv_req;
	}

	desc_remain = recv_req;
	index = 0; /* index in destination descriptor array */

	/* Note: since we handle wrap-around, the should be no more than 2 iterations */
	do {
		/* Copy bulk of descriptors from the descriptor ring */
		memcpy(&descs[index], &rx_ring_base[rxq->rx_cons_shadow], block_size * sizeof(*descs));

		rx_ring_base[rxq->rx_cons_shadow].cmds[7] = AGNIC_COOKIE_DRIVER_WATERMARK;

		/* Increment consumer index, update remaining descriptors count and block size */
		AGNIC_RING_PTR_INC(rxq->rx_cons_shadow, block_size, rxq->count);
		desc_remain -= block_size;
		index = block_size; /* next desc index in destination array */
		block_size = desc_remain; /* next desc index in source ring */
	} while (desc_remain);

	/* Update Consumer index in GNCT */
	/* make sure all writes are done (i.e. descriptor were copied)
	 * before incrementing the consumer index
	 */
	writel(rxq->rx_cons_shadow, rxq->consumer_p);

	/* Update number of received descriptors */
	*num = recv_req;

	return 0;
}

int agnic_pfio_inq_put_buffs(struct agnic_pfio		*pfio,
			     u8				 tc,
			     u8				 qid,
			     struct agnic_buff_inf	*bufs,
			     u16			*num)
{
	struct agnic_ring *bpq;
	struct agnic_buff_inf *buf_desc;
	u16 num_bpds = *num, desc_remain;
	u16 block_size, index;
	u32 free_count, cons_val;

	/* Get queue params */
	bpq = pfio->in_tcs[tc].bp_rings[qid];

	/* Read consumer index */
	cons_val = readl(bpq->consumer_p);

	/* Calculate number of free descriptors */
	free_count = AGNIC_RING_FREE(bpq->tx_prod_shadow, cons_val, bpq->count);

	if (unlikely(free_count < num_bpds)) {
		pr_debug("num_bpds(%d), free_count(%d) (BPool %d)\n", num_bpds, free_count, pool->id);
		num_bpds = free_count;
	}

	if (unlikely(!num_bpds)) {
		pr_debug("BPool full\n");
		*num = 0;
		return 0;
	}

	/* In case there is a wrap-around, handle the number of desc till the end of queue */
	block_size = min(num_bpds, (u16)(bpq->count - bpq->tx_prod_shadow));

	desc_remain = num_bpds;
	index = 0; /* index in source descriptor array */

	/* Get ring base */
	buf_desc = (struct agnic_buff_inf *)bpq->desc;

	/* In case there is a wrap-around, the first iteration will handle the
	 * descriptors till the end of queue. The rest will be handled at the
	 * following iteration.
	 * Note that there should be no more than 2 iterations.
	 **/
	do {
		/* Copy bulk of descriptors to descriptor ring */
		memcpy(&buf_desc[bpq->tx_prod_shadow], &bufs[index], sizeof(struct agnic_buff_inf) * block_size);

		/* Increment producer index, update remaining descriptors count and block size */
		AGNIC_RING_PTR_INC(bpq->tx_prod_shadow, block_size, bpq->count);
		desc_remain -= block_size;
		index = block_size;	/* next desc index in source array */
		block_size = desc_remain; /* next desc index in target ring */
	} while (desc_remain > 0);

	/* Update Producer index in GNPT */
	/* make sure all writes are done (i.e. descriptor were copied)
	 * before incrementing the producer index
	 */
	writel(bpq->tx_prod_shadow, bpq->producer_p);

	/* Update number of updated descriptors */
	*num = num_bpds;

	return 0;
}
