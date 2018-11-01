/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#include "std_internal.h"

#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "lib/lib_misc.h"

#include "giu_queue_topology.h"
#include "file_map.h"

#define REGFILE_SUPPORTED_VERSION	000002

struct giu_gpio_probe_params gpio_probe_params[GIU_MAX_NUM_GPIO];
void *map_addr[GIU_MAX_NUM_GPIO];

static int giu_gpio_read_tc_config(void **tc_config_base_addr, struct giu_gpio_qs_params *qs_params,
				uintptr_t qs_va_base, uintptr_t ptrs_va_base)
{
	struct giu_tc *giu_tc;
	struct giu_queue *giu_queue;

	void *read_addr = *tc_config_base_addr;
	int tc_id, queue_id;
	int ret;

	qs_params->tcs = kmalloc((qs_params->num_tcs * sizeof(struct giu_gpio_tc)), GFP_KERNEL);
	if (!qs_params->tcs) {
		ret = -ENOMEM;
		goto error;
	}
	memset(qs_params->tcs, 0, qs_params->num_tcs * sizeof(struct giu_gpio_tc));

	for (tc_id = 0; tc_id < qs_params->num_tcs; tc_id++) {
		struct giu_gpio_tc *tc_params = &qs_params->tcs[tc_id];

		/* Read TC Configuration */
		giu_tc = (struct giu_tc *)read_addr;
		read_addr += sizeof(struct giu_tc);

		if (giu_tc->id != tc_id) {
			pr_err("TC ID %d (in regfile) is different from expected ID (%d)\n",
				giu_tc->id, tc_id);
			ret = -EFAULT;
			goto error;
		}

		/* Set TC Parameters */
		tc_params->dest_num_qs = giu_tc->dest_num_queues;
		tc_params->hash_type = (enum giu_gpio_hash_type)giu_tc->ingress_rss_type;
		tc_params->num_qs = giu_tc->num_queues;
		tc_params->queues = kmalloc((tc_params->num_qs * sizeof(struct giu_gpio_queue)), GFP_KERNEL);
		if (!tc_params->queues) {
			pr_err("Failed to allocate TC %d queues\n", tc_id);
			ret = -ENOMEM;
			goto error;
		}
		memset(tc_params->queues, 0, tc_params->num_qs * sizeof(struct giu_gpio_queue));

		for (queue_id = 0; queue_id < tc_params->num_qs; queue_id++) {
			struct giu_gpio_queue *gpio_q = &tc_params->queues[queue_id];

			/* Read Queue Configuration */
			giu_queue = (struct giu_queue *)read_addr;
			read_addr += sizeof(struct giu_queue);

			/* Set Queue Parameters */
			gpio_q->desc_ring_base = (struct giu_gpio_desc *)(qs_va_base + giu_queue->phy_base_offset);
			gpio_q->desc_total = giu_queue->size;
			gpio_q->cons_addr = (u32 *)(ptrs_va_base + giu_queue->cons_offset);
			gpio_q->prod_addr = (u32 *)(ptrs_va_base + giu_queue->prod_offset);
			gpio_q->payload_offset = giu_queue->payload_offset;

			pr_debug("Queue Params (TC %d Q %d):\n", giu_tc->id, giu_queue->hw_id);
			pr_debug("\tdesc_ring_base %p\n", gpio_q->desc_ring_base);
			pr_debug("\tring size 0x%x\n", gpio_q->desc_total);
			pr_debug("\tcons_addr %p\n", gpio_q->cons_addr);
			pr_debug("\tprod_addr %p\n", gpio_q->prod_addr);
			pr_debug("\tpayload_offset %d\n", gpio_q->payload_offset);
		}
	}

	/* Update new config address */
	*tc_config_base_addr = read_addr;

	return 0;

error:
	if (!qs_params->tcs)
		return ret;
	/* Free allocated memory for TCs and Queues */
	for (tc_id = 0; tc_id < qs_params->num_tcs; tc_id++) {
		struct giu_gpio_tc *tc_params = &qs_params->tcs[tc_id];

		if (tc_params->queues) {
			/* Free Queue Array */
			kfree(tc_params->queues);
		}
	}

	/* Free TC Array */
	kfree(qs_params->tcs);

	return ret;
}

static int giu_gpio_read_bp_config(void **bm_config_base_addr, struct giu_gpio_queue *bp_params,
				uintptr_t qs_va_base, uintptr_t ptrs_va_base)
{
	void *read_addr = *bm_config_base_addr;
	struct giu_queue *bp_queue; /* Regfile BP data */

	/* Read BM Configuration */
	bp_queue = (struct giu_queue *)read_addr;
	read_addr += sizeof(struct giu_queue);

	bp_params->desc_ring_base = (struct giu_gpio_desc *)(qs_va_base + bp_queue->phy_base_offset);
	bp_params->desc_total = bp_queue->size;
	bp_params->cons_addr = (u32 *)(ptrs_va_base + bp_queue->cons_offset);
	bp_params->prod_addr = (u32 *)(ptrs_va_base + bp_queue->prod_offset);
	bp_params->buff_len = bp_queue->buff_len;
	pr_debug("Queue Params (BP %d):\n", bp_queue->hw_id);
	pr_debug("\tdesc_ring_base %p\n", bp_params->desc_ring_base);
	pr_debug("\tring size 0x%x\n", bp_params->desc_total);
	pr_debug("\tcons_addr %p\n", bp_params->cons_addr);
	pr_debug("\tprod_addr %p\n", bp_params->prod_addr);
	pr_debug("\tbuff length %d\n", bp_params->buff_len);

	/* Update new config address */
	*bm_config_base_addr = read_addr;

	return 0;
}

static int giu_gpio_read_regfile(void *regs_addr, struct giu_gpio_probe_params *gpio_probe_params)
{
	struct giu_regfile *reg_data;
	void *read_addr = regs_addr;
	int ret;
	struct sys_iomem_params		iomem_params;
	struct sys_iomem_info		sys_iomem_info;
	uintptr_t			qs_va, ptrs_va;

	/* Read PF Configuration */
	reg_data = (struct giu_regfile *)read_addr;
	read_addr += sizeof(struct giu_regfile);

	/* Verify version */
	if (reg_data->version != REGFILE_SUPPORTED_VERSION) {
		pr_err("Regfile version 0x%06x is not supported (expected 0x%06x)\n",
			reg_data->version, REGFILE_SUPPORTED_VERSION);
		return -ENOTSUP;
	}

	/* Set BP configurations */
	if (reg_data->num_bm_qs != 1) {
		pr_err("GPIO: only single BP is supported (Regfile contains %d)\n", reg_data->num_bm_qs);
		return -ENOTSUP;
	}

	pr_debug("dma_uio_mem_name (%s), flags 0x%x\n",
		 reg_data->dma_uio_mem_name, reg_data->flags);

	iomem_params.type = SYS_IOMEM_T_SHMEM;
	iomem_params.devname = reg_data->dma_uio_mem_name;
	iomem_params.index = 1;

	if (sys_iomem_get_info(&iomem_params, &sys_iomem_info)) {
		pr_err("sys_iomem_get_info error\n");
		return -EFAULT;
	}

	qs_va = (uintptr_t)sys_iomem_info.u.shmem.va;
	ptrs_va = qs_va;

	ret = giu_gpio_read_bp_config(&read_addr, &gpio_probe_params->bpool, qs_va, ptrs_va);
	if (ret) {
		pr_err("Failed to read Ingress TC configuration (%d)\n", ret);
		return ret;
	}

	/* Set Ingress TC configurations */
	gpio_probe_params->inqs_params.num_tcs = reg_data->num_ingress_tcs;
	ret = giu_gpio_read_tc_config(&read_addr, &gpio_probe_params->inqs_params, qs_va, ptrs_va);
	if (ret) {
		pr_err("Failed to read Ingress TC configuration (%d)\n", ret);
		return ret;
	}

	/* Set Egress TC configurations */
	gpio_probe_params->outqs_params.num_tcs = reg_data->num_egress_tcs;
	ret = giu_gpio_read_tc_config(&read_addr, &gpio_probe_params->outqs_params, qs_va, ptrs_va);
	if (ret) {
		pr_err("Failed to read Ingress TC configuration (%d)\n", ret);
		return ret;
	}

	return 0;
}

int giu_gpio_init_topology(int giu_id, char *regfile_name)
{
	struct file_map_info	map_info;
	int			ret;

	if (giu_id >= GIU_MAX_NUM_GPIO)	{
		pr_err("giu_id (%d) exceeds mac gpio number (%d)\n", giu_id, GIU_MAX_NUM_GPIO);
		return -1;
	}

	if (regfile_name == NULL) {
		pr_err("Register file name was not set\n");
		return -1;
	}

	if (map_addr[giu_id] != NULL) {
		pr_debug("GIU %d queue topology is already initialized\n", giu_id);
		return 0;
	}

	ret = file_map(regfile_name, &map_info);
	if (ret) {
		pr_err("Failed to map register file\n");
		return -1;
	}

	/* Read Q topology from register file and update
	 * GPIO internal structure
	 */
	ret = giu_gpio_read_regfile(map_info.map_addr, &gpio_probe_params[giu_id]);
	if (ret) {
		pr_err("Failed to read Register file configuration (%d)\n", ret);
		return ret;
	}

	/* save the mapping address for later use */
	map_addr[giu_id] = map_info.map_addr;

	return 0;
}

int giu_gpio_deinit_topology(int giu_id)
{
	pr_err("giu_gpio_init_topology is not implemented\n");

	return 0;
}

int giu_gpio_topology_set_init_done(int giu_id)
{
	struct giu_regfile *reg_data = (struct giu_regfile *)map_addr[giu_id];

	if (reg_data == NULL) {
		pr_err("GIU %d register file is not mapped\n", giu_id);
		return -1;
	}

	return 0;
}

/*
 *	giu_regfile_register_queue
 *
 *	This function register Queue params in Regfile Queue structure
 *	It gets the info from the SNIC-DB and finally update directly the regfile
 *
 *	hw_q_id    - The real unique Queue index
 *	q_type     - The type of the Queue (Egress / Ingress / BM/...)
 *	file_map   - Pointer to Pointer to Regfile mempry map
 *
 *	@retval	0 on success
 *	@retval	error-code otherwise (< 0)
 */
int giu_regfile_register_queue(union giu_gpio_q_params *giu_gpio_q_p,
			       enum giu_queue_type q_type,
			       u32 q_add_data,
			       phys_addr_t qs_phys_base,
			       phys_addr_t ptrs_phys_base,
			       void **file_map)
{
	struct giu_queue reg_giu_queue;
	struct mqa_queue_info queue_info;

	if (giu_gpio_q_p == NULL) {
		pr_err("Failed to get queue params from DB (Queue: %d)\n", (int)giu_gpio_q_p->lcl_q.q_id);
		return -ENODEV;
	}

	mqa_queue_get_info(giu_gpio_q_p->lcl_q.q, &queue_info);

	reg_giu_queue.hw_id		= queue_info.q_id;
	/** TODO - change params naming - change reg_giu_queue.size to reg_giu_queue.len*/
	reg_giu_queue.size		= queue_info.len;
	reg_giu_queue.type		= q_type;
	reg_giu_queue.phy_base_offset	= (phys_addr_t)(uintptr_t)(queue_info.phy_base_addr - qs_phys_base);
	/* Prod/Cons addr are Virtual. Needs to translate them to offset */
	reg_giu_queue.prod_offset = (phys_addr_t)(uintptr_t)(queue_info.prod_phys - ptrs_phys_base);
	reg_giu_queue.cons_offset = (phys_addr_t)(uintptr_t)(queue_info.cons_phys - ptrs_phys_base);

	/* Note: buff_size & payload_offset are union and they are set
	 *	 acoording to the Q type.
	 */
	if (q_type == QUEUE_BP)
		/** TODO - change params naming - change buff_size to buff_len */
		reg_giu_queue.buff_len = q_add_data;
	else
		reg_giu_queue.payload_offset = q_add_data;

	/* Copy Queues parameters */
	pr_debug("\t\tCopy Queue %d Information to Regfile\n", reg_giu_queue.hw_id);
	memcpy(*file_map, &reg_giu_queue, sizeof(struct giu_queue));
	*file_map += sizeof(struct giu_queue);

	return 0;
}
