/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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

#include "std_internal.h"

#include "drivers/mv_mqa.h"
#include "drivers/mv_mqa_queue.h"
#include "giu_queue_topology.h"
#include "drivers/giu_regfile_def.h"
#include "file_map.h"

#define REGFILE_SUPPORTED_VERSION	000002

struct giu_gpio_params giu_params[GIU_MAX_NUM_GPIO];
void *map_addr[GIU_MAX_NUM_GPIO];

/* TODO:
 * Need to create a GPIO virtual mapping of the prod / cons tables, in front of
 * the PCIe UIO driver.
 * For now, the mapping is passed from the management application to GPIO
 * using the register file.
 */
struct notify_table_map {
	void *prod_tbl_base_phys;
	void *prod_tbl_base_virt;
	void *cons_tbl_base_phys;
	void *cons_tbl_base_virt;
};

static int giu_gpio_map_queue(struct giu_gpio_queue *queue, struct notify_table_map *tables_map)
{
	queue->desc_ring_base = mv_sys_dma_mem_phys2virt((phys_addr_t)queue->desc_ring_base_phys);

	queue->cons_addr = tables_map->cons_tbl_base_virt +
				((void *)queue->cons_addr_phys - tables_map->cons_tbl_base_phys);
	queue->prod_addr = tables_map->prod_tbl_base_virt +
				((void *)queue->prod_addr_phys - tables_map->prod_tbl_base_phys);

	pr_debug("\tdesc_ring_base (virt) %p\n", queue->desc_ring_base);
	pr_debug("\tcons_addr (virt) %p\n", queue->cons_addr);
	pr_debug("\tprod_addr (virt) %p\n", queue->prod_addr);

	return 0;
}

static int giu_gpio_read_tc_config(void **tc_config_base_addr, struct giu_gpio_qs_params *qs_params,
				   struct notify_table_map *tables_map)
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
			gpio_q->desc_ring_base_phys = giu_queue->phy_base_addr;
			gpio_q->desc_total = giu_queue->size;
			gpio_q->cons_addr_phys = giu_queue->cons_addr;
			gpio_q->prod_addr_phys = giu_queue->prod_addr;
			gpio_q->payload_offset = giu_queue->payload_offset;

			pr_debug("Queue Params (TC %d Q %d):\n", giu_tc->id, giu_queue->hw_id);
			pr_debug("\tdesc_ring_base %p\n", giu_queue->phy_base_addr);
			pr_debug("\tsize 0x%x\n", giu_queue->size);
			pr_debug("\tcons_addr %p\n", giu_queue->cons_addr);
			pr_debug("\tprod_addr %p\n", giu_queue->prod_addr);
			pr_debug("\tpayload_offset %d\n", giu_queue->payload_offset);

			ret = giu_gpio_map_queue(gpio_q, tables_map);
			if (ret)
				goto error;
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
				   struct notify_table_map *tables_map)
{
	void *read_addr = *bm_config_base_addr;
	struct giu_queue *bp_queue; /* Regfile BP data */
	int ret = 0;

	/* Read BM Configuration */
	bp_queue = (struct giu_queue *)read_addr;
	read_addr += sizeof(struct giu_queue);

	bp_params->desc_ring_base_phys = bp_queue->phy_base_addr;
	bp_params->desc_total = bp_queue->size;
	bp_params->cons_addr_phys = bp_queue->cons_addr;
	bp_params->prod_addr_phys = bp_queue->prod_addr;
	bp_params->buff_len = bp_queue->buff_len;
	pr_debug("Queue Params (BP %d):\n", bp_queue->hw_id);
	pr_debug("\tdesc_ring_base %p\n", bp_queue->phy_base_addr);
	pr_debug("\tsize 0x%x\n", bp_queue->size);
	pr_debug("\tcons_addr %p\n", bp_queue->cons_addr);
	pr_debug("\tprod_addr %p\n", bp_queue->prod_addr);
	pr_debug("\tbuff length %d\n", bp_queue->buff_len);

	ret = giu_gpio_map_queue(bp_params, tables_map);
	if (ret)
		return ret;
	/* Update new config address */
	*bm_config_base_addr = read_addr;

	return 0;
}

static int giu_gpio_read_regfile(void *regs_addr, struct giu_gpio_params *giu_params)
{
	struct giu_regfile *reg_data;
	struct notify_table_map tables_map;
	void *read_addr = regs_addr;
	int ret;

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

	/* Set notification tables virtual and physical addresses */
	tables_map.prod_tbl_base_phys = reg_data->prod_tbl_base_phys;
	tables_map.prod_tbl_base_virt = reg_data->prod_tbl_base_virt;
	tables_map.cons_tbl_base_phys = reg_data->cons_tbl_base_phys;
	tables_map.cons_tbl_base_virt = reg_data->cons_tbl_base_virt;

	ret = giu_gpio_read_bp_config(&read_addr, &giu_params->bpool, &tables_map);
	if (ret) {
		pr_err("Failed to read Ingress TC configuration (%d)\n", ret);
		return ret;
	}

	/* Set Ingress TC configurations */
	giu_params->inqs_params.num_tcs = reg_data->num_ingress_tcs;
	ret = giu_gpio_read_tc_config(&read_addr, &giu_params->inqs_params, &tables_map);
	if (ret) {
		pr_err("Failed to read Ingress TC configuration (%d)\n", ret);
		return ret;
	}

	/* Set Egress TC configurations */
	giu_params->outqs_params.num_tcs = reg_data->num_egress_tcs;
	ret = giu_gpio_read_tc_config(&read_addr, &giu_params->outqs_params, &tables_map);
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
	ret = giu_gpio_read_regfile(map_info.map_addr, &giu_params[giu_id]);
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

	/* Set 'Ready' indication */
	reg_data->flags |= REGFILE_READY;

	return 0;
}
