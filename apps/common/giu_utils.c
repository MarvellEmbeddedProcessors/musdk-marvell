/******************************************************************************
*  Copyright (C) 2018 Marvell International Ltd.
*
*  This program is provided "as is" without any warranty of any kind, and is
*  distributed under the applicable Marvell limited use license agreement.
*******************************************************************************/

#include <string.h>
#include "mv_std.h"
#include "giu_utils.h"
#include "pp2_utils.h" /* for shadow queue definitions */
#include "mv_giu_bpool.h"


u8 mvapp_giu_max_num_qs_per_tc;


int app_giu_port_init(int giu_id, struct giu_gpio **giu_gpio)
{
	struct giu_bpool *giu_bpool;
	char		 file_name[REGFILE_MAX_FILE_NAME];
	char		 name[20];
	int		 bpool_id = 0; /* TODO: get this value from higher levels */
	int		 gpio_id = 0; /* TODO: get this value from higher levels */
	int		 err;

	/* Map GIU regfile */
	snprintf(file_name, sizeof(file_name), "%s%s%d", REGFILE_VAR_DIR, REGFILE_NAME_PREFIX, 0);

	/* Create bpool match string */
	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "giu_pool-%d:%d", giu_id, bpool_id);

	/* Probe the bpool */
	err = giu_bpool_probe(name, file_name, &giu_bpool);
	if (err) {
		pr_err("GIU BPool Init failed (%d)\n", err);
		return -1;
	}

	/* Create gpio match string */
	memset(name, 0, sizeof(name));
	snprintf(name, sizeof(name), "gpio-%d:%d", giu_id, gpio_id);

	/* Probe the GIU GPIO */
	err = giu_gpio_probe(name, file_name, giu_gpio);
	if (err) {
		pr_err("GIU GPIO Init failed (%d)\n", err);
		return -1;
	}

	return 0;
}

void app_giu_port_local_init(int id,
			int lcl_id,
			int giu_id,
			struct lcl_giu_port_desc *lcl_port,
			struct giu_gpio *gpio)
{
	struct giu_gpio_capabilities capa;
	u32 outq_size;
	u16 num_outqs;
	u8 tc, q;
	int i;

	lcl_port->id		= id;
	lcl_port->lcl_id	= lcl_id;
	lcl_port->giu_id	= giu_id;
	lcl_port->gpio_id	= -1; /* TODO: see if it's needed */
	lcl_port->gpio		= gpio;

	giu_gpio_get_capabilities(lcl_port->gpio, &capa);
	num_outqs = capa.outtcs_inf.outtcs_inf[0].num_outqs;
	outq_size = capa.outtcs_inf.outtcs_inf[0].outqs_inf[0].size;

	for (tc = 0; tc < capa.outtcs_inf.num_outtcs; tc++) {
		if (capa.outtcs_inf.outtcs_inf[tc].num_outqs != num_outqs)
			pr_err("number of outQs must be equal in all TCs!!\n");
		for (q = 0; q < capa.outtcs_inf.outtcs_inf[tc].num_outqs; q++)
			if (capa.outtcs_inf.outtcs_inf[tc].outqs_inf[q].size != outq_size)
				pr_err("Size of ALL outQs must be equal!!\n");
	}

	lcl_port->num_shadow_qs = num_outqs;
	lcl_port->shadow_q_size	= outq_size;
	lcl_port->shadow_qs = (struct giu_tx_shadow_q *)malloc(num_outqs * sizeof(struct giu_tx_shadow_q));

	for (i = 0; i < lcl_port->num_shadow_qs; i++) {
		lcl_port->shadow_qs[i].read_ind = 0;
		lcl_port->shadow_qs[i].write_ind = 0;

		lcl_port->shadow_qs[i].ents =
			(struct giu_buff_ent *)malloc(outq_size * sizeof(struct giu_buff_ent));
	}
}

int app_giu_build_bpool(int bpool_id, u32 num_of_buffs)
{
	struct giu_bpool *bpool = &giu_bpools[bpool_id];
	struct giu_bpool_capabilities capa;
	void *buff_virt_addr;
	void *buff_phys_addr;
	int i, err;

	giu_bpool_get_capabilities(bpool, &capa);
	if (capa.buff_len != bpool->buff_len) {
		pr_err("missmatch in BPool buff-len (%d vs %d)!\n",
		capa.buff_len, bpool->buff_len);
		return -EFAULT;
	}
	if (capa.max_num_buffs < num_of_buffs)
		num_of_buffs = capa.max_num_buffs;

	pr_debug("Adding (%d Bytes) buffers into BPOOL.\n", bpool->buff_len);

	buff_virt_addr = mv_sys_dma_mem_alloc(bpool->buff_len * num_of_buffs, 4);
	if (!buff_virt_addr) {
		pr_err("failed to allocate giu bpool mem!\n");
		return -ENOMEM;
	}
	if (app_get_high_addr() == MVAPPS_INVALID_COOKIE_HIGH_BITS)
		app_set_high_addr((uintptr_t)buff_virt_addr & MVAPPS_COOKIE_HIGH_MASK);
	else if (((uintptr_t)buff_virt_addr & MVAPPS_COOKIE_HIGH_MASK) != app_get_high_addr()) {
		pr_err("app_allocate_bpool_buffs: upper 32-bits are 0x%x, should be 0x%x\n",
			upper_32_bits((uintptr_t)buff_virt_addr), upper_32_bits(app_get_high_addr()));
		return -EFAULT;
	}

	buff_phys_addr = (void *)(uintptr_t)mv_sys_dma_mem_virt2phys(buff_virt_addr);

	for (i = 0; i < num_of_buffs; i++) {
		struct giu_buff_inf buff_inf;

		buff_inf.addr = (u64)buff_phys_addr + (i * bpool->buff_len);
		buff_inf.cookie = (u64)buff_virt_addr + (i * bpool->buff_len);

		err = giu_bpool_put_buff(bpool, &buff_inf);
		if (err)
			return err;
	}

	return 0;
}
