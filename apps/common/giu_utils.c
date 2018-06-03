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

void app_giu_port_local_init(int id, int lcl_id, int giu_id, struct lcl_giu_port_desc *lcl_port, struct giu_gpio *gpio,
			     u16 num_outqs, u32 outq_size)
{
	int i;

	lcl_port->id		= id;
	lcl_port->lcl_id	= lcl_id;
	lcl_port->giu_id	= giu_id;
	lcl_port->gpio_id	= -1; /* TODO: see if it's needed */
	lcl_port->gpio		= gpio;

	lcl_port->num_shadow_qs = num_outqs;
	lcl_port->shadow_q_size	= outq_size;
	lcl_port->shadow_qs = (struct giu_tx_shadow_q *)malloc(num_outqs * sizeof(struct giu_tx_shadow_q));

	for (i = 0; i < lcl_port->num_shadow_qs; i++) {
		lcl_port->shadow_qs[i].read_ind = 0;
		lcl_port->shadow_qs[i].write_ind = 0;

		lcl_port->shadow_qs[i].buffs_inf =
			(struct giu_buff_inf *)malloc(outq_size * sizeof(struct giu_buff_inf));
	}
}

int app_giu_build_bpool(int bpool_id, struct bpool_inf *infs)
{
	struct giu_bpool *bpool = &giu_bpools[bpool_id];
	struct giu_buff_inf *buffs_inf;
	int num_of_buffs = infs->num_buffs - 1;
	void *buff_virt_addr;
	void *buff_phys_addr;
	int i, err;

	pr_debug("Adding (%d Bytes) buffers into BPOOL.\n", bpool->buff_len);

	buffs_inf = (struct giu_buff_inf *)malloc(num_of_buffs * sizeof(struct giu_buff_inf));
	if (buffs_inf == NULL) {
		pr_err("Failed to allocate buffs_inf\n");
		return -ENOMEM;
	}

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

	buff_phys_addr = (void *)mv_sys_dma_mem_virt2phys(buff_virt_addr);

	for (i = 0; i < num_of_buffs; i++) {
		buffs_inf[i].addr = (u64)buff_phys_addr + (i * bpool->buff_len);
		buffs_inf[i].cookie = (u64)buff_virt_addr + (i * bpool->buff_len);

		err = giu_bpool_put_buff(bpool, &buffs_inf[i]);
		if (err)
			return err;
	}

	return 0;
}
