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

#include <string.h>
#include "mv_std.h"
#include "giu_utils.h"
#include "pp2_utils.h" /* for shadow queue definitions */
#include "mv_giu_bpool.h"


u8 mvapp_giu_max_num_qs_per_tc;


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
	lcl_port->shadow_q_size	= outq_size + 1;
	lcl_port->shadow_qs = (struct giu_tx_shadow_q *)malloc(num_outqs * sizeof(struct giu_tx_shadow_q));

	for (i = 0; i < lcl_port->num_shadow_qs; i++) {
		lcl_port->shadow_qs[i].read_ind = 0;
		lcl_port->shadow_qs[i].write_ind = 0;

		lcl_port->shadow_qs[i].ents =
			(struct giu_buff_ent *)malloc(outq_size * sizeof(struct giu_buff_ent));
	}
}

int app_giu_build_bpool(struct giu_bpool *bpool, u32 num_of_buffs)
{
	struct giu_bpool_capabilities capa;
	void *buff_virt_addr;
	void *buff_phys_addr;
	int i, err;

	giu_bpool_get_capabilities(bpool, &capa);

	if (capa.max_num_buffs < num_of_buffs)
		num_of_buffs = capa.max_num_buffs;

	pr_debug("Adding %d buffers (of %d Bytes) into BPOOL.\n",
		num_of_buffs, capa.buff_len);

	buff_virt_addr = mv_sys_dma_mem_alloc(capa.buff_len * num_of_buffs, 4);
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

		buff_inf.addr = (u64)buff_phys_addr + (i * capa.buff_len);
		buff_inf.cookie = (u64)buff_virt_addr + (i * capa.buff_len);

		err = giu_bpool_put_buff(bpool, &buff_inf);
		if (err)
			return err;
	}

	return 0;
}
