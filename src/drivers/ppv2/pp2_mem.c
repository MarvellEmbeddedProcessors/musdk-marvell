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

#include "std_internal.h"
#include "lib/uio_helper.h"

#include "pp2_types.h"
#include "pp2_mem.h"
#include "pp2.h"

/**
 * User I/O map API
 *
 */

int pp2_sys_io_exists(const char *name)
{
	struct uio_info_t *uio_info;

	if (!name)
		return 0;
	uio_info = uio_find_devices_byname(name);
	if (uio_info)
		return 1;
	return 0;
}

int pp2_sys_ioinit(pp2_maps_handle_t *pp2_maps_hdl, const char *name)
{
	struct pp2_uio **pp = (struct pp2_uio **)pp2_maps_hdl;

	*pp = kcalloc(1, sizeof(struct pp2_uio), GFP_KERNEL);
	if (!*pp)
		return -ENOMEM;

	(*pp)->uio_info = uio_find_devices_byname(name);
	if (!(*pp)->uio_info)
		return -ENODEV;

	struct uio_info_t *node;

	node = (*pp)->uio_info;
	while (node) {
		uio_get_all_info(node);
		node = node->next;
	}

	return 0;
}

static void add_mem_entry(struct uio_mem_t **headp, struct uio_mem_t *entry)
{
	entry->next = *headp;
	*headp = entry;
}

static struct uio_mem_t *remove_mem_entry(struct uio_mem_t **headp,
					  const char *name)
{
	struct uio_mem_t *entry = *headp;
	struct uio_mem_t *node = NULL;

	while (entry) {
		if (!strncmp(entry->info->maps[entry->map_num].name,
			     name, UIO_MAX_NAME_SIZE)) {
			*headp = entry->next;
			headp = &*headp;
			node = entry;
			return node;
		}

		headp = &entry->next;
		entry = entry->next;
	}
	return node;
}

uintptr_t pp2_sys_iomap(pp2_maps_handle_t pp2_maps_hdl, uint32_t *pa,
			const char *name)
{
	struct pp2_uio *pp = (struct pp2_uio *)pp2_maps_hdl;
	struct uio_mem_t *mem = NULL;
	uintptr_t va = (uintptr_t)NULL;

	mem = uio_find_mem_byname(pp->uio_info, name);
	if (!mem)
		return (uintptr_t)NULL;

	if (mem->fd < 0) {
		char dev_name[16];

		snprintf(dev_name, sizeof(dev_name),
			 "/dev/uio%d", mem->info->uio_num);
		mem->fd = open(dev_name, O_RDWR);
	}

	if (mem->fd >= 0) {
		va = (uintptr_t)uio_single_mmap(mem->info,
						mem->map_num, mem->fd);
		if (pa)
			*pa = mem->info->maps[mem->map_num].addr;
		add_mem_entry(&pp->mem, mem);
	} else {
		uio_free_mem_info(mem);
	}

	return va;
}

int pp2_sys_iounmap(pp2_maps_handle_t pp2_maps_hdl, const char *name)
{
	struct pp2_uio *pp = (struct pp2_uio *)pp2_maps_hdl;
	struct uio_mem_t *mem;

	mem = remove_mem_entry(&pp->mem, name);
	if (!mem)
		return -ENOENT;
	uio_single_munmap(mem->info, mem->map_num);
	/**
	 * TODO
	 * Handle device closing if no map registered as mapped. Change file
	 * descriptor to -1.
	 * If no memory is mapped I don't see any reason to keep the
	 * device opened.
	 *
	 */
	uio_free_mem_info(mem);

	return 0;
}

void pp2_sys_iodestroy(pp2_maps_handle_t pp2_maps_hdl)
{
	struct pp2_uio *pp = (struct pp2_uio *)pp2_maps_hdl;

	uio_free_info(pp->uio_info);
	kfree(pp);
}
