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

#include <linux/export.h>
#include "std_internal.h"
#include "drivers/mv_pp2.h"
#include "drivers/mv_pp2_ppio.h"

EXPORT_SYMBOL(pp2_ppio_init);
EXPORT_SYMBOL(pp2_init);
EXPORT_SYMBOL(mv_sys_dma_mem_destroy);
EXPORT_SYMBOL(pp2_get_num_inst);
EXPORT_SYMBOL(pp2_deinit);
EXPORT_SYMBOL(pp2_hif_init);
EXPORT_SYMBOL(__dma_virt_base);
EXPORT_SYMBOL(pp2_bpool_init);
EXPORT_SYMBOL(pp2_ppio_set_mru);
EXPORT_SYMBOL(pp2_bpool_put_buff);
EXPORT_SYMBOL(pp2_ppio_get_mtu);
EXPORT_SYMBOL(__dma_phys_base);
EXPORT_SYMBOL(mv_sys_dma_mem_alloc);
EXPORT_SYMBOL(pp2_ppio_set_mtu);
EXPORT_SYMBOL(pp2_netdev_get_ppio_info);
EXPORT_SYMBOL(pp2_ppio_enable);
EXPORT_SYMBOL(pp2_ppio_recv);
EXPORT_SYMBOL(pp2_ppio_get_num_outq_done);
EXPORT_SYMBOL(pp2_ppio_send);
EXPORT_SYMBOL(pp2_bpools);
EXPORT_SYMBOL(pp2_bpool_put_buffs);
