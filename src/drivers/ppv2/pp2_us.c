/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
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

/**
 * @file pp2_us.c
 *
 * PPDK container structures and packet processor initialization
 */

#include "std_internal.h"

#include "pp2_types.h"
#include "../../modules/include/mv_pp_uio.h"

#include "pp2.h"
#include "pp2_dm.h"
#include "pp2_port.h"
#include "pp2_bm.h"
#include "pp2_gop_dbg.h"
#include "pp2_hw_cls.h"
#include "cls/pp2_cls_mng.h"

struct pp2 *pp2_ptr;
struct netdev_if_params *netdev_params;

int pp2_init(struct pp2_init_params *params)
{
	u32 pp2_id, pp2_num_inst;
	struct pp2_ppio_params lb_port_params;
	int rc;

	pp2_ptr = kcalloc(1, sizeof(struct pp2), GFP_KERNEL);
	if (unlikely(!pp2_ptr)) {
		pr_err("%s out of memory pp2 alloc\n", __func__);
		return -ENOMEM;
	}
	memcpy(&pp2_ptr->init, params, sizeof(*params));
	pp2_ptr->pp2_common.hif_slot_map = 0;
	pp2_ptr->pp2_common.rss_tbl_map = params->rss_tbl_reserved_map;
	/* TODO: Check first_inq params are valid */

	/* Retrieve netdev if information */
	pp2_num_inst = pp2_get_num_inst();
	netdev_params = kmalloc(sizeof(*netdev_params) * pp2_num_inst * PP2_NUM_ETH_PPIO, GFP_KERNEL);
	if (!netdev_params)
		return -ENOMEM;

	memset(netdev_params, 0, sizeof(*netdev_params) * pp2_num_inst * PP2_NUM_ETH_PPIO);
	pp2_netdev_if_info_get(netdev_params);

	/* Initialize in an opaque manner from client,
	* depending on HW, one or two packet processors.
	*/
	for (pp2_id = 0; pp2_id < pp2_num_inst; pp2_id++) {
		struct pp2_inst *inst;

		inst = pp2_inst_create(pp2_ptr, pp2_id);
		if (!inst) {
			pr_err("cannot create PP%u\n", pp2_id);

			if (pp2_id == PP2_ID1) {
				/* Also destroy the previous instance */
				pp2_destroy(pp2_ptr->pp2_inst[PP2_ID0]);
			}
			kfree(pp2_ptr);

			return -ENOMEM;
		}
		/* Store the PPDK handle as parent and store
		 * this instance as child handle for the PPDK
		 */
		pp2_ptr->pp2_inst[pp2_id] = inst;

		/* Initialize this packet processor */
		pp2_inst_init(inst);
		pp2_ptr->num_pp2_inst++;
	}

	pr_debug("PackProcs   %2u\n", pp2_num_inst);
	lb_port_params.type = PP2_PPIO_T_NIC;
	lb_port_params.inqs_params.num_tcs = 0;
	lb_port_params.outqs_params.num_outqs = PP2_LPBK_PORT_NUM_TXQ;
	lb_port_params.outqs_params.outqs_params[0].size = PP2_LPBK_PORT_TXQ_SIZE;

	/* Initialize the loopback port */
	for (pp2_id = 0; pp2_id < pp2_num_inst; pp2_id++) {
		struct pp2_port *port;

		rc = pp2_port_open(pp2_ptr, &lb_port_params, pp2_id, PP2_LOOPBACK_PORT, &port);
		if (rc) {
			pr_err("[%s] ppio init failed.\n", __func__);
			return(-EFAULT); /* TODO: pp2_destroy on error (currently error_ret not possible) */
		}
		pp2_port_config_outq(port);
		pp2_port_start(port, PP2_TRAFFIC_EGRESS);
	}

	return 0;
}
