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


#include "std_internal.h"

#include "pp2_types.h"
#include "pp2_hif.h"
#include "pp2.h"
#include "pp2_port.h"
#include "pp2_bpool.h"
#include "lib/lib_misc.h"



struct pp2_ppio {
	struct pp2_port *port;
};

static struct pp2_ppio ppio_array[PP2_MAX_NUM_PACKPROCS][PP2_NUM_ETH_PPIO];


static inline struct pp2_dm_if * pp2_dm_if_get(struct pp2_ppio *ppio, struct pp2_hif *hif)
{
	return(ppio->port->parent->dm_ifs[hif->regspace_slot]);
}

int pp2_ppio_init(struct pp2_ppio_params *params, struct pp2_ppio **ppio)
{
	u8  match[2];
	int port_id, pp2_id, rc;

	if (mv_sys_match(params->match, "ppio", 2, match)) {
		pr_err("[%s] Invalid match string!\n", __FUNCTION__);
		return(-ENXIO);
	}

	if (pp2_is_init() == false)
		return(-EPERM);

	pp2_id = match[0];
	port_id = match[1];

	if (port_id >= PP2_NUM_ETH_PPIO) {
		pr_err("[%s] Invalid ppio.\n", __FUNCTION__);
		return(-ENXIO);
	}
	if (pp2_id >= pp2_ptr->num_pp2_inst) {
		pr_err("[%s] Invalid pp2 instance.\n", __FUNCTION__);
		return(-ENXIO);
	}
	if (pp2_ptr->init.ppios[pp2_id][port_id].is_enabled == 0) {
		pr_err("[%s] ppio is reserved.\n", __FUNCTION__);
		return(-EFAULT);
	}
	if (ppio_array[pp2_id][port_id].port != NULL) {
		pr_err("[%s] ppio already exists.\n", __FUNCTION__);
		return(-EEXIST);
	}

	rc = pp2_port_open(pp2_ptr, params, pp2_id, port_id, &(ppio_array[pp2_id][port_id].port));
	if (!rc) {
		*ppio = &ppio_array[pp2_id][port_id];
	}

	pp2_port_config_inq((*ppio)->port);
	pp2_port_config_outq((*ppio)->port);

	return rc;
}

void pp2_ppio_deinit(struct pp2_ppio *ppio)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
}


int pp2_ppio_enable(struct pp2_ppio *ppio)
{
	pp2_port_start(ppio->port, PP2_TRAFFIC_INGRESS_EGRESS);
	return (0);
}

int pp2_ppio_disable(struct pp2_ppio *ppio)
{
	pp2_port_stop(ppio->port);
	return (0);
}

/* Note: Function cannot be inlined, because of reference to pool->id */
void pp2_ppio_outq_desc_set_pool(struct pp2_ppio_desc *desc, struct pp2_bpool *pool)
{
	desc->cmds[0] = (desc->cmds[0] & ~(TXD_POOL_ID_MASK | TXD_BUFMODE_MASK)) |
		(pool->id << 16 & TXD_POOL_ID_MASK) | (1 << 7 & TXD_BUFMODE_MASK);
}

int pp2_ppio_send(struct pp2_ppio *ppio, struct pp2_hif *hif, u8 qid, struct pp2_ppio_desc *descs, u16 *num)
{
	struct pp2_dm_if *dm_if;
	u16 desc_sent, desc_req = *num;

	dm_if = pp2_dm_if_get(ppio, hif);

	desc_sent = pp2_port_enqueue(ppio->port, dm_if, qid, desc_req, descs);
	if (unlikely(desc_sent < desc_req)) {
		pr_debug("[%s] pp2_id %u Port %u qid %u, send_request %u sent %u!\n", __FUNCTION__,
			 ppio->port->parent->id, ppio->port->id, qid, *num, desc_sent);
		*num = desc_sent;
	}
	return(0);
}

int pp2_ppio_send_sg(struct pp2_ppio *ppio,
		     struct pp2_hif *hif,
		     u8  qid,
		     struct pp2_ppio_sg_desc *descs,
		     u16 *num)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUP;
}

int pp2_ppio_get_num_outq_done(struct pp2_ppio *ppio,
			       struct pp2_hif *hif,
			       u8 qid,
			       u16 *num)
{
	struct pp2_dm_if *dm_if;
	u32 outq_physid;


	dm_if = pp2_dm_if_get(ppio, hif);
	outq_physid = ppio->port->txqs[qid]->id;
	*num = pp2_port_outq_status(dm_if, outq_physid);

	return 0;
}

/* ALL TX Setter functions, and RX Getter functions are u32 based */
static inline void pp2_ppio_desc_swap_ncopy(struct pp2_ppio_desc *dst, struct pp2_ppio_desc *src)
{
	u32 *src_cmd = (u32 *)src;
	u32 *dst_cmd = (u32 *)dst;

	for (int i = 0; i < PP2_PPIO_DESC_NUM_WORDS; i++) {
		*dst_cmd = le32toh(*src_cmd);
		dst_cmd++;
		src_cmd++;
	}
}

int pp2_ppio_recv(struct pp2_ppio *ppio, u8 tc, u8 qid, struct pp2_ppio_desc *descs, u16 *num)
{
	struct pp2_port *port = ppio->port;
	struct pp2_desc *rx_desc, *extra_rx_desc;
	struct pp2_rx_queue * rxq;
	u32 recv_req = *num, extra_num = 0;
	int log_rxq, rxq_id, rc = 0;

	/* TODO: After validation, delete recv_req variable */
	log_rxq = port->tc[tc].first_log_rxq + qid;
	rxq_id = port->first_rxq + log_rxq;
	rxq = port->rxqs[log_rxq];

	if (recv_req > rxq->desc_received) {
		rxq->desc_received = pp2_rxq_received(port, rxq_id);
		if (unlikely(recv_req > rxq->desc_received)) {
			recv_req = rxq->desc_received;
			*num = recv_req;
		}
	}

	/* TODO : Make pp2_rxq_get_desc inline */
	rx_desc = pp2_rxq_get_desc(rxq, &recv_req, &extra_rx_desc, &extra_num);
#if __BYTE_ORDER == __BIG_ENDIAN
	for (int i = 0; i < recv_req; i++) {
		pp2_ppio_desc_swap_ncopy(&descs[i], &rx_desc[i]);
	}
#else
	memcpy(descs, rx_desc, recv_req * sizeof(*descs));
#endif
	if (extra_num) {
		memcpy(&descs[recv_req], extra_rx_desc, extra_num * sizeof(*descs));
		recv_req += extra_num; /* Put the split numbers back together */
	}
	/*  Update HW */
	pp2_port_inq_update(port, log_rxq, recv_req, recv_req);
	rxq->desc_received -= recv_req;

	return rc;
}

int pp2_ppio_set_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr)
{
	int rc;

	rc = pp2_port_set_mac_addr(ppio->port, (const uint8_t *) addr);
	return rc;
}
int pp2_ppio_get_mac_addr(struct pp2_ppio *ppio, eth_addr_t addr)
{
	int rc;

	rc = pp2_port_get_mac_addr(ppio->port, (uint8_t *)addr);
	return rc;
}

int pp2_ppio_set_mtu(struct pp2_ppio *ppio, u16 mtu)
{
	int rc;

	rc = pp2_port_set_mtu(ppio->port, mtu);
	return rc;
}

int pp2_ppio_get_mtu(struct pp2_ppio *ppio, u16 *mtu)
{
	pp2_port_get_mtu(ppio->port, mtu);
	return 0;
}

int pp2_ppio_set_mru(struct pp2_ppio *ppio, u16 len)
{
	int rc;
	rc = pp2_port_set_mru(ppio->port, len);
	return rc;
}

int pp2_ppio_get_mru(struct pp2_ppio *ppio, u16 *len)
{
	pp2_port_get_mru(ppio->port, len);

	return 0;
}

int pp2_ppio_set_uc_promisc(struct pp2_ppio *ppio, int en)
{
	int rc;

	rc = pp2_port_set_uc_promisc(ppio->port, en);
	return rc;
}

int pp2_ppio_get_uc_promisc(struct pp2_ppio *ppio, int *en)
{
	int rc;

	rc = pp2_port_get_uc_promisc(ppio->port, (u32 *)en);
	return rc;
}

int pp2_ppio_set_mc_promisc(struct pp2_ppio *ppio, int en)
{
	int rc;

	rc = pp2_port_set_mc_promisc(ppio->port, en);
	return rc;
}

int pp2_ppio_get_mc_promisc(struct pp2_ppio *ppio, int *en)
{
	int rc;

	rc = pp2_port_get_mc_promisc(ppio->port, (u32 *)en);
	return rc;
}

int pp2_ppio_add_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr)
{
	int rc;

	rc = pp2_port_add_mac_addr(ppio->port, (const uint8_t *)addr);
	return rc;
}

int pp2_ppio_remove_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr)
{
	int rc;

	rc = pp2_port_remove_mac_addr(ppio->port, (const uint8_t *)addr);
	return rc;
}

int pp2_ppio_flush_mac_addrs(struct pp2_ppio *ppio, int uc, int mc)
{
	int rc;

	rc = pp2_port_flush_mac_addrs(ppio->port, uc, mc);
	return rc;
}

int pp2_ppio_add_vlan(struct pp2_ppio *ppio, u16 vlan)
{
	int rc;

	rc = pp2_port_add_vlan(ppio->port, vlan);
	return rc;
}

int pp2_ppio_remove_vlan(struct pp2_ppio *ppio, u16 vlan)
{
	int rc;

	rc = pp2_port_remove_vlan(ppio->port, vlan);
	return rc;
}

int pp2_ppio_flush_vlan(struct pp2_ppio *ppio)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUP;
}

