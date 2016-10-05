/**************************************************************************//**
	Copyright (C) 2016 Marvell International Ltd.
*//***************************************************************************/

#include "mv_std.h"

#include "mv_pp2_ppio.h"


struct pp2_ppio {
	int dummy;
};


int pp2_ppio_init(struct pp2_ppio_params *params, struct pp2_ppio **ppio)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUPP;
}


int pp2_ppio_enable(struct pp2_ppio *ppio);
int pp2_ppio_disable(struct pp2_ppio *ppio);

int pp2_ppio_set_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr);
int pp2_ppio_get_mac_addr(struct pp2_ppio *ppio, eth_addr_t addr);
int pp2_ppio_set_mtu(struct pp2_ppio *ppio, u16 mtu);
int pp2_ppio_get_mtu(struct pp2_ppio *ppio, u16 *mtu);
int pp2_ppio_set_mru(struct pp2_ppio *ppio, u16 len);
int pp2_ppio_get_mru(struct pp2_ppio *ppio, u16 *len);
int pp2_ppio_set_uc_promisc(struct pp2_ppio *ppio, int en);
int pp2_ppio_get_uc_promisc(struct pp2_ppio *ppio, int *en);
int pp2_ppio_set_mc_promisc(struct pp2_ppio *ppio, int en);
int pp2_ppio_get_mc_promisc(struct pp2_ppio *ppio, int *en);
int pp2_ppio_add_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr);
int pp2_ppio_remove_mac_addr(struct pp2_ppio *ppio, const eth_addr_t addr);
int pp2_ppio_flush_mac_addrs(struct pp2_ppio *ppio, int uc, int mc);


int pp2_ppio_send(struct pp2_ppio	*ppio,
		  struct pp2_hif	*hif,
		  u8			 qid,
		  struct pp2_ppio_desc	*descs,
		  u16			*num)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUPP;
}

int pp2_ppio_send_sg(struct pp2_ppio		*ppio,
		     struct pp2_hif		*hif,
		     u8			 qid,
		     struct pp2_ppio_sg_desc	*descs,
		     u16			*num)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUPP;
}

int pp2_ppio_get_num_outq_done(struct pp2_ppio	*ppio,
			       struct pp2_hif	*hif,
			       u8		 qid,
			       u16		*num)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUPP;
}

int pp2_ppio_recv(struct pp2_ppio	*ppio,
		  u8			 tc,
		  u8			 qid,
		  struct pp2_ppio_desc	*descs,
		  u16			*num)
{
	pr_err("[%s] routine not supported yet!\n", __FUNCTION__);
	return -ENOTSUPP;
}
