/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */
#ifndef _ARMADA_GIU_NIC_MD_H_
#define _ARMADA_GIU_NIC_MD_H_

#include <linux/if_ether.h>

struct meta_data_info {
	u64 meta_data[2];
};

struct md_tbl_entry {
	u8  eth_addr[ETH_ALEN];
	struct meta_data_info md;
};

void  agnic_set_tx_meta_data(struct meta_data_info *md);
void  agnic_get_tx_meta_data(struct meta_data_info *md);
void  agnic_get_rx_meta_data(struct meta_data_info *md);
void  agnic_set_rx_meta_data_store(bool fl);
bool  agnic_get_rx_meta_data_store(void);
bool  agnic_get_tx_meta_data_classify(void);
void  agnic_set_tx_meta_data_classify(bool fl);

int     agnic_get_metadata_tbl_size(void);
int     agnic_add_metadata_tbl_entry(struct md_tbl_entry *entry);
struct  md_tbl_entry *agnic_get_metadata_tbl_entry(int entry_idx);

void agnic_metadata_receive_hdl(struct sk_buff *skb);
void agnic_metadata_xmit_hdl(struct sk_buff *skb);

#endif

