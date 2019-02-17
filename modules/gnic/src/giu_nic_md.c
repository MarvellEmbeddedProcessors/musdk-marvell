/*
* ***************************************************************************
* Copyright (C) 2018 Marvell International Ltd.
* ***************************************************************************
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
* ***************************************************************************
*/

#include <linux/etherdevice.h>
#include "mv_gnic_nss.h"
#include "giu_nic_md.h"

#define META_DATA_DEFAULT_VALUE  0xdeadbeafdeadbeaf
#define MAX_MD_TBL_SIZE          20

struct agnic_md_info {
	u64 tx_meta_data[2];
	u64 rx_meta_data[2];
	bool rx_metadata_store;
	bool tx_metadata_classify;

	struct md_tbl_entry md_tbl[MAX_MD_TBL_SIZE];
	int    md_tbl_size;
};

static struct agnic_md_info md_info = {
	.tx_meta_data[0] = META_DATA_DEFAULT_VALUE,
	.tx_meta_data[1] = META_DATA_DEFAULT_VALUE,
};

void agnic_metadata_receive_hdl(struct sk_buff *skb)
{
	char *md;

	if (!md_info.rx_metadata_store)
		return;

	if (!skb_mac_header_was_set(skb)) {
		pr_warn("no mac header was set\n");
		return;
	}

	/* make sure we have enough room to refer the md location */
	if (skb_headroom(skb) < MV_NSS_METADATA_LEN + ETH_HLEN) {
		pr_warn("not enough head room to refer md\n");
		return;
	}

	/* save the received meta data into the nss info global       */
	/* at this point, skb->data points after the ethernet header  */
	md = skb_mac_header(skb) - MV_NSS_METADATA_LEN;
	memcpy(md_info.rx_meta_data, md, MV_NSS_METADATA_LEN);
}
EXPORT_SYMBOL(agnic_metadata_receive_hdl);

static void *get_metadata_classify(struct sk_buff *skb)
{
	int i;
	struct ethhdr *ehdr = (struct ethhdr *)eth_hdr(skb);

	for (i = 0 ; i < md_info.md_tbl_size ; i++) {
		if (ether_addr_equal_unaligned(md_info.md_tbl[i].eth_addr, ehdr->h_dest))
			return md_info.md_tbl[i].md.meta_data;
	}

	return md_info.tx_meta_data;
}

static void *get_metadata_skb(struct sk_buff *skb)
{
	void *meta_data = get_metadata_classify(skb);

	return meta_data;
}

void agnic_metadata_xmit_hdl(struct sk_buff *skb)
{
	void *md_src, *md_dst;

	/* push skb data to point on metadata */
	md_dst = skb_push(skb, MV_NSS_METADATA_LEN);
	if (!md_info.tx_metadata_classify)
		return;

	md_src = get_metadata_skb(skb);
	memcpy(md_dst, md_src, MV_NSS_METADATA_LEN);
}
EXPORT_SYMBOL(agnic_metadata_xmit_hdl);

void agnic_set_tx_meta_data(struct meta_data_info *md)
{
	md_info.tx_meta_data[0] = md->meta_data[0];
	md_info.tx_meta_data[1] = md->meta_data[1];
}

void agnic_get_tx_meta_data(struct meta_data_info *md)
{
	md->meta_data[0] = md_info.tx_meta_data[0];
	md->meta_data[1] = md_info.tx_meta_data[1];
}

void agnic_get_rx_meta_data(struct meta_data_info *md)
{
	md->meta_data[0] = md_info.rx_meta_data[0];
	md->meta_data[1] = md_info.rx_meta_data[1];
}

void agnic_set_rx_meta_data_store(bool fl)
{
	md_info.rx_metadata_store = fl;
}

bool agnic_get_rx_meta_data_store(void)
{
	return md_info.rx_metadata_store;
}

bool agnic_get_tx_meta_data_classify(void)
{
	return md_info.tx_metadata_classify;
}

void agnic_set_tx_meta_data_classify(bool fl)
{
	md_info.tx_metadata_classify = fl;
}

int agnic_add_metadata_tbl_entry(struct md_tbl_entry *entry)
{
	if (md_info.md_tbl_size == MAX_MD_TBL_SIZE)
		return false;

	ether_addr_copy(md_info.md_tbl[md_info.md_tbl_size].eth_addr, entry->eth_addr);
	memcpy(&md_info.md_tbl[md_info.md_tbl_size].md.meta_data, entry->md.meta_data, sizeof(md_info.md_tbl[0].md));
	md_info.md_tbl_size++;

	return true;
}

int agnic_get_metadata_tbl_size(void)
{
	return md_info.md_tbl_size;
}

struct md_tbl_entry *agnic_get_metadata_tbl_entry(int entry_idx)
{
	return &md_info.md_tbl[entry_idx];
}

