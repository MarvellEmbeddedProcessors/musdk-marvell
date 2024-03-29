/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _GIU_BUFF_DESC_H_
#define _GIU_BUFF_DESC_H_

#include "std_internal.h"

/* This file includes the definitions of all data structures that should be
 * shared with the various drivers and modules on the SNIC side.
 * The file should be copied AS-IS, in order to make sure that data-structures
 * on both sides are 100% aligned.
 */

#define HOST_FORMAT_DIRECT_SG	2
#define HOST_NUM_SG_ENT_MASK	0x1F


#define HOST_TXD_FORMAT_MASK	(0x30000000)
#define HOST_TXD_FORMAT_SHIFT	28


struct host_tx_desc {
	/* 0x0 - 0x3
	 * fields order: msb ... lsb
	 * Byte 0: res:1 | l3_offset:7
	 * Byte 1: gen_ip4_csum:1 | gen_l4_csum:2 | ip_hdr_len:5
	 * Byte 2: l2_pad_en:1 | res2:1 | buf_release_mode:1 | pool_idx:5
	 * Byte 3: res3:2 | last:1 | first:1 | l3_info:2 | l4_type:2
	 */
	u32 flags;

	/* 0x4 - 0x7 */
	u8 pkt_offset;
	u8 res4;
	u16 byte_cnt;

	/* 0x8 - 0xB */
	u16 res5;
	u8 num_sg_ent;
	u8 res;

	/* 0xC - 0xF */
	u32 res6;

	/* 0x10 - 0x17 */
	u64 buffer_addr;

	/* 0x18 - 0x1F */
	u64 cookie;
}  __packed;

#define HOST_RXD_FORMAT_MASK	(0x00180000)
#define HOST_RXD_FORMAT_SHIFT	19

struct host_rx_desc {
	/* 0x0 - 0x3
	 * fields order: msb ... lsb
	 * Byte 0: res:1 | l3offset:7
	 * Byte 1: err_sum:1 | err_code:2 | ip_hdr_len:5
	 * Byte 2: res:1 | l4_csum_ok:1 | res:1 | pool_idx:5
	 * Byte 3: res:1 | l3_info:3 | l4_info:3 | ipv4_hdr_err:1
	 */
	u32 flags;

	/* 0x4 - 0x7 */
	u8 pkt_offset;
	/* portnum_dp: port_num:3 | dp:2 | res:3 */
	u8 portnum_dp;
	u16 byte_cnt;

	/* 0x8 - 0xB */
	u16 res4;
	u8 num_sg_ent; /* currently 5bits, 0x1f */
	u8 res;

	/* 0xC - 0xF */
	u32 timestamp_hashkey;

	/* 0x10 - 0x17 */
	u64 buffer_addr;

	/* 0x18 - 0x1F */
	u64 cookie;
} __packed;

/* Buffers Pool Descriptor */
struct host_bpool_desc {
	u64 buff_addr_phys;
	u64 buff_cookie;
} __packed;

#endif /* _GIU_BUFF_DESC_H_ */
