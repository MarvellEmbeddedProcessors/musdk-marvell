/* Copyright (c) 2018 Marvell.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef _MV_DP_EXT_HDR_H_
#define _MV_DP_EXT_HDR_H_


#include <linux/kernel.h>
#include <linux/types.h>


#ifdef __cplusplus
extern "C" {
#endif

/*input: input is in host order done during msg build
u32 * mv_dp_ext_hdr_to_be(u32 *hdr);
*/

/*input: input is in network order, out:host and virtual addr*/
u32 *mv_dp_ext_hdr_to_host(u32 *hdr);

/*input: input in network order, phy addr*/
enum mv_dp_rc mv_dp_ext_hdr_be_release(u32 *hdr);

/*input: input in host order and virt addr*/
enum mv_dp_rc mv_dp_ext_hdr_host_release(u32 *hdr);

/*input: input is in host order*/
void mv_dp_ext_hdr_host_show(const u32 *const hdr, int print_count);
/*input is in be and phy ptr*/
void mv_dp_ext_hdr_be_show(const u32 * const hdr, int print_count, bool inv);


#ifdef __cplusplus
}
#endif


#endif
