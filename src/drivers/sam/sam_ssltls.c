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
#include "drivers/mv_sam.h"
#include "lib/net.h"
#include "lib/lib_misc.h"

#include "drivers/mv_sam_cio.h"
#include "sam.h"

u16 sam_ssltls_version_convert(enum sam_ssltls_version version)
{
	u16 ver;

	switch (version) {
	case SAM_SSL_VERSION_3_0:
		ver = SAB_SSL_VERSION_3_0;
		break;
	case SAM_TLS_VERSION_1_0:
		ver = SAB_TLS_VERSION_1_0;
		break;
	case SAM_TLS_VERSION_1_1:
		ver = SAB_TLS_VERSION_1_1;
		break;
	case SAM_TLS_VERSION_1_2:
		ver = SAB_TLS_VERSION_1_2;
		break;
	case SAM_DTLS_VERSION_1_0:
		ver = SAB_DTLS_VERSION_1_0;
		break;
	case SAM_DTLS_VERSION_1_2:
		ver = SAB_DTLS_VERSION_1_2;
		break;
	default:
		pr_err("Unexpected SSLTLS version %d\n", version);
		ver = 0;
	}
	return ver;
}

static int sam_cio_check_ssltls_params(struct sam_cio_ssltls_params *request)
{
	return 0;
}

static inline u32 sam_hw_ring_ssltls_cdr_desc_write(struct sam_hw_ring *hw_ring,
						     struct sam_sa *session,
						     struct sam_cio_ssltls_params *request)
{
	u32 thw;	/* token_header_word*/
	u32 proto_word, desc_num;
	int prep_data;
	u32 first_last_mask;
	int j;

	/* Prepared CDR descriptors */
	prep_data = 0;
	desc_num = 0;
	for (j = 0;  j < request->num_bufs; j++) {
		u32 data_size = request->src[j].len;

		first_last_mask = 0;
		/* write token for first descriptor only */
		if (j == 0) {
			struct sam_hw_cmd_desc *cmd_desc;

			first_last_mask |= SAM_DESC_FIRST_SEG_MASK;
			/* Token is built inside the device */
			cmd_desc = sam_hw_cmd_desc_get(hw_ring, hw_ring->next_cdr);
			thw = (request->pkt_size & SAM_TOKEN_PKT_LEN_MASK);
			thw |= SAM_TOKEN_TYPE_AUTO_MASK;

			if (session->params.dir == SAM_DIR_DECRYPT) {
				thw |= SAM_TOKEN_DTLS_INBOUND_MASK;

				if (session->params.u.ssltls.is_capwap)
					thw |= SAM_TOKEN_DTLS_CAPWAP_MASK;
			} else
				thw |= SAM_TOKEN_DTLS_CONTENT_SET(request->type);

			proto_word = SAM_CMD_TOKEN_OFFSET_SET(request->l3_offset);
			sam_hw_cdr_ext_token_write(cmd_desc,
						   &request->sa->sa_buf,
						   thw,
						   FIRMWARE_CMD_PKT_LDT_MASK,
						   proto_word);
		}

		if (j == (request->num_bufs - 1)) {
			first_last_mask |= SAM_DESC_LAST_SEG_MASK;
			data_size = request->pkt_size - prep_data;
			if (prep_data >= request->pkt_size) {
				pr_err("%s: crypto data size %d > packet size %d\n",
						__func__, prep_data, request->pkt_size);
				sam_hw_ring_roolback(hw_ring, 1, j);
				desc_num = 0;
				break;
			}
		} else
			prep_data += data_size;

		/* Write CDR descriptor */
		sam_hw_cdr_proto_cmd_desc_write(hw_ring, request->src[j].paddr,
						data_size, first_last_mask);
#ifdef MVCONF_SAM_DEBUG
		if (sam_debug_flags & SAM_CIO_DEBUG_FLAG) {
			printf("\nInput DMA buffer: %d bytes, physAddr = %p\n",
				request->pkt_size, (void *)request->src[j].paddr);
			mv_mem_dump(request->src[j].vaddr, request->pkt_size);
		}
#endif /* MVCONF_SAM_DEBUG */

		desc_num++;
	}
	return desc_num;
}

/* Update "total length" and "csum" field in IP header and "length" field in UDP header
 */
void sam_dtls_ip4_post_proc(struct sam_cio_op *operation, struct sam_hw_res_desc *res_desc,
			       struct sam_cio_op_result *result)
{
	u32 val32;
	u8 iph_len, l3_offset;
	struct iphdr *iph;
	struct udphdr *udph;
	u16 ip_len, udp_len;

	val32 = readl_relaxed(&res_desc->words[7]);
	l3_offset = SAM_RES_TOKEN_OFFSET_GET(val32);

	/* Update IP total length field */
	iph = (struct iphdr *)(operation->out_frags[0].vaddr + l3_offset);
	ip_len = (u16)(result->out_len - l3_offset);
	iph->tot_len = htobe16(ip_len);

	/* Update UDP length field */
	iph_len = iph->ihl * 4;
	udph = (struct udphdr *)((char *)iph + iph_len);
	udp_len = ip_len - iph_len;
	udph->len = htobe16(udp_len);

}

void sam_dtls_ip6_post_proc(struct sam_cio_op *operation, struct sam_hw_res_desc *res_desc,
			       struct sam_cio_op_result *result)
{
	u32 val32;
	u8 ip6h_len, l3_offset;
	struct ip6_hdr *ip6h;
	struct udphdr *udph;
	u16 ip_len, udp_len;

	val32 = readl_relaxed(&res_desc->words[7]);
	l3_offset = SAM_RES_TOKEN_OFFSET_GET(val32);

	/* Update IPv6 payload length field */
	ip6h = (struct ip6_hdr *)(operation->out_frags[0].vaddr + l3_offset);
	ip6h_len = sizeof(struct ip6_hdr);
	ip_len = (u16)(result->out_len - l3_offset - ip6h_len);
	ip6h->ip6_plen = htobe16(ip_len);

	/* Update UDP length field - no extension headers support */
	udph = (struct udphdr *)((char *)ip6h + ip6h_len);
	udp_len = ip_len;
	udph->len = htobe16(udp_len);

}


int sam_cio_enq_ssltls(struct sam_cio *cio, struct sam_cio_ssltls_params *requests, u16 *num)
{
	struct sam_sa *session;
	struct sam_cio_op *operation;
	struct sam_cio_ssltls_params *request;
	int i, j, err, todo;
	u32 rdr_submit = 0;
	u32 cdr_submit = 0;

	todo = *num;
	if (todo >= cio->params.size)
		todo = cio->params.size - 1;

	for (i = 0; i < todo; i++) {
		request = &requests[i];

		/* Check request validity */
		err = sam_cio_check_ssltls_params(request);
		if (err)
			return err;

		/* Look if there are enough free resources */
		if (!sam_cio_is_free_slot(cio, request->num_bufs)) {
			SAM_STATS(cio->stats.enq_full++);
			break;
		}
#ifdef MVCONF_SAM_DEBUG
		if (sam_debug_flags & SAM_CIO_DEBUG_FLAG)
			print_sam_cio_ssltls_params(request);
#endif /* MVCONF_SAM_DEBUG */

		session = request->sa;

		/* Get next operation structure */
		operation = &cio->operations[cio->next_request];

		/* Save some fields from request needed for result processing */
		operation->sa = request->sa;
		operation->cookie = request->cookie;
		operation->copy_len = request->pkt_size;
		operation->num_bufs_in = request->num_bufs;
		/* only one destination buffer is supported */
		operation->num_bufs_out = 1;
		for (j = 0;  j < operation->num_bufs_out; j++) {
			operation->out_frags[j].vaddr = request->dst[j].vaddr;
			operation->out_frags[j].paddr = request->dst[j].paddr;
			operation->out_frags[j].len = request->dst[j].len;
		}

		if (session->cio != cio) {
#ifdef MVCONF_SAM_DEBUG
			if (session->cio != NULL) {
				pr_warn("Session is moved from cio=%d:%d to cio=%d:%d\n",
					session->cio->hw_ring.device, session->cio->hw_ring.ring,
					cio->hw_ring.device, cio->hw_ring.ring);
			}
#endif
			session->cio = cio;
		}

		if (cio->hw_ring.type == HW_EIP197) {
			u32 seg_mask;

			/* Write prepared RDR descriptor */
			seg_mask = SAM_DESC_FIRST_SEG_MASK | SAM_DESC_LAST_SEG_MASK;
			sam_hw_rdr_desc_write(&cio->hw_ring, request->dst->paddr,
					      request->dst->len, seg_mask);
			rdr_submit++;

			/* write CDR descriptors and token */
			cdr_submit += sam_hw_ring_ssltls_cdr_desc_write(&cio->hw_ring,
									session,
									request);
		} else {
			goto error_enq;
		}

		cio->next_request = sam_cio_next_idx(cio, cio->next_request);
		SAM_STATS(cio->stats.enq_bytes += operation->copy_len);
	}
	/* submit requests */
	if (likely(i)) {
		sam_hw_rdr_ring_submit(&cio->hw_ring, rdr_submit);
		sam_hw_cdr_ring_submit(&cio->hw_ring, cdr_submit);
		SAM_STATS(cio->stats.enq_pkts += i);
	}
	*num = (u16)i;

	return 0;

error_enq:
	return -EINVAL;
}

