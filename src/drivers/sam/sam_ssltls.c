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

static inline void sam_hw_ring_ssltls_request_write(struct sam_hw_ring *hw_ring, int next_request,
				struct sam_sa *session, struct sam_cio_ssltls_params *request)
{
	u32 token_header_word, proto_word;
	struct sam_hw_cmd_desc *cmd_desc = sam_hw_cmd_desc_get(hw_ring, next_request);
	struct sam_hw_res_desc *res_desc = sam_hw_res_desc_get(hw_ring, next_request);

	/* Write prepared RDR descriptor first */
	sam_hw_rdr_prep_desc_write(res_desc, request->dst->paddr, request->dst->len);

	/* Write CDR descriptor */
	sam_hw_cdr_proto_cmd_desc_write(cmd_desc, request->src->paddr, request->pkt_size);

	/* Token is built inside the device */
	token_header_word = (request->pkt_size & SAM_TOKEN_PKT_LEN_MASK);
	token_header_word |= SAM_TOKEN_TYPE_AUTO_MASK;

	if (session->params.dir == SAM_DIR_DECRYPT) {
		token_header_word |= SAM_TOKEN_DTLS_INBOUND_MASK;

		if (session->params.u.ssltls.is_capwap)
			token_header_word |= SAM_TOKEN_DTLS_CAPWAP_MASK;
	} else
		token_header_word |= SAM_TOKEN_DTLS_CONTENT_SET(request->type);

	proto_word = SAM_CMD_TOKEN_OFFSET_SET(request->l3_offset);
	sam_hw_cdr_ext_token_write(cmd_desc, &request->sa->sa_buf, token_header_word,
					FIRMWARE_CMD_PKT_LDT_MASK, proto_word);
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

	iph = (struct iphdr *)(operation->out_frags[0].vaddr + l3_offset);
	ip_len = htobe16((u16)(result->out_len - l3_offset));

	/* Update IP total length field */
	iph->tot_len = ip_len;

	/* Set recalculated IP4 checksum */
	iph_len = iph->ihl * 4;
	udph = (struct udphdr *)((char *)iph + iph_len);
	udp_len = ip_len - htobe16(iph_len);
	udph->len = udp_len;
}

void sam_dtls_ip6_in_post_proc(struct sam_cio_op *operation, struct sam_hw_res_desc *res_desc,
			       struct sam_cio_op_result *result)
{
	pr_info("%s: Not supported yet\n", __func__);
}


int sam_cio_enq_ssltls(struct sam_cio *cio, struct sam_cio_ssltls_params *requests, u16 *num)
{
	struct sam_sa *session;
	struct sam_cio_op *operation;
	struct sam_cio_ssltls_params *request;
	int i, j, err, todo;

	todo = *num;
	if (todo >= cio->params.size)
		todo = cio->params.size - 1;

	for (i = 0; i < todo; i++) {
		request = &requests[i];

		/* Check request validity */
		err = sam_cio_check_ssltls_params(request);
		if (err)
			return err;

		/* Check maximum number of pending requests */
		if (sam_cio_is_full(cio)) {
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
		operation->num_bufs = request->num_bufs;
		for (j = 0;  j < request->num_bufs; j++) {
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
			sam_hw_ring_ssltls_request_write(&cio->hw_ring, cio->next_request,
							 session, request);
		} else {
			goto error_enq;
		}

#ifdef MVCONF_SAM_DEBUG
		if (sam_debug_flags & SAM_CIO_DEBUG_FLAG) {
			struct sam_hw_cmd_desc *cmd_desc = sam_hw_cmd_desc_get(&cio->hw_ring, cio->next_result);
			struct sam_hw_res_desc *res_desc = sam_hw_res_desc_get(&cio->hw_ring, cio->next_result);

			print_result_desc(res_desc, 1);
			print_cmd_desc(cmd_desc);

			printf("\nInput DMA buffer: %d bytes, physAddr = %p\n",
				operation->copy_len, (void *)request->src->paddr);
			mv_mem_dump(request->src->vaddr, operation->copy_len);
		}
#endif /* MVCONF_SAM_DEBUG */

		cio->next_request = sam_cio_next_idx(cio, cio->next_request);
		SAM_STATS(cio->stats.enq_bytes += operation->copy_len);
	}
	/* submit requests */
	if (i) {
		sam_hw_ring_submit(&cio->hw_ring, i);
		SAM_STATS(cio->stats.enq_pkts += i);
	}
	*num = (u16)i;

	return 0;

error_enq:
	return -EINVAL;
}

