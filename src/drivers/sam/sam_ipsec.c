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

struct sam_hw_ipsec_result {
	u8 next_header;
	u8 l3_offset;
	u8 pad_len;
	u8 reserved;
};

static int sam_cio_check_ipsec_params(struct sam_cio_ipsec_params *request)
{
	return 0;
}

static inline void sam_hw_ring_ipsec_request_set(struct sam_hw_ring *hw_ring, int next_request,
						 struct sam_sa *session, struct sam_cio_ipsec_params *request,
						 int reuse)
{
	u32 proto_word, token_header_word;
	struct sam_hw_cmd_desc *cmd_desc = sam_hw_cmd_desc_get(hw_ring, next_request);
	struct sam_hw_res_desc *res_desc = sam_hw_res_desc_get(hw_ring, next_request);

	/* Write prepared RDR descriptor first */
	sam_hw_rdr_prep_desc_write(res_desc, request->dst->paddr, request->dst->len,
				   (SAM_DESC_FIRST_SEG_MASK | SAM_DESC_LAST_SEG_MASK));

	/* Write CDR descriptor */
	sam_hw_cdr_proto_cmd_desc_write(cmd_desc, request->src->paddr, request->pkt_size);

	/* Token is built inside the device */
	token_header_word = (request->pkt_size & SAM_TOKEN_PKT_LEN_MASK);
	token_header_word |= SAM_TOKEN_TYPE_AUTO_MASK;
	if (reuse)
		token_header_word |= SAM_TOKEN_REUSE_AUTO_MASK;

	proto_word = SAM_CMD_TOKEN_OFFSET_SET(request->l3_offset);
	if (session->params.dir == SAM_DIR_ENCRYPT)
		proto_word |= SAM_CMD_TOKEN_NEXT_HDR_SET(IPPROTO_ESP);
	else
		proto_word |= SAM_CMD_TOKEN_NEXT_HDR_SET(0);

	sam_hw_cdr_ext_token_write(cmd_desc, &request->sa->sa_buf, token_header_word,
					FIRMWARE_CMD_PKT_LIP_MASK, proto_word);
}

void sam_ipsec_prepare_tunnel_header(struct sam_session_params *params, u8 *tunnel_header)
{
	if (!params->u.ipsec.is_tunnel)
		return;

	if (params->u.ipsec.is_ip6) {
		/* TBD */
	} else {
		struct iphdr *iph = (struct iphdr *)tunnel_header;

		memset(iph, 0, sizeof(struct iphdr));
		iph->ihl = 5; /* 20 Bytes */
		iph->version = 4;

		if (params->u.ipsec.tunnel.u.ipv4.df)
			iph->frag_off = IP_DF;

		iph->ttl = params->u.ipsec.tunnel.u.ipv4.ttl;
		iph->protocol = IPPROTO_ESP;

		if (params->u.ipsec.tunnel.u.ipv4.sip)
			memcpy(&iph->saddr, params->u.ipsec.tunnel.u.ipv4.sip, 4);

		if (params->u.ipsec.tunnel.u.ipv4.dip)
			memcpy(&iph->daddr, params->u.ipsec.tunnel.u.ipv4.dip, 4);
	}
}

void sam_ipsec_ip4_transport_in_post_proc(struct sam_cio_op *operation, struct sam_hw_res_desc *res_desc,
					  struct sam_cio_op_result *result)
{
	u32 val32;
	u8 next_hdr, l3_offset;
	struct iphdr *iph;
	u16 csum, ip_len, *ptr16;

	val32 = readl_relaxed(&res_desc->words[7]);
	next_hdr = SAM_RES_TOKEN_NEXT_HDR_GET(val32);
	l3_offset = SAM_RES_TOKEN_OFFSET_GET(val32);

	iph = (struct iphdr *)(operation->out_frags[0].vaddr + l3_offset);
	ip_len = htobe16((u16)(result->out_len - l3_offset));

	/* Read old checksum */
	csum = ~(iph->check);

	/* Update IP total length field */
	csum = mv_sub_csum16(csum, iph->tot_len);
	csum = mv_add_csum16(csum, ip_len);
	iph->tot_len = ip_len;

	/* read 16 bits TTL + Protocol */
	ptr16 = (u16 *)&iph->ttl;
	csum = mv_sub_csum16(csum, *ptr16);

	/* Set protocol field */
	iph->protocol = next_hdr;
	csum = mv_add_csum16(csum, *ptr16);

	/* Set recalculated IP4 checksum */
	iph->check = ~csum;
}

void sam_ipsec_ip6_transport_in_post_proc(struct sam_cio_op *operation, struct sam_hw_res_desc *res_desc,
					  struct sam_cio_op_result *result)
{
	pr_info("%s: Not supported yet\n", __func__);
}

void sam_ipsec_ip4_tunnel_out_post_proc(struct sam_cio_op *operation, struct sam_hw_res_desc *res_desc,
					struct sam_cio_op_result *result)
{
	u32 val32;
	u16 ip_len;
	u8 l3_offset;
	struct sam_session_params *params = &operation->sa->params;
	struct iphdr *iph;

	val32 = readl_relaxed(&res_desc->words[7]);
	l3_offset = SAM_RES_TOKEN_OFFSET_GET(val32);
	iph = (struct iphdr *)(operation->out_frags[0].vaddr + l3_offset);

	ip_len = htobe16((u16)(result->out_len - l3_offset));

	val32 = readl_relaxed(&res_desc->words[5]);

	/* Build IPv4 header */
	memcpy(iph, operation->sa->tunnel_header, sizeof(struct iphdr));

	if (params->u.ipsec.tunnel.copy_dscp)
		iph->tos = SAM_RES_TOKEN_TOS_GET(val32);

	iph->tot_len = ip_len;
	if (params->u.ipsec.tunnel.copy_df) {
		if (val32 & SAM_RES_TOKEN_DF_MASK)
			iph->frag_off |= IP_DF;
		else
			iph->frag_off &= ~IP_DF;
	}
}

void sam_ipsec_ip6_tunnel_out_post_proc(struct sam_cio_op *operation, struct sam_hw_res_desc *res_desc,
					struct sam_cio_op_result *result)
{
	pr_info("%s: Not supported yet\n", __func__);
}

void sam_ipsec_ip4_tunnel_in_post_proc(struct sam_cio_op *operation, struct sam_hw_res_desc *res_desc,
					struct sam_cio_op_result *result)
{
	u32 val32;
	u8 l3_offset;
	struct sam_session_params *params = &operation->sa->params;
	struct iphdr *iph;

	val32 = readl_relaxed(&res_desc->words[7]);
	l3_offset = SAM_RES_TOKEN_OFFSET_GET(val32);
	iph = (struct iphdr *)(operation->out_frags[0].vaddr + l3_offset);

	if (params->u.ipsec.tunnel.copy_dscp)
		iph->tos = SAM_RES_TOKEN_TOS_GET(val32);

	if (params->u.ipsec.tunnel.copy_df) {
		if (val32 & SAM_RES_TOKEN_DF_MASK)
			iph->frag_off |= IP_DF;
		else
			iph->frag_off &= ~IP_DF;
	}
}

void sam_ipsec_ip6_tunnel_in_post_proc(struct sam_cio_op *operation, struct sam_hw_res_desc *res_desc,
					struct sam_cio_op_result *result)
{
	pr_info("%s: Not supported yet\n", __func__);
}

int sam_cio_enq_ipsec(struct sam_cio *cio, struct sam_cio_ipsec_params *requests, u16 *num)
{
	struct sam_sa *session;
	struct sam_cio_op *operation;
	struct sam_cio_ipsec_params *request;
	int i, j, err, todo, reuse;

	todo = *num;
	if (todo >= cio->params.size)
		todo = cio->params.size - 1;

	for (i = 0; i < todo; i++) {
		request = &requests[i];

		/* Check request validity */
		err = sam_cio_check_ipsec_params(request);
		if (err)
			return err;

		/* Check maximum number of pending requests */
		if (sam_cio_is_full(cio)) {
			SAM_STATS(cio->stats.enq_full++);
			break;
		}
#ifdef MVCONF_SAM_DEBUG
		if (sam_debug_flags & SAM_CIO_DEBUG_FLAG)
			print_sam_cio_ipsec_params(request);
#endif /* MVCONF_SAM_DEBUG */

		session = request->sa;

		/* Get next operation structure */
		operation = &cio->operations[cio->next_request];

		/* Save some fields from request needed for result processing */
		operation->sa = request->sa;
		operation->cookie = request->cookie;
		operation->copy_len = request->pkt_size;
		operation->num_bufs = request->num_bufs;
		for (j = 0;  j < operation->num_bufs; j++) {
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
			reuse = 0;
			session->cio = cio;
		} else
			reuse = 1;

		if (cio->hw_ring.type == HW_EIP197) {
			sam_hw_ring_ipsec_request_set(&cio->hw_ring, cio->next_request,
						      session, request, reuse);
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


