/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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

static inline u32 sam_hw_ring_ipsec_cdr_desc_write(struct sam_hw_ring *hw_ring,
						    struct sam_sa *session,
						    struct sam_cio_ipsec_params *request,
						    int reuse)
{
	int prep_data;
	u32 first_last_mask, desc_num;
	u32 proto_word;
	u32 thw;	/* token_header_word*/
	int j;

	/* Prepared CDR descriptors */
	prep_data = 0;
	desc_num = 0;
	for (j = 0;  j < request->num_bufs; j++) {
		u32 data_size = request->src[j].len;

		first_last_mask = 0;
		if (j == 0) {
			struct sam_hw_cmd_desc *cmd_desc;

			first_last_mask |= SAM_DESC_FIRST_SEG_MASK;
			/* Token is built inside the device for first descriptor only */
			cmd_desc = sam_hw_cmd_desc_get(hw_ring, hw_ring->next_cdr);
			thw = (request->pkt_size & SAM_TOKEN_PKT_LEN_MASK);
			thw |= SAM_TOKEN_TYPE_AUTO_MASK;
			if (reuse)
				thw |= SAM_TOKEN_REUSE_AUTO_MASK;

			proto_word = SAM_CMD_TOKEN_OFFSET_SET(request->l3_offset);
			if (session->params.dir == SAM_DIR_ENCRYPT)
				proto_word |= SAM_CMD_TOKEN_NEXT_HDR_SET(IPPROTO_ESP);
			else
				proto_word |= SAM_CMD_TOKEN_NEXT_HDR_SET(0);

			sam_hw_cdr_ext_token_write(cmd_desc, &request->sa->sa_buf,
						   thw, FIRMWARE_CMD_PKT_LIP_MASK,
						   proto_word);
		}

		if (j == (request->num_bufs - 1)) {
			first_last_mask |= SAM_DESC_LAST_SEG_MASK;
			if (prep_data >= request->pkt_size) {
				pr_err("%s: crypto data size %d > packet size %d\n",
						__func__, prep_data, request->pkt_size);
				sam_hw_ring_roolback(hw_ring, 1, j);
				desc_num = 0;
				break;
			}
			data_size = request->pkt_size - prep_data;
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
	u32 rdr_submit = 0;
	u32 cdr_submit = 0;

	todo = *num;
	if (todo >= cio->params.size)
		todo = cio->params.size - 1;

	for (i = 0; i < todo; i++) {
		request = &requests[i];

		/* Check request validity */
		err = sam_cio_check_ipsec_params(request);
		if (err)
			return err;

		/* Look if there are enough free resources */
		if (!sam_cio_is_free_slot(cio, request->num_bufs)) {
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
			reuse = 0;
			session->cio = cio;
		} else
			reuse = 1;

		if (cio->hw_ring.type != HW_EIP97IES) {
			u32 seg_mask;

			/* Write prepared RDR descriptor */
			seg_mask = SAM_DESC_FIRST_SEG_MASK | SAM_DESC_LAST_SEG_MASK;
			sam_hw_rdr_desc_write(&cio->hw_ring, request->dst->paddr,
					      request->dst->len, seg_mask);
			rdr_submit++;

			/* write CDR descriptors and token */
			cdr_submit += sam_hw_ring_ipsec_cdr_desc_write(&cio->hw_ring, session,
								      request, reuse);
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


