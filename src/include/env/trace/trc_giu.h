/******************************************************************************
 *	Copyright (C) 2017 Marvell International Ltd.
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
#ifdef TRACE

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER gie

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "trc_giu.h"

#if !defined(_TRC_GIU_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TRC_GIU_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
	gie,
	flow,
	TP_ARGS(
	char*, message,
	char*, giu_name
	),
	TP_FIELDS(
	ctf_string(msg, message)
	ctf_string(giu_name, giu_name)
	)
)

TRACEPOINT_EVENT(
	gie,
	dma,
	TP_ARGS(
	void*, src,
	void*, dst,
	int, size
	),
	TP_FIELDS(
	ctf_integer_hex(void *, src, src)
	ctf_integer_hex(void *, dst, dst)
	ctf_integer_hex(int, size, size)
	)
)

TRACEPOINT_EVENT(
	gie,
	queue,
	TP_ARGS(
	char*, msg,
	int, qes,
	int, src_qid,
	int, src_head,
	int, src_tail,
	int, dst_qid,
	int, dst_head,
	int, dst_tail
	),
	TP_FIELDS(
	ctf_string(msg, msg)
	ctf_integer(int, qes, qes)
	ctf_integer(int, src_qid, src_qid)
	ctf_integer(int, src_head, src_head)
	ctf_integer(int, src_tail, src_tail)
	ctf_integer(int, dst_qid, dst_qid)
	ctf_integer(int, dst_head, dst_head)
	ctf_integer(int, dst_tail, dst_tail)
	)
)

#endif /* _TRC_GIU_H */

#include <lttng/tracepoint-event.h>

#else /* TRACE */
#define tracepoint(name, args...)
#endif /* TRACE */
