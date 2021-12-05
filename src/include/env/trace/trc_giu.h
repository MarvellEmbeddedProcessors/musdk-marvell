/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

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
