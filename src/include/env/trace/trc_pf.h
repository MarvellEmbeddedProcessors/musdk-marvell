/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifdef TRACE

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER nmnicpf

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "trc_pf.h"

#if !defined(_TRC_NIC_PF_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TRC_NIC_PF_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
	nmnicpf,
	setup,
	TP_ARGS(
	int, nic_pf_id,
	char*, message
	),
	TP_FIELDS(
	ctf_string(msg, message)
	ctf_integer(int, nic_pf_id, nic_pf_id)
	)
)

#endif /* _TRC_NIC_PF_H */

#include <lttng/tracepoint-event.h>

#else /* TRACE */
#define tracepoint(name, args...)
#endif /* TRACE */
