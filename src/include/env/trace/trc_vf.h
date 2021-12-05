/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifdef TRACE

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER nmnicvf

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "trc_vf.h"

#if !defined(_TRC_NIC_VF_H) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define _TRC_NIC_VF_H

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
	nmnicvf,
	setup,
	TP_ARGS(
	int, nic_vf_id,
	char*, message
	),
	TP_FIELDS(
	ctf_string(msg, message)
	ctf_integer(int, nic_vf_id, nic_vf_id)
	)
)

#endif /* _TRC_NIC_VF_H */

#include <lttng/tracepoint-event.h>

#else /* TRACE */
#define tracepoint(name, args...)
#endif /* TRACE */
