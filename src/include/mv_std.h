/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __MV_STD_H__
#define __MV_STD_H__

/* MUSDK primary build flags. must be at the top */
#include "env/mv_autogen_build_assert.h"
#include "env/mv_autogen_comp_flags.h"

/* MUSDK enviromental related headers */
#ifdef __KERNEL__
/* MUSDK should have been compliant with Linux Kernel;
 * so, there should not be any include files here
 */

#else /* __KERNEL__ */
#include "env/mv_types.h"
#include "env/mv_compiler.h"
#include "env/mv_common.h"
#include "env/mv_errno.h"
#include "env/mv_debug.h"
#endif /* __KERNEL__ */

/* MUSDK specific (but general) header files */
#include "env/mv_sys_dma.h"

#endif /* __MV_STD_H__ */
