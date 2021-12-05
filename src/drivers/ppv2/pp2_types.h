/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pp2_types.h
 *
 * PPDK - Data types
 *
 */

#ifndef _PP2_TYPES_H_
#define _PP2_TYPES_H_

#include "std_internal.h"

struct base_addr {
	uintptr_t va;
	phys_addr_t pa;
};

#endif /* _PP2_TYPES_H_ */
