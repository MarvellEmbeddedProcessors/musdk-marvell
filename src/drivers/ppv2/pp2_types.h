/**
 * @file pp2_types.h
 *
 * PPDK - Data types
 *
 */

#ifndef _PP2_TYPES_H_
#define _PP2_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

typedef uint64_t phys_addr_t;

struct base_addr {
	uintptr_t va;
	phys_addr_t pa;
};

#define TRUE    (true)
#define FALSE   (false)

#endif /* _PP2_TYPES_H_ */
