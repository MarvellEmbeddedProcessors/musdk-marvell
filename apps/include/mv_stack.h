/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __mv_stack_h__
#define __mv_stack_h__

/* includes */
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "mv_std.h"

/* LIFO Stack implementation */

/**
 * Stack instance structure
 */
struct mv_stack {
	int stack_idx;		/**< index of next free place in the stack */
	int num_of_elements;	/**< maximum number of elements can be pushed to stack */
	void **stack_elements;	/**< array of elements in the stack */
};

/**
 *  Check if the stack is full
 *
 * @param[in]	stack	  - stack handler.
 *
 * @retval	true	- stack is full
 * @retval	false	- stack is not full
 */
static inline bool mv_stack_is_full(struct mv_stack *stack)
{
	if (stack->stack_idx == stack->num_of_elements)
		return true;

	return false;
}

/**
 *  Check if the stack is empty
 *
 * @param[in]	stack	  - stack handler.
 *
 * @retval	true	- stack is empty
 * @retval	false	- stack is not empty
 */
static inline bool mv_stack_is_empty(struct mv_stack *stack)
{
	if (stack->stack_idx == 0)
		return true;

	return false;
}

/**
 *  Push new element to the stack
 *
 * @param[in]	stack	  - stack handler.
 * @param[in]	value	  - value to be pushed to the stack.
 *
 * @retval	0         - Success. New element was added to the stack.
 * @retval	Negative  - Failure. Stack is full.
 */
static inline int mv_stack_push(struct mv_stack *stack, void *value)
{
	if (stack->stack_idx == stack->num_of_elements) {
		pr_err("%s: Stack is FULL\n", __func__);
		return -1;
	}

	stack->stack_elements[stack->stack_idx] = value;
	stack->stack_idx++;
	return 0;
}

/**
 *  Pop element from the top of the stack
 *
 * @param[in]	stack	  - stack handler.
 *
 * @retval	value   - Success. Returned element is removed from the stack.
 * @retval	NULL	- Failure. Stack is empty.
 */
static inline void *mv_stack_pop(struct mv_stack *stack)
{
	if (stack->stack_idx == 0) {
		pr_debug("%s: Stack is EMPTY\n", __func__);
		return NULL;
	}

	stack->stack_idx--;
	return stack->stack_elements[stack->stack_idx];
}

/**
 *  Get number of elements in the stack
 *
 * @param[in]	stack	  - stack handler.
 *
 * @retval	value   - Number of elements stored in the stack.
 */
static inline int mv_stack_elements(struct mv_stack *stack)
{
	return stack->stack_idx;
}

/**
 *  Get number of free places in the stack
 *
 * @param[in]	stack	  - stack handler.
 *
 * @retval	value   - Number of free places in the stack.
 */
static inline int mv_stack_free_elements(struct mv_stack *stack)
{
	return stack->num_of_elements - stack->stack_idx;
}

/**
 *  Create new stack instance
 *
 * @param[in]	num_of_elements - maximum number of elements in the stack.
 * @param[out]	stack		- address of place to save handler of new created stack instance.
 *
 * @retval	0         - success
 * @retval	Negative  - failure
 */
int mv_stack_create(int num_of_elements, struct mv_stack **stack);

/**
 * Delete stack instance
 *
 * @param[in]	stack	  - stack handler.
 *
 * @retval	0         - success
 * @retval	Negative  - failure
 */
int mv_stack_delete(struct mv_stack *stack);

#endif /* __mv_stack_h__ */
