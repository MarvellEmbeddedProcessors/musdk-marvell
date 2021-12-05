/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* includes */

#include "mv_stack.h"

/* defines  */

/* Public functions */

/**
 *  Create new stack instance
 *
 * @param[in]	num_of_elements - maximum number of elements in the stack.
 * @param[out]	stack		- address of place to save handler of new created stack instance.
 *
 * @retval	0         - success
 * @retval	Negative  - failure
 */
int mv_stack_create(int num_of_elements, struct mv_stack **stack)
{
	struct mv_stack *p_stack;
	void **p_stack_elements;

	p_stack = malloc(sizeof(struct mv_stack));
	p_stack_elements = malloc(num_of_elements * sizeof(void *));
	if ((p_stack == NULL) || (p_stack_elements == NULL)) {
		pr_err("%s: Can't create new stack\n", __func__);
		free(p_stack);
		free(p_stack_elements);
		return -ENOMEM;
	}
	memset(p_stack_elements, 0, num_of_elements * sizeof(u32));
	p_stack->num_of_elements = num_of_elements;
	p_stack->stack_idx = 0;
	p_stack->stack_elements = p_stack_elements;

	*stack = p_stack;
	return 0;
}

/**
 * Delete stack instance
 *
 * @param[in]	stack	  - stack handler.
 *
 * @retval	0         - success
 * @retval	Negative  - failure
 */
int mv_stack_delete(struct mv_stack *stack)
{
	if ((stack == NULL) || (stack->stack_elements == NULL))
		return 0;

	if (stack->stack_idx != 0)
		pr_warn("%d stack elements are not freed yet\n", stack->stack_idx);

	free(stack->stack_elements);
	free(stack);

	return 0;
}
