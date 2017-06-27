/******************************************************************************
 *	Copyright (C) 2016 Marvell International Ltd.
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
