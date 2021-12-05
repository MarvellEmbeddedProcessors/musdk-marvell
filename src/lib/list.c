/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "std_internal.h"
#include "lib/list.h"

void list_append(struct list *new_lst, struct list *head)
{
	struct list *frst = LIST_FIRST(new_lst);

	if (frst != new_lst) {
		struct list *last = LIST_LAST(new_lst);
		struct list *curr = LIST_NEXT(head);

		LIST_PREV(frst)  = head;
		LIST_FIRST(head) = frst;
		LIST_NEXT(last)  = curr;
		LIST_LAST(curr)  = last;
	}
}

int list_num_objs(struct list *lst)
{
	struct list *tmp;
	int			 num_objs = 0;

	if (!list_is_empty(lst))
		LIST_FOR_EACH(tmp, lst)
			num_objs++;

	return num_objs;
}
