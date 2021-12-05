/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __LIST_KS_H__
#define __LIST_KS_H__

#include <linux/list.h>

/* Kernel list implementation wrappers */

#define list			list_head
#define list_is_empty		list_empty
#define list_add_to_tail	list_add
#define LIST_FIRST_OBJECT	list_first_entry
#define LIST_FOR_EACH		list_for_each
#define LIST_OBJECT		container_of
#define INIT_LIST		INIT_LIST_HEAD

#define LIST_FOR_EACH_OBJECT(_pos, _type, _head, _member) \
	list_for_each_entry(_pos, _head, _member)
#define LIST_FOR_EACH_OBJECT_REVERSE(_pos, _type, _head, _member) \
	list_for_each_entry_reverse(_pos, _head, _member)

/* Additional code */

#define LIST_FIRST(_lst)	((_lst)->next)
#define LIST_LAST(_lst)		((_lst)->prev)
#define LIST_NEXT		LIST_FIRST
#define LIST_PREV		LIST_LAST

int list_num_objs(struct list *lst);

#endif /* __LIST_KS_H__ */
