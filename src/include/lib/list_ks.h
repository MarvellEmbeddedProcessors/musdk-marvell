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
