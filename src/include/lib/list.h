/*******************************************************************************
 * Copyright (C) Marvell International Ltd. and its affiliates
 *
 * This software file (the "File") is owned and distributed by Marvell
 * International Ltd. and/or its affiliates ("Marvell") under the following
 * alternative licensing terms.  Once you have made an election to distribute the
 * File under one of the following license alternatives, please (i) delete this
 * introductory statement regarding license alternatives, (ii) delete the three
 * license alternatives that you have not elected to use and (iii) preserve the
 * Marvell copyright notice above.
 *
 ********************************************************************************
 * Marvell Commercial License Option
 *
 * If you received this File from Marvell and you have entered into a commercial
 * license agreement (a "Commercial License") with Marvell, the File is licensed
 * to you under the terms of the applicable Commercial License.
 *
 ********************************************************************************
 * Marvell GPL License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the General
 * Public License Version 2, June 1991 (the "GPL License"), a copy of which is
 * available along with the File in the license.txt file or by writing to the Free
 * Software Foundation, Inc., or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
 *
 * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
 * DISCLAIMED.  The GPL License provides additional details about this warranty
 * disclaimer.
 *
 ********************************************************************************
 * Marvell GNU General Public License FreeRTOS Exception
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File in accordance with the terms and conditions of the Lesser
 * General Public License Version 2.1 plus the following FreeRTOS exception.
 * An independent module is a module which is not derived from or based on
 * FreeRTOS.
 * Clause 1:
 * Linking FreeRTOS statically or dynamically with other modules is making a
 * combined work based on FreeRTOS. Thus, the terms and conditions of the GNU
 * General Public License cover the whole combination.
 * As a special exception, the copyright holder of FreeRTOS gives you permission
 * to link FreeRTOS with independent modules that communicate with FreeRTOS solely
 * through the FreeRTOS API interface, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting combined work
 * under terms of your choice, provided that:
 * 1. Every copy of the combined work is accompanied by a written statement that
 * details to the recipient the version of FreeRTOS used and an offer by yourself
 * to provide the FreeRTOS source code (including any modifications you may have
 * made) should the recipient request it.
 * 2. The combined work is not itself an RTOS, scheduler, kernel or related
 * product.
 * 3. The independent modules add significant and primary functionality to
 * FreeRTOS and do not merely extend the existing functionality already present in
 * FreeRTOS.
 * Clause 2:
 * FreeRTOS may not be used for any competitive or comparative purpose, including
 * the publication of any form of run time or compile time metric, without the
 * express permission of Real Time Engineers Ltd. (this is the norm within the
 * industry and is intended to ensure information accuracy).
 *
 ********************************************************************************
 * Marvell BSD License Option
 *
 * If you received this File from Marvell, you may opt to use, redistribute and/or
 * modify this File under the following licensing terms.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *	* Redistributions of source code must retain the above copyright notice,
 *	  this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the distribution.
 *
 *	* Neither the name of Marvell nor the names of its contributors may be
 *	  used to endorse or promote products derived from this software without
 *	  specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************/

#ifndef __LIST_H__
#define __LIST_H__

#ifdef __KERNEL__

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

#else

/**
 * A list structure
 */
struct list {
	struct list *next;  /**< A pointer to the next list object	*/
	struct list *prev;  /**< A pointer to the previous list object	*/
};

#define LIST_FIRST(_lst)	(_lst)->next
#define LIST_LAST(_lst)		(_lst)->prev
#define LIST_NEXT		LIST_FIRST
#define LIST_PREV		LIST_LAST

/**
 * Macro to declare of a list.
 *
 * @param[in]	_lst_name	The list object name.
 */
#define LIST(_lst_name)	struct list _lst_name = LIST_INIT(_lst_name)

/**
 * Macro for initialization of a list struct.
 *
 * @param[in]	_lst	The struct list object to initialize.
 */
#define LIST_INIT(_lst)	{&(_lst), &(_lst)}

/**
 * Macro to initialize a list pointer.
 *
 * @param[in]	_lst	The list pointer.
 */
#define INIT_LIST(_lst)   LIST_FIRST(_lst) = LIST_LAST(_lst) = (_lst)

/**
 * Macro to get the struct (object) for this entry.
 *
 * @param[in]	_lst	The list pointer.
 * @param[in]	_type	The type of the struct (object) this list is embedded in.
 * @param[in]	_member	The name of the struct list object within the struct.
 *
 * @return	The structure pointer for this entry.
 */
#define LIST_OBJECT(_lst, _type, _member) \
	((_type *)((char *)(_lst) - MEMBER_OFFSET(_type, _member)))


/**
 * Macro to get the struct (object) for entry->next.
 *
 * @param[in]	_lst	The list pointer.
 * @param[in]	_type	The type of the struct (object) this list is embedded in.
 * @param[in]	_member	The name of the struct list object within the struct.
 *
 * @return	The structure pointer for entry->next.
 */
#define LIST_FIRST_OBJECT(_lst, _type, _member) \
	LIST_OBJECT(_lst->next, _type, _member)

/**
 * Macro to iterate over a list.
 *
 * @param[in]	_pos	A pointer to a list to use as a loop counter.
 * @param[in]	_head	A pointer to the head for your list pointer.
 *
 * @caution	You can't delete items with this routine. For deletion,
 *			use LIST_FOR_EACH_SAFE().
 */
#define LIST_FOR_EACH(_pos, _head) \
	for (_pos = LIST_FIRST(_head); _pos != (_head); _pos = LIST_NEXT(_pos))

/**
 * Macro to iterate over a list safe against removal of list entry.
 *
 * @param[in]	_pos	A pointer to a list to use as a loop counter.
 * @param[in]	_tmp	Another pointer to a list to use as temporary storage.
 * @param[in]	_head	A pointer to the head for your list pointer.
 */
#define LIST_FOR_EACH_SAFE(_pos, _tmp, _head)			\
	for (_pos = LIST_FIRST(_head), _tmp = LIST_FIRST(_pos);	\
		_pos != (_head);				\
		_pos = _tmp, _tmp = LIST_NEXT(_pos))

/**
 * Macro to iterate over list of given type safely.
 *
 * @param[in]	_pos	A pointer to a list to use as a loop counter.
 * @param[in]	_tmp	Another pointer to a list to use as temporary storage.
 * @param[in]	_type	The type of the struct this is embedded in.
 * @param[in]	_head	A pointer to the head for your list pointer.
 * @param[in]	_member	The name of the list_struct within the struct.
 */
#define LIST_FOR_EACH_OBJECT_SAFE(_pos, _tmp, _head, _type, _member)		\
	for (_pos = LIST_OBJECT(LIST_FIRST(_head), _type, _member),		\
		 _tmp = LIST_OBJECT(LIST_FIRST(&_pos->_member), _type, _member);\
		 &_pos->_member != (_head);					\
		 _pos = _tmp,							\
		 _tmp = LIST_OBJECT(LIST_FIRST(&_pos->_member), _type, _member))

/**
 * Macro to iterate over list of given type.
 *
 * @param[in]	_pos	A pointer to a list to use as a loop counter.
 * @param[in]	_type	The type of the struct this is embedded in.
 * @param[in]	_head	A pointer to the head for your list pointer.
 * @param[in]	_member	The name of the list_struct within the struct.
 *
 * @caution	You can't delete items with this routine. For deletion,
 *			use LIST_FOR_EACH_OBJECT_SAFE().
 */
#define LIST_FOR_EACH_OBJECT(_pos, _type, _head, _member)		\
	for (_pos = LIST_OBJECT(LIST_FIRST(_head), _type, _member);	\
		 &_pos->_member != (_head);				\
		 _pos = LIST_OBJECT(LIST_FIRST(&_pos->_member), _type, _member))

/**
 * Macro to iterate over list of given type on reverse way.
 *
 * @param[in]	_pos	A pointer to a list to use as a loop counter.
 * @param[in]	_type	The type of the struct this is embedded in.
 * @param[in]	_head	A pointer to the head for your list pointer.
 * @param[in]	_member	The name of the list_struct within the struct.
 *
 * @caution	You can't delete items with this routine. For deletion,
 *			use LIST_FOR_EACH_OBJECT_SAFE().
 */
#define LIST_FOR_EACH_OBJECT_REVERSE(_pos, _type, _head, _member)		\
	for (_pos = LIST_OBJECT(LIST_LAST(_head), _type, _member);	\
		 &_pos->_member != (_head);				\
		 _pos = LIST_OBJECT(LIST_LAST(&_pos->_member), _type, _member))


/**
 * Add a new entry to a (head of a) list.
 *
 * Insert a new entry after the specified head.
 * This is good for implementing stacks.
 *
 * @param[in]	new_lst	A pointer to a new list entry to be added.
 * @param[in]	head	A pointer to a list head to add it after.
 *
 * @retval	none
 */
static inline void list_add(struct list *new_lst, struct list *head)
{
	LIST_PREV(LIST_NEXT(head))	= new_lst;
	LIST_NEXT(new_lst)		= LIST_NEXT(head);
	LIST_PREV(new_lst)		= head;
	LIST_NEXT(head)			= new_lst;
}

/**
 * Add a new entry to a (tail of a) list.
 *
 * Insert a new entry before the specified head.
 * This is good for implementing queues.
 *
 * @param[in]	new_lst	A pointer to a new list entry to be added.
 * @param[in]	head	A pointer to a list head to add it before.
 *
 * @retval	none
 */
static inline void list_add_to_tail(struct list *new_lst, struct list *head)
{
	LIST_NEXT(LIST_PREV(head))	= new_lst;
	LIST_PREV(new_lst)		= LIST_PREV(head);
	LIST_NEXT(new_lst)		= head;
	LIST_PREV(head)			= new_lst;
}

/**
 * Deletes entry from a list.
 *
 * @param[in]	ent		A pointer to the element to delete from the list.
 *
 * @retval	none
 *
 * @caution	list_is_empty() on entry does not return true after this,
 *			the entry is in an undefined state.
 */
static inline void list_del(struct list *ent)
{
	LIST_PREV(LIST_NEXT(ent)) = LIST_PREV(ent);
	LIST_NEXT(LIST_PREV(ent)) = LIST_NEXT(ent);
}

/**
 * Deletes entry from list and reinitialize it.
 *
 * @param[in]	ent		A pointer to the element to delete from the list.
 *
 * @retval	none
 */
static inline void list_del_init(struct list *ent)
{
	list_del(ent);
	INIT_LIST(ent);
}

/**
 * Delete from one list and add as another's head.
 *
 * @param[in]	ent		A pointer to the entry to move.
 * @param[in]	head	A pointer to the list head that will follow our entry.
 *
 * @retval	none
 */
static inline void list_move(struct list *ent, struct list *head)
{
	list_del(ent);
	list_add(ent, head);
}

/**
 * Delete from one list and add as another's tail.
 *
 * @param[in]	ent		A pointer to the entry to move.
 * @param[in]	head	A pointer to the list head that will follow our entry.
 *
 * @retval	none
 */
static inline void list_move_to_tail(struct list *ent, struct list *head)
{
	list_del(ent);
	list_add_to_tail(ent, head);
}

/**
 * Check whether a list is empty.
 *
 * @param[in]	lst		A pointer to the list to test.
 *
 * @retval	1 if the list is empty.
 * @retval	0 if the list is not empty.
 */
static inline int list_is_empty(struct list *lst)
{
	return (LIST_FIRST(lst) == lst);
}

/**
 * Join two lists.
 *
 * @param[in]	new_lst		A pointer to the new list to add.
 * @param[in]	head		A pointer to the place to add it in the first list.
 *
 * @retval	none
 */
void list_append(struct list *new_lst, struct list *head);

/**
 * Count number of objects in the list.
 *
 * @param[in]	lst		A pointer to the list which objects are to be counted.
 *
 * @retval	Number of objects in the list.
 */
int list_num_objs(struct list *lst);

#endif

#endif /* __LIST_H__ */
