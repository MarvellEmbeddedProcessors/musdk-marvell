/******************************************************************************
 *      Copyright (C) 2016 Marvell International Ltd.
 *
 *  If you received this File from Marvell, you may opt to use, redistribute
 *  and/or modify this File under the following licensing terms.
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of Marvell nor the names of its contributors may be
 *        used to endorse or promote products derived from this software
 *        without specific prior written permission.
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

#ifndef GENERIC_LIST_H_
#define GENERIC_LIST_H_

#include <stdbool.h>

/** Type for defining the generic list struct */
typedef struct generic_list_t* generic_list;

/** Defining error messages for generic list functions */
typedef enum generic_list_message_t {
	LIST_SUCCESS, LIST_NULL_ARGUMENT, LIST_OUT_OF_MEMORY, LIST_INVALID_CURRENT,
} generic_list_message;

/** Type for defining the generic list element */
typedef void* generic_list_element;

/** Function that Copies the given element*/
typedef generic_list_element (*copy_generic_list_element)(generic_list_element);

/** Function that deallocating the given element */
typedef void (*free_generic_list_element)(generic_list_element);

/** Function that compares between two given elements */
typedef int (*compare_elements)(generic_list_element, generic_list_element);

/** Defining key  for filter function */
typedef void* filter_key;

/** Checks if the element need to be filtered */
typedef bool (*filter_generic_list_element)(generic_list_element, filter_key);

/**
 * Creates a new generic list (allocated).
 * @param copy_element - A copy element function pointer
 * @param free_element - A free element function pointer
 * @return
 * 	NULL - in case of allocation failed or NULL pointer argument
 * 	A new empty generic list otherwise.
 */
generic_list generic_list_create(copy_generic_list_element copy_element, free_generic_list_element free_element);

/**
 * Copies the given generic list.
 *
 * @param the list to be copied
 * @return
 * 	NULL - in case of allocation failed or NULL pointer argument
 * 	A copied generic list otherwise.
 */
generic_list generic_list_copy(generic_list list);

/** Returns the number of elements in the given list */
int generic_list_get_size(generic_list list);

/** Sets the internal iterator to the first element and return it */
generic_list_element generic_list_get_first(generic_list list);

/** Sets the internal iterator to the last element and return it. */
generic_list_element generic_list_get_last(generic_list list);

/** Sets the internal iterator to the next element and return it. */
generic_list_element generic_list_get_next(generic_list list);

/** Return the internal iterator element */
generic_list_element generic_list_get_current(generic_list list);

/** Inserts a new element to the head of the given list */
generic_list_message generic_list_insert_first(generic_list list, generic_list_element element);

/** Inserts a new element to the tail of the given list */
generic_list_message generic_list_insert_last(generic_list list, generic_list_element element);

/** Inserts a new element before the internal iterator of the given list */
generic_list_message generic_list_insert_before_current(generic_list list, generic_list_element element);

/** Inserts a new element after the internal iterator of the given list */
generic_list_message generic_list_insert_after_current(generic_list list, generic_list_element element);

/** Removes the element that pointered by the internal iterator */
generic_list_message generic_list_remove_current(generic_list list);

/** Sorts the given list according to the compare function */
generic_list_message generic_list_sort(generic_list list, compare_elements compare_element);

/** Returns a new filtered list according to the parameters given to the function */
generic_list generic_list_filter(generic_list list, filter_generic_list_element filter_element, filter_key key);

/** Removes all the elements from the given list */
generic_list_message generic_list_clear(generic_list list);

/** Deallocating the given list and its elements */
void generic_list_destroy(generic_list list);

/** Iteration over all rhe elements of the given list*/
#define LIST_FOREACH(type,iterator,list) \
	type iterator; \
	for(iterator = generic_list_get_first(list) ; \
		iterator ;\
		iterator = generic_list_get_next(list))

#endif
