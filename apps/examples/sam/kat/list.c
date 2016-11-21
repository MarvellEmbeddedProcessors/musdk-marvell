/*******************************************************************************
Copyright (C) 2016 Marvell International Ltd.

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the three
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

*******************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>

#include "generic_list.h"

typedef struct node_t {
	generic_list_element data;
	struct node_t* next;
	copy_generic_list_element copyElementFunc;
	free_generic_list_element freeElementFunc;
}*Node;

struct generic_list_t {
	Node head;
	Node currentNode;
	copy_generic_list_element copyElement;
	free_generic_list_element freeElement;
};

void generic_list_destroy(generic_list list);
static Node nodeCreate(generic_list_element element, Node nextNode,
		copy_generic_list_element copyElementFunc, free_generic_list_element freeElementFunc);
static void nodeDestroySingle(Node node);
static void nodeDestroyAll(Node node);
static Node listGetPreviousNode(generic_list list);
static bool nodeCopyChain(Node src, Node* dst);
static Node listGetLastNode(generic_list list);
static bool listIsEmpty(generic_list list);

static void swap(Node first, Node second);
generic_list generic_list_create(copy_generic_list_element copyElement, free_generic_list_element freeElement) {
	if (copyElement == NULL || freeElement == NULL) {
		return NULL;
	}
	generic_list newList = malloc(sizeof(*newList));
	if (newList == NULL) {
		return NULL;
	}
	newList->head = NULL;
	newList->currentNode = NULL;
	newList->copyElement = copyElement;
	newList->freeElement = freeElement;
	return newList;
}

generic_list generic_list_copy(generic_list list) {
	if (list == NULL) {
		return NULL;
	}
	generic_list copyList = generic_list_create(list->copyElement, list->freeElement);
	if (copyList == NULL) {
		return NULL;
	}
	if (!nodeCopyChain(list->head, &copyList->head)) {
		generic_list_destroy(copyList);
		return NULL;
	}
	if (!generic_list_get_current(list)) {
		copyList->currentNode = NULL;
	} else {
		Node ptr1 = list->head, ptr2 = copyList->head;
		while (ptr1 != list->currentNode) {
			ptr2 = ptr2->next;
			ptr1 = ptr1->next;
		}
		copyList->currentNode = ptr2;
	}

	return copyList;
}

static bool nodeCopyChain(Node src, Node *dst) {
// Stop condition
	if (src == NULL) {
		*dst = NULL;
		return true;
	}
// Recursive step
	Node restNodes = NULL;
	if (!nodeCopyChain(src->next, &restNodes)) {
		return false;
	}

// Copy current node
	Node newNode = nodeCreate(src->data, restNodes, src->copyElementFunc,
			src->freeElementFunc);
	if (!newNode) {
		nodeDestroyAll(restNodes);
		return false;
	}

	*dst = newNode;

	return true;

}

static Node nodeCreate(generic_list_element element, Node nextNode,
		copy_generic_list_element copyElementFunc, free_generic_list_element freeElementFunc) {
	Node newNode = malloc(sizeof(*newNode));
	if (newNode == NULL) {
		return NULL;
	}
	generic_list_element newElement = copyElementFunc(element);
	if (!newElement) {
		nodeDestroySingle(newNode);
		return NULL;
	}
	newNode->data = newElement;
	newNode->next = nextNode;
	newNode->copyElementFunc = copyElementFunc;
	newNode->freeElementFunc = freeElementFunc;
	return newNode;
}
static void nodeDestroySingle(Node node) {
	if (!node) {
		return;
	}
	if (node->data) {
		node->freeElementFunc(node->data);
	}
	free(node);
}

static void nodeDestroyAll(Node node) {
	if (!node) {
		return;
	}
	nodeDestroyAll(node->next);
	if (node->data) {
		node->freeElementFunc(node->data);
	}
	free(node);
}

void generic_list_destroy(generic_list list) {
	if (list == NULL) {
		return;
	}
	nodeDestroyAll(list->head);
	free(list);
}

generic_list_message generic_list_insert_first(generic_list list, generic_list_element element) {
	if (!list || !element) {
		return LIST_NULL_ARGUMENT;
	}
	Node newNode = nodeCreate(element, list->head, list->copyElement,
			list->freeElement);
	if (newNode == NULL) {
		return LIST_OUT_OF_MEMORY;
	}
	list->head = newNode;
	return LIST_SUCCESS;
}

generic_list_message generic_list_insert_last(generic_list list, generic_list_element element) {
	if (!list || !element) {
		return LIST_NULL_ARGUMENT;
	}
	Node newNode = nodeCreate(element, NULL, list->copyElement,
			list->freeElement);
	if (!newNode) {
		return LIST_OUT_OF_MEMORY;
	}

	if (listIsEmpty(list)) {
		list->head = newNode;
	} else {
		Node last = listGetLastNode(list);
		last->next = newNode;
	}
	return LIST_SUCCESS;
}

generic_list_message generic_list_insert_after_current(generic_list list, generic_list_element element) {
	if (!list || !element) {
		return LIST_NULL_ARGUMENT;
	}
	if (!generic_list_get_current(list)) {
		return LIST_INVALID_CURRENT;
	}
	Node newNode = nodeCreate(element, list->currentNode->next,
			list->copyElement, list->freeElement);
	if (!newNode) {

		return LIST_OUT_OF_MEMORY;
	}
	list->currentNode->next = newNode;
	return LIST_SUCCESS;
}

generic_list_message generic_list_insert_before_current(generic_list list, generic_list_element element) {
	if (!list || !element) {
		return LIST_NULL_ARGUMENT;
	}
	if (!generic_list_get_current(list)) {

		return LIST_INVALID_CURRENT;

	} else if (list->currentNode == list->head) {

		list->head = nodeCreate(element, list->currentNode, list->copyElement,
				list->freeElement);

		if (!list->head) {
			list->head = list->currentNode;
			return LIST_OUT_OF_MEMORY;
		}

	} else {
		Node previousNode = listGetPreviousNode(list);
		assert(previousNode);
		previousNode->next = nodeCreate(element, list->currentNode,
				list->copyElement, list->freeElement);

		if (!previousNode->next) {
			previousNode->next = list->currentNode;
			return LIST_OUT_OF_MEMORY;
		}
	}
	return LIST_SUCCESS;
}

generic_list_message generic_list_sort(generic_list list, compare_elements compareElement) {
	if (!list || !compareElement) {
		return LIST_NULL_ARGUMENT;
	}

	Node ptrCurrent = list->head;
	Node ptrLast = NULL;

	/* Checking for empty list */
	if (ptrCurrent == NULL) {
		return LIST_SUCCESS;
	}

	/* Bubble sort the given linked list */
	bool swapped = false;
	do {
		swapped = false;
		ptrCurrent = list->head;

		while (ptrCurrent->next != ptrLast) {
			assert(ptrCurrent->data);
			assert(ptrCurrent->next->data);
			if (compareElement(ptrCurrent->data, ptrCurrent->next->data) > 0) {
				swap(ptrCurrent, ptrCurrent->next);
				swapped = true;
			}
			ptrCurrent = ptrCurrent->next;
		}
		ptrLast = ptrCurrent;
	} while (swapped);

	return LIST_SUCCESS;
}

/* function to swap data of two nodes a and b*/
static void swap(Node first, Node second) {
	generic_list_element temp = first->data;
	first->data = second->data;
	second->data = temp;
}
generic_list generic_list_filter(generic_list list, filter_generic_list_element filterElement, filter_key key) {
	if (!filterElement || !list) {
		return NULL;
	}

	generic_list filteredList = generic_list_create(list->copyElement, list->freeElement);

	LIST_FOREACH(generic_list_element, element, list)
	{
		if (filterElement(element, key)) {
			generic_list_insert_last(filteredList, element);
		}
	}

	// Set the current node of the filteredList to its first element
	generic_list_get_first(filteredList);

	return filteredList;
}

generic_list_message generic_list_remove_current(generic_list list) {
	if (!list) {
		return LIST_NULL_ARGUMENT;
	}
	if (!generic_list_get_current(list)) {
		return LIST_INVALID_CURRENT;
	}
	if (list->currentNode == list->head) {
		list->head = list->head->next;
	} else {
		Node previousNode = listGetPreviousNode(list);
		assert(previousNode != NULL);
		previousNode->next = list->currentNode->next;
	}
	nodeDestroySingle(list->currentNode);
	list->currentNode = NULL;
	return LIST_SUCCESS;
}

generic_list_message generic_list_clear(generic_list list) {
	if (!list) {
		return LIST_NULL_ARGUMENT;
	}
	nodeDestroyAll(list->head);
	list->currentNode = NULL;
	list->head = NULL;
	return LIST_SUCCESS;
}

generic_list_element generic_list_get_first(generic_list list) {
	if (!list || listIsEmpty(list)) {
		return NULL;
	}
	list->currentNode = list->head;
	return list->currentNode->data;
}

generic_list_element generic_list_get_last(generic_list list) {
	if (!list || listIsEmpty(list)) {
		return NULL;
	}
	list->currentNode = listGetLastNode(list);
	return list->currentNode->data;
}

generic_list_element generic_list_get_next(generic_list list) {
	if (!list || !generic_list_get_current(list)) {
		return NULL;
	}
	if (!list->currentNode->next) {
		list->currentNode = NULL;
		return NULL;
	}
	list->currentNode = list->currentNode->next;
	return list->currentNode->data;
}

generic_list_element generic_list_get_current(generic_list list) {

	if (!list || !list->head
			|| (!listGetPreviousNode(list) && (list->currentNode != list->head))) {
		return NULL;
	}
	return list->currentNode->data;
}

static Node listGetLastNode(generic_list list) {
	if (!list || listIsEmpty(list)) {
		return NULL;
	}
	Node last;
	for (last = list->head; last->next != NULL; last = last->next) {
	}
	return last;
}
static Node listGetPreviousNode(generic_list list) {
	if (!list || listIsEmpty(list) || !list->currentNode) {
		return NULL;
	}
	Node previous;
	for (previous = list->head;
			previous->next != list->currentNode && previous->next != NULL;
			previous = previous->next) {
	}
	if (!previous->next) {
		return NULL;
	}
	return previous;
}

static bool listIsEmpty(generic_list list) {
	assert(list);
	return !list->head;
}

int generic_list_get_size(generic_list list) {
	if (!list) {
		return -1;
	}

	int count = 0;
	Node node = NULL;
	for (node = list->head; node != NULL; node = node->next) {
		count++;
	}
	return count;
}

