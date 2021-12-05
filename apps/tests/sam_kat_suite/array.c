/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "array.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/**
 * array_t: Structure for data array and its length,
 *
 * data - data array.
 * len - The length of the data.
 */
struct array_t {
	unsigned char *data;
	int len;
};
///////////////////////////////////////////////////////////////////////////////
/**
 * arrayCopyData: Creates a copy of the given data.
 *
 * @param data - The data to be copied.
 * @param len - The data length.
 * @return
 * 	NULL - If there was an allocation problem
 * 	The copied data otherwise
 */
static unsigned char *arrayCopyData(unsigned char *data, int len)
{
	unsigned char *copyData = malloc(len);
	int i;
	if (copyData == NULL) {
		return NULL;
	}
	for (i = 0; i < len; i++) {
		copyData[i] = data[i];
	}
	return copyData;
}
///////////////////////////////////////////////////////////////////////////////

ArrayPtr arrayCreate(unsigned char *data, int len)
{
	if (!data || len <= 0) {
		return NULL;
	}
	ArrayPtr newArray = malloc(sizeof(*newArray));
	if (!newArray) {
		return NULL;
	}
	newArray->data = arrayCopyData(data, len);
	if (!newArray->data) {
		free(newArray);
		return NULL;
	}
	newArray->len = len;
	return newArray;
}

ArrayPtr arrayCopy(ArrayPtr array)
{
	if (!array) {
		return NULL;
	}
	ArrayPtr copyArray = malloc(sizeof(*copyArray));
	if (!copyArray) {
		return NULL;
	}
	copyArray->data = arrayCopyData(array->data, array->len);
	if (!copyArray->data) {
		free(copyArray);
		return NULL;
	}
	copyArray->len = array->len;
	return copyArray;
}

void arrayDestroy(ArrayPtr array)
{
	if (!array) {
		return;
	}
	free(array->data);
	free(array);
}

int arrayGetLen(ArrayPtr array)
{
	if (!array) {
		return 0;
	}
	return array->len;
}

int arrayGetData(ArrayPtr array, unsigned char *dst, int dstLen)
{
	if (!array || !dst) {
		return 0;
	}
	if (dstLen > array->len)
		dstLen = array->len;

	memcpy(dst, array->data, dstLen);

	return dstLen;
}
