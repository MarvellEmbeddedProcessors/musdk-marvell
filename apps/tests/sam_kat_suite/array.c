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
