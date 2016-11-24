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

#ifndef ARRAY_H_
#define ARRAY_H_

/** Type used for returning message from array functions */
typedef enum {
	ARRAY_NULL_ARGS,
	ARRAY_SUCCESS,
	ARRAY_NOT_VALID_ARGS,
	ARRAY_OUT_OF_MEMORY,
	ARRAY_FULL,
	ARRAY_NO_ELEMENT,
	ARRAY_NOT_RIGHT_SIZE
} ArrayMessage;

///////////////////////////////////////////////////////////////////////////////

/** Type for defining the array */
typedef struct array_t* ArrayPtr;

/**
 * arrayCreate: Creates a new array structure.
 *
 * @param data - The data to insert to the new array.
 * @param len - The length of the data.
 * @return
 * 	NULL - If there was a problem
 * 	the new array otherwise
 */
ArrayPtr arrayCreate(unsigned char* data, int len);
/**
 * arrayCopy: Creates a copy of the given array.
 *
 * @param array - The array to be copied.
 * @return
 * 	NULL - If there was an allocation problem
 * 	the copied array otherwise
 */
ArrayPtr arrayCopy(ArrayPtr array);
/**
 * arrayDestroy: Destroys the given array.
 *
 * @param array - The array to be destroyed.
 */
void arrayDestroy(ArrayPtr array);
/**
 * arrayGetData: copy the array data to destination array.
 *
 * @param array - The array to copy the data from.
 * @param dstLen - The length of the destination array.
 * @param dst - The destination array to copy the element to.
 * @return
 * 	ARRAY_NULL_ARGS - If at least one of the parameters is NULL
 * 	ARRAY_NOT_RIGHT_SIZE - If size of the output array doesn't match
 * 		the element length
 * 	ARRAY_SUCCESS - If the function succeeded to copy the data
 */
ArrayMessage arrayGetData(ArrayPtr array, unsigned char* dst, int dstLen);
/**
 * arrayGetLen: Gets the length of the array data.
 *
 * @param array - The array to get the length of the data from.
 * @return
 * 	the data length
 */
int arrayGetLen(ArrayPtr array);

///////////////////////////////////////////////////////////////////////////////

/** Type for defining the complex array (matrix) */
typedef struct array_complex_t* ArrayComplexPtr;

/**
 * arrayComplexCreate: Creates a new complex array structure.
 *
 * @param size - the number of data arrays to be added.
 * @return
 * 	NULL - If there was a problem
 * 	The new complex array otherwise
 */
ArrayComplexPtr arrayComplexCreate(int size);
/**
 * arrayComplexCopy: Creates a copy of the given complex array.
 *
 * @param complexArray - The complex array to be copied.
 * @return
 * 	NULL - If there was an allocation problem
 * 	The copied complex array otherwise
 */
ArrayComplexPtr arrayComplexCopy(ArrayComplexPtr complexArray);
/**
 * arrayComplexDestroy: Destroys the given complex array.
 *
 * @param array - The complex array to be destroyed.
 */
void arrayComplexDestroy(ArrayComplexPtr complexArray);
/**
 * arrayComplexAddData:
 *
 * 		Adds a new data to the given complex array.
 *
 * @param complexArray - The complex array to add the data to
 * @param data - The data to be added
 * @param dataLen - The data length
 * @return
 * 	ARRAY_NULL_ARGS - If at least one of the parameters is NULL
 *	ARRAY_NOT_VALID_ARGS - If at least one of the parameters is not valid
 *	ARRAY_OUT_OF_MEMORY - If an allocation failed
 *	ARRAY_FULL - If the complex array is full
 * 	ARRAY_SUCCESS - If the function succeeded to add the new data
 */
ArrayMessage arrayComplexAddData(ArrayComplexPtr complexArray,
		unsigned char* data, int dataLen);
/**
 * arrayGetData: copy the array data to destination array.
 *
 * @param complexArray - The complex array to copy the data from.
 * @param index - The index of the data to be copied.
 * @param dstLen - The length of the destination array.
 * @param dst - The destination array to copy the element to.
 * @return
 * 	ARRAY_NULL_ARGS - If at least one of the parameters is NULL
 * 	ARRAY_NO_ELEMENT - If the element doesn't exist in the block
 * 	ARRAY_NOT_RIGHT_SIZE - If size of the output array doesn't match
 * 		the element length
 * 	ARRAY_SUCCESS - If the function succeeded to copy the data
 */
ArrayMessage arrayComplexGetData(ArrayComplexPtr complexArray, int index,
		unsigned char* dst, int dstLen);
/**
 * arrayGetLen: Gets the length of the an array data.
 *
 * @param complexArray - The complex array to get the length of the data from.
 * @param index - The index of the data to get the length from.
 * @return
 * 	the data length
 */
int arrayComplexGetLen(ArrayComplexPtr complexArray, int index);

#endif /* ARRAY_H_ */
