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
