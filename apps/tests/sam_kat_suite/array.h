/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ARRAY_H_
#define ARRAY_H_

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
ArrayPtr arrayCreate(unsigned char *data, int len);
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
 * @return - number of copied bytes
 */
int arrayGetData(ArrayPtr array, unsigned char *dst, int dstLen);
/**
 * arrayGetLen: Gets the length of the array data.
 *
 * @param array - The array to get the length of the data from.
 * @return
 * 	the data length
 */
int arrayGetLen(ArrayPtr array);

#endif /* ARRAY_H_ */
