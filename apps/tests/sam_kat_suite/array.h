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
