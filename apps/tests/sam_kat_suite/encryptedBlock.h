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

#ifndef ENCRYPTEDBLOCK_H_
#define ENCRYPTEDBLOCK_H_

/** Type for defining the encrypted block */
typedef struct encryptedBlock_t* EncryptedBlockPtr;

/** Type for defining the encrypted block type of element */
typedef enum {
	INVALID,
	NAME,
	KEY,
	AUTH_KEY,
	IV,
	AAD,
	ICB,
	PLAINTEXT,
	CIPHERTEXT,
	TESTCOUNTER,
	ALGORITHM,
	MODE,
	AUTH_ALGORITHM,
	DIRECTION,
	CRYPTO_OFFSET,
	AUTH_OFFSET,
	LAST
} EncryptedBlockType;

/** The type that declares on a new session block */
#define NEW_BLOCK_TYPE		ALGORITHM

/** The type that declares on a new operation block */
#define NEW_OPERATION_TYPE	PLAINTEXT

static inline int isEncryptedBlockTypeSession(EncryptedBlockType type)
{
	return ((type == NAME) ||
		(type == KEY) ||
		(type == AUTH_KEY) ||
		(type == TESTCOUNTER) ||
		(type == ALGORITHM) ||
		(type == AUTH_ALGORITHM) ||
		(type == DIRECTION) ||
		(type == MODE));
}

static inline int isEncryptedBlockTypeOperation(EncryptedBlockType type)
{
	return ((type == IV) ||
		(type == AAD) ||
		(type == ICB) ||
		(type == PLAINTEXT) ||
		(type == CIPHERTEXT) ||
		(type == CRYPTO_OFFSET) ||
		(type == AUTH_OFFSET));
}

/** Type used for returning message from encrypted block functions */
typedef enum {
	ENCRYPTEDBLOCK_NULL_ARGS,
	ENCRYPTEDBLOCK_SUCCESS,
	ENCRYPTEDBLOCK_NOT_VALID,
	ENCRYPTEDBLOCK_NOT_VALID_TYPE,
	ENCRYPTEDBLOCK_NOT_VALID_ARGS,
	ENCRYPTEDBLOCK_NO_ELEMENT,
	ENCRYPTEDBLOCK_NOT_RIGHT_SIZE,
	ENCRYPTEDBLOCK_OUT_OF_MEMORY,
	ENCRYPTEDBLOCK_TOO_BIG_STRING,
	ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST,
	ENCRYPTEDBLOCK_NEGATIVE_COUNTER,
	ENCRYPTEDBLOCK_NOT_ENOUGH_OPERATIONS
} EncryptedBlockMessage;

/**
 * encryptedBlockCreate: Creates a new encrypted block.
 *
 * @param encryptedBlock - The encrypted block to be created.
 * @return
 *	ENCRYPTEDBLOCK_NULL_ARGS - If at least one of the parameters is NULL
 * 	ENCRYPTEDBLOCK_OUT_OF_MEMORY - If an allocation failed
 * 	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded to create a new block
 */
EncryptedBlockMessage encryptedBlockCreate(EncryptedBlockPtr *encryptedBlock);
/**
 * encryptedBlockCopy: Creates a copy of the encrypted block.
 *
 * @param srcBlock - The encrypted block to be copied.
 * @param copyBlock - pointer to the copied encrypted block.
 * 	This is an output parameter.
 * @return
 * 	ENCRYPTEDBLOCK_NULL_ARGS - If at least one of the parameters is NULL
 * 	ENCRYPTEDBLOCK_OUT_OF_MEMORY - If an allocation failed
 * 	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded to copy the block
 */
EncryptedBlockMessage encryptedBlockCopy(EncryptedBlockPtr srcBlock,
		EncryptedBlockPtr *copyBlock);

/**
 * encryptedBlockSessionAddElement: Adds an element to the encryptedBlock.
 *
 * @param encryptedBlock - The encrypted block to add the element to.
 * @param encryptedBlockType - The type of the element.
 * @param element - The element array to be added.
 * @param elementLen - The lengh of the element array.
 * @return
 * 	ENCRYPTEDBLOCK_NULL_ARGS - If at least one of the parameters is NULL
 * 	ENCRYPTEDBLOCK_OUT_OF_MEMORY - If an allocation failed
 * 	ENCRYPTEDBLOCK_NOT_VALID_TYPE - If the type is not valid encrypted block type
 *	ENCRYPTEDBLOCK_NOT_VALID_ARGS - If at least one of the parameters is not valid
 * 	ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST: If the specified type of element
 * 		already inserted to the block
 *	ENCRYPTEDBLOCK_NOT_ENOUGH_OPERATIONS - If the array is full
 * 	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded to add the element
 */
EncryptedBlockMessage encryptedBlockSessionAddElement(EncryptedBlockPtr encryptedBlock,
		EncryptedBlockType encryptedBlockType, unsigned char *elementIntArray,
		int elementLen);


EncryptedBlockMessage encryptedBlockOperationCreate(EncryptedBlockPtr encryptedBlock,
						int index);

/**
 * encryptedBlockOperationAddElement: Adds an element to the encryptedBlock.
 *
 * @param encryptedBlock - The encrypted block to add the element to.
 * @param encryptedBlockType - The type of the element.
 * @param element - The element array to be added.
 * @param elementLen - The length of the element array.
 * @return
 *	ENCRYPTEDBLOCK_NULL_ARGS - If at least one of the parameters is NULL
 *	ENCRYPTEDBLOCK_OUT_OF_MEMORY - If an allocation failed
 *	ENCRYPTEDBLOCK_NOT_VALID_TYPE - If the type is not valid encrypted block type
 *	ENCRYPTEDBLOCK_NOT_VALID_ARGS - If at least one of the parameters is not valid
 *	ENCRYPTEDBLOCK_ELEMENT_ALREADY_EXIST: If the specified type of element
 *		already inserted to the block
 *	ENCRYPTEDBLOCK_NOT_ENOUGH_OPERATIONS - If the array is full
 *	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded to add the element
 */
EncryptedBlockMessage encryptedBlockOperationAddElement(EncryptedBlockPtr encryptedBlock,
		EncryptedBlockType encryptedBlockType, int index,
		unsigned char *elementIntArray, int elementLen);

/**
 * encryptedBlockDestroy: Destroys the given encrypted block.
 *
 * @param encryptedBlock - The encrypted block to be destroyed.
 */
void encryptedBlockDestroy(EncryptedBlockPtr encryptedBlock);

/**
 * encryptedBlockGet(ELEMENT): copy the encrypted block element to the
 * 		output array.
 *
 * @param encryptedBlock - The encrypted block to copy the element from.
 * @param size - The size of the output array.
 * @param outputArray - The output array to copy the element to.
 * only for cipher text and plain text:
 * (@param index - The index of the array to be copied)
 * @return
 * 	ENCRYPTEDBLOCK_NULL_ARGS - If at least one of the parameters is NULL
 * 	ENCRYPTEDBLOCK_NO_ELEMENT - If the element doesn't exist in the block
 * 	ENCRYPTEDBLOCK_NOT_RIGHT_SIZE - If size of the output array doesn't match
 * 		the element length
 * 	ENCRYPTEDBLOCK_SUCCESS - If the function succeeded to copy the element
 */
EncryptedBlockMessage encryptedBlockGetKey(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char *outputArray);
EncryptedBlockMessage encryptedBlockGetAuthKey(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char *outputArray);
EncryptedBlockMessage encryptedBlockGetIv(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char *outputArray, int index);
EncryptedBlockMessage encryptedBlockGetIcb(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char *outputArray, int index);
EncryptedBlockMessage encryptedBlockGetAad(EncryptedBlockPtr encryptedBlock,
		int size, unsigned char *outputArray, int index);
EncryptedBlockMessage encryptedBlockGetPlainText(
		EncryptedBlockPtr encryptedBlock, int size, unsigned char *outputArray,
		int index);
EncryptedBlockMessage encryptedBlockGetCipherText(
		EncryptedBlockPtr encryptedBlock, int size, unsigned char *outputArray,
		int index);

/**
 * encryptedBlockGet(ELEMENT)Len: Gets the number of bytes of the encrypted block element.
 *
 * @param encryptedBlock - The encrypted block to get the length of the element
 * 	 from.
 * (@param index - The index of the wanted array)
 * @return
 * 	the element length
 */
int encryptedBlockGetKeyLen(EncryptedBlockPtr encryptedBlock);
int encryptedBlockGetAuthKeyLen(EncryptedBlockPtr encryptedBlock);
int encryptedBlockGetIvLen(EncryptedBlockPtr encryptedBlock, int index);
int encryptedBlockGetIcbLen(EncryptedBlockPtr encryptedBlock, int index);
int encryptedBlockGetAadLen(EncryptedBlockPtr encryptedBlock, int index);
int encryptedBlockGetPlainTextLen(EncryptedBlockPtr encryptedBlock, int index);
int encryptedBlockGetCipherTextLen(EncryptedBlockPtr encryptedBlock, int index);
/**
 * encryptedBlockGet(ELEMENT): Gets the numeric value of the encrypted block element.
 *
 * @param encryptedBlock - The encrypted block to get the length of the element
 * 	 from.
 * (@param index - The index of the wanted array)
 * @return
 * 	the element value
 */
int encryptedBlockGetTestCounter(EncryptedBlockPtr encryptedBlock);
int encryptedBlockGetCryptoOffset(EncryptedBlockPtr encryptedBlock, int index);
int encryptedBlockGetAuthOffset(EncryptedBlockPtr encryptedBlock, int index);

/**
 * encryptedBlockGet(ELEMENT): Gets the encrypted block element.
 *
 * @param encryptedBlock - The encrypted block to get the element from.
 * @return
 * 	the element type
 */
char *encryptedBlockGetName(EncryptedBlockPtr encryptedBlock);
char *encryptedBlockGetMode(EncryptedBlockPtr encryptedBlock);
char *encryptedBlockGetAlgorithm(
		EncryptedBlockPtr encryptedBlock);
char *encryptedBlockGetAuthAlgorithm(
		EncryptedBlockPtr encryptedBlock);
char *encryptedBlockGetDirection(
		EncryptedBlockPtr encryptedBlock);

#endif /* ENCRYPTEDBLOCK_H_ */
