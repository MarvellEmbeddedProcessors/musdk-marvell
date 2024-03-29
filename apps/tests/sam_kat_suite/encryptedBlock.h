/* Copyright (c) 2019 Marvell.
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ENCRYPTEDBLOCK_H_
#define ENCRYPTEDBLOCK_H_

#include <stdbool.h>
#include "array.h"

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
	TEXT_LEN,
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
		(type == AUTH_OFFSET) ||
		(type == TEXT_LEN)
		);
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
 * operation: Structure for operation,
 *            with all of its relevant information.
 *
 * iv           - The initial vector for the operation.
 * aad          - The additional authentication data for the operation.
 * icb          - The digest for the operation.
 * textLen      - The plain/cipher text length. If 0 - use sizeof(plainText).
 * plainText    - The plain text for the operation. If NULL use random.
 * cipherText   - The cipher text for the operation. If NULL use random.
 * cryptoOffset - The offset to start encryption.
 * authOffset   - The offset to start authentication.
 */
struct operation {
	ArrayPtr iv;
	ArrayPtr aad;
	ArrayPtr icb;
	ArrayPtr plainText;
	ArrayPtr cipherText;
	int textLen;
	int cryptoOffset;
	int authOffset;
};
/**
 * encryptedBlock_t: Structure for encrypted block,
 *                   with all of its relevant information.
 *
 * algorithm        - The algorithm name.
 * mode             - The mode name.
 * authAlgorithm    - The authentication algorithm name.
 * direction        - The direction name.
 * name             - The name of the encrypted block.
 * key              - The key of the encrypted block.
 * authKey          - The authentication key of the encrypted block.
 * operationCounter - The number of operations.
 * operations       - the operations of the current encrypted block.
 * testCounter      - The number of loop test for each operation.
 */
struct encryptedBlock_t {
	char algorithm[16];
	char mode[16];
	char authAlgorithm[16];
	char direction[16];
	char name[64];
	ArrayPtr key;
	ArrayPtr authKey;
	int operationCounter;
	struct operation *operations[16];
	int testCounter;
};

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

bool encryptedBlockOperationExist(EncryptedBlockPtr encryptedBlock, int index);

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
